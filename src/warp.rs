// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::{f64::consts::FRAC_PI_2, ffi::OsString, fs::File, path::Path};

use anyhow::Result;
use clap::{Args, ValueEnum};
use na::{vector, Vector3};
use nalgebra as na;
use ply_rs_bw::ply::{
    Addable as _, DefaultElement, ElementDef, Ply, Property, PropertyDef, PropertyType, ScalarType,
};
use stl_io::{IndexedMesh, Triangle};

use crate::{
    tessellation::tesselate,
    transform::{adaptive::fit_adaptive, Transform, TransformData, TransformType},
    utils::{parse_vector, Aabb, Mesh},
};

const DEFAULT_MAX_EDGE_LEN: f64 = 1.0; // 1 mm
const DEFAULT_TYPE: TransformType = TransformType::Conical;
const DEFAULT_SLOPE_ANGLE: f64 = 30.0; // degrees
const DEFAULT_HEIGHT: f64 = 2.0; // mm
const DEFAULT_PITCH: f64 = 10.0; // mm
const DEFAULT_RADIUS: f64 = 100.0; // mm
const DEFAULT_FLAT_BOTTOM: f64 = 0.0; // mm
const DEFAULT_NUM_ITER: usize = 300;
const DEFAULT_MINIBATCH_SIZE: usize = 100;
const DEFAULT_MAX_ANGLE: f64 = 45.0; // degrees
const DEFAULT_JACOBIAN_LOSS_WEIGHT: f64 = 0.01;
const DEFAULT_L_INFINITY_LOSS_WEIGHT: f64 = 0.01;

#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum)]
enum VisualizationType {
    /// Z coordinate after transformation
    PostZ,
    /// Overhang angle before transformation
    PreOverhangAngle,
}

#[derive(Args)]
pub struct WarpArgs {
    input_file: OsString,
    #[arg(short, long)]
    output_file: Option<OsString>,
    #[arg(short, long, default_value_t = DEFAULT_MAX_EDGE_LEN)]
    max_edge_len: f64,
    #[arg(short = 't', long = "type", value_enum, default_value_t = DEFAULT_TYPE)]
    transform_type: TransformType,
    #[arg(short, long, default_value_t = DEFAULT_SLOPE_ANGLE)]
    slope_angle: f64,
    #[arg(short = 'H', long, default_value_t = DEFAULT_HEIGHT)]
    height: f64,
    #[arg(short, long, default_value_t = DEFAULT_PITCH)]
    pitch: f64,
    #[arg(short, long, default_value_t = DEFAULT_RADIUS)]
    radius: f64,
    #[arg(long, default_value_t = DEFAULT_FLAT_BOTTOM)]
    flat_bottom: f64,
    #[arg(short, long, value_parser = parse_vector)]
    center: Option<Vector3<f64>>,
    #[arg(long, value_enum)]
    visualize: Option<VisualizationType>,
    #[arg(long, default_value_t = DEFAULT_NUM_ITER)]
    num_iter: usize,
    #[arg(long, default_value_t = DEFAULT_MINIBATCH_SIZE)]
    minibatch_size: usize,
    #[arg(long, default_value_t = DEFAULT_MAX_ANGLE)]
    max_angle: f64,
    #[arg(long, default_value_t = DEFAULT_JACOBIAN_LOSS_WEIGHT)]
    jacobian_loss_weight: f64,
    #[arg(long, default_value_t = DEFAULT_L_INFINITY_LOSS_WEIGHT)]
    l_infinity_loss_weight: f64,
}

pub fn command_main(args: WarpArgs) -> Result<()> {
    let input_path = Path::new(&args.input_file);
    let input_mesh: Mesh = stl_io::read_stl(&mut File::open(input_path)?)?.into();
    let Aabb { origin, size } = input_mesh.calc_aabb();
    let center = args.center.unwrap_or(vector![
        origin.x + size.x / 2.0,
        origin.y + size.y / 2.0,
        origin.z
    ]);

    let transform = match args.transform_type {
        TransformType::Conical => {
            // TODO:
            if args.slope_angle < 0.0 && args.flat_bottom != 0.0 {
                panic!("Flat bottom is not supported for negative slope angle");
            }
            Transform::Conical {
                slope_angle: args.slope_angle * std::f64::consts::PI / 180.0,
                flat_bottom: args.flat_bottom,
            }
        }
        TransformType::Sinusoidal => Transform::Sinusoidal {
            height: args.height,
            pitch: args.pitch,
            flat_bottom: args.flat_bottom,
        },
        TransformType::Spherical => {
            // TODO:
            if args.radius < 0.0 {
                panic!("Only positive radius is supported");
            }
            Transform::Spherical {
                radius: args.radius,
                flat_bottom: args.flat_bottom,
            }
        }
        TransformType::Adaptive => Transform::Adaptive(
            fit_adaptive(
                &input_mesh,
                &center,
                args.num_iter,
                args.minibatch_size,
                args.max_angle * std::f64::consts::PI / 180.0,
                args.jacobian_loss_weight,
                args.l_infinity_loss_weight,
            )
            .expect("Fitting failed"),
        ),
    };

    let tesselated_mesh = tesselate(input_mesh, args.max_edge_len);

    let warped_mesh = warp_mesh(&tesselated_mesh, &transform, center);

    let warped_aabb = warped_mesh.calc_aabb();

    let mut default_output_path = input_path.to_owned();
    default_output_path.set_extension("warped.stl");
    let output_path = match args.output_file {
        Some(output_path) => output_path.into(),
        None => default_output_path,
    };

    let mut transform_file_path = input_path.to_owned();
    transform_file_path.set_extension("transform.json");

    if let Some(viz_type) = args.visualize {
        let mut visualization_file_path = input_path.to_owned();
        visualization_file_path.set_extension("visualization.ply");

        let mut ply = visualize(&tesselated_mesh, &transform, center, viz_type);
        let writer = ply_rs_bw::writer::Writer::new();

        let mut visualization_file = File::create(visualization_file_path)?;
        writer.write_ply(&mut visualization_file, &mut ply)?;
    }

    let mut output_file = File::create(output_path)?;
    stl_io::write_stl(&mut output_file, unindex_stl(warped_mesh.into()).iter())?;

    let transform_file = File::create(transform_file_path)?;
    serde_json::to_writer(
        transform_file,
        &TransformData {
            transform,
            warped_aabb,
        },
    )?;

    Ok(())
}

fn unindex_stl(mesh: IndexedMesh) -> Vec<Triangle> {
    mesh.faces
        .iter()
        .map(|triangle| Triangle {
            normal: triangle.normal,
            vertices: triangle.vertices.map(|i| mesh.vertices[i]),
        })
        .collect()
}

fn warp_mesh(input: &Mesh, transform: &Transform, center: Vector3<f64>) -> Mesh {
    let vertices = input
        .vertices
        .iter()
        .map(|vert| transform.apply(vert - center))
        .collect();

    Mesh {
        vertices,
        triangles: input.triangles.clone(),
    }
}

fn visualize(
    mesh: &Mesh,
    transform: &Transform,
    center: Vector3<f64>,
    viz_type: VisualizationType,
) -> Ply<DefaultElement> {
    let val: Vec<_> = match viz_type {
        VisualizationType::PostZ => {
            // Calculate z coordinates after transform
            mesh.vertices
                .iter()
                .map(|pos| transform.apply(pos - center).z)
                .collect()
        }
        VisualizationType::PreOverhangAngle => {
            // Calculate overhang angle (positive means overhang)
            mesh.calc_vert_normals()
                .iter()
                .map(|normal| normal.z.acos() - FRAC_PI_2)
                .collect()
        }
    };

    let val_max = val
        .clone()
        .into_iter()
        .reduce(f64::max)
        .unwrap_or(std::f64::MIN);
    let val_min = val
        .clone()
        .into_iter()
        .reduce(f64::min)
        .unwrap_or(std::f64::MAX);

    let mut ply = Ply::<DefaultElement>::new();

    // Define "vertex" element
    let mut vertex_element = ElementDef::new("vertex".into());
    vertex_element.properties.add(PropertyDef::new(
        "x".into(),
        PropertyType::Scalar(ScalarType::Float),
    ));
    vertex_element.properties.add(PropertyDef::new(
        "y".into(),
        PropertyType::Scalar(ScalarType::Float),
    ));
    vertex_element.properties.add(PropertyDef::new(
        "z".into(),
        PropertyType::Scalar(ScalarType::Float),
    ));
    vertex_element.properties.add(PropertyDef::new(
        "red".into(),
        PropertyType::Scalar(ScalarType::UChar),
    ));
    vertex_element.properties.add(PropertyDef::new(
        "green".into(),
        PropertyType::Scalar(ScalarType::UChar),
    ));
    vertex_element.properties.add(PropertyDef::new(
        "blue".into(),
        PropertyType::Scalar(ScalarType::UChar),
    ));
    ply.header.elements.add(vertex_element);

    // Define "face" element
    let mut face_element = ElementDef::new("face".into());
    face_element.properties.add(PropertyDef::new(
        "vertex_index".into(),
        PropertyType::List(ScalarType::UChar, ScalarType::Int),
    ));
    ply.header.elements.add(face_element);

    let colormap = match viz_type {
        VisualizationType::PostZ => colorous::TURBO,
        VisualizationType::PreOverhangAngle => colorous::BROWN_GREEN,
    };

    // Write vertices into PLY
    let vertices = mesh
        .vertices
        .iter()
        .zip(val.iter())
        .map(|(pos, val)| {
            let mut vertex = DefaultElement::new();

            // Vertex position
            vertex.insert("x".into(), Property::Float(pos.x as f32));
            vertex.insert("y".into(), Property::Float(pos.y as f32));
            vertex.insert("z".into(), Property::Float(pos.z as f32));

            // Color a vertex based on calculated value
            let color = colormap.eval_continuous((val - val_min) / (val_max - val_min));
            vertex.insert("red".into(), Property::UChar(color.r));
            vertex.insert("green".into(), Property::UChar(color.g));
            vertex.insert("blue".into(), Property::UChar(color.b));

            vertex
        })
        .collect();
    ply.payload.insert("vertex".into(), vertices);

    // Write faces into PLY
    let faces = mesh
        .triangles
        .iter()
        .map(|[v0, v1, v2]| {
            let mut face = DefaultElement::new();
            face.insert(
                "vertex_index".into(),
                Property::ListInt(vec![*v0 as i32, *v1 as i32, *v2 as i32]),
            );
            face
        })
        .collect();
    ply.payload.insert("face".into(), faces);

    ply
}
