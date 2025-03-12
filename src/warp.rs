// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::{ffi::OsString, fs::File, path::Path};

use anyhow::Result;
use clap::Args;
use na::{vector, Vector3};
use nalgebra as na;
use stl_io::{IndexedMesh, Triangle};

use crate::{
    tessellation::tesselate,
    transform::{Transform, TransformData, TransformType},
    utils::{Aabb, Mesh},
};

const DEFAULT_MAX_EDGE_LEN: f64 = 1.0; // 1 mm
const DEFAULT_TYPE: TransformType = TransformType::Conical;
const DEFAULT_SLOPE_ANGLE: f64 = 30.0; // degrees
const DEFAULT_HEIGHT: f64 = 2.0; // mm
const DEFAULT_PITCH: f64 = 10.0; // mm
const DEFAULT_FLAT_BOTTOM: f64 = 0.0; // mm

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
    #[arg(long, default_value_t = DEFAULT_FLAT_BOTTOM)]
    flat_bottom: f64,
}

pub fn command_main(args: WarpArgs) -> Result<()> {
    let input_path = Path::new(&args.input_file);
    let input_mesh = stl_io::read_stl(&mut File::open(input_path)?)?.into();
    let Aabb { origin, size } = calc_aabb(&input_mesh);
    let center = vector![origin.x + size.x / 2.0, origin.y + size.y / 2.0, origin.z];

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
    };

    let tesselated_mesh = tesselate(input_mesh, args.max_edge_len);

    let warped_mesh = warp_mesh(tesselated_mesh, transform, center);

    let warped_aabb = calc_aabb(&warped_mesh);

    let mut default_output_path = input_path.to_owned();
    default_output_path.set_extension("warped.stl");
    let output_path = match args.output_file {
        Some(output_path) => output_path.into(),
        None => default_output_path,
    };

    let mut transform_file_path = input_path.to_owned();
    transform_file_path.set_extension("transform.json");

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

fn calc_aabb(input: &Mesh) -> Aabb {
    let mut min = Vector3::from_element(std::f64::MAX);
    let mut max = Vector3::from_element(std::f64::MIN);

    for vert in input.vertices.iter() {
        min = min.map_with_location(|i, _, e: f64| e.min(vert[i]));
        max = max.map_with_location(|i, _, e: f64| e.max(vert[i]));
    }

    Aabb {
        origin: min,
        size: max - min,
    }
}

fn warp_mesh(input: Mesh, transform: Transform, center: Vector3<f64>) -> Mesh {
    let vertices = input
        .vertices
        .into_iter()
        .map(|vert| transform.apply(vert - center))
        .collect();

    Mesh {
        vertices,
        triangles: input.triangles,
    }
}
