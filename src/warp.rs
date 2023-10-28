use std::{fs::File, path::Path, ffi::OsString};

use clap::Args;
use anyhow::Result;
use nalgebra as na;
use na::{vector, Vector3};
use stl_io::{IndexedMesh, Triangle};

use crate::{tessellation::tesselate, utils::{to_na, from_na}};

const DEFAULT_MAX_EDGE_LEN: f32 = 1.0;    // 1 mm
const DEFAULT_SLOPE_ANGLE: f32 = 30.0;  // degrees

#[derive(Args)]
pub struct WarpArgs {
    input_file: OsString,
    #[arg(short, long)]
    output_file: Option<OsString>,
    #[arg(short, long, default_value_t = DEFAULT_MAX_EDGE_LEN)]
    max_edge_len: f32,
    #[arg(short, long, default_value_t = DEFAULT_SLOPE_ANGLE)]
    slope_angle: f32,
}

pub fn command_main(args: WarpArgs) -> Result<()> {
    let input_path = Path::new(&args.input_file);
    let input_mesh = stl_io::read_stl(&mut File::open(input_path)?)?;
    let (origin, size) = calc_aabb(&input_mesh);
    let center = vector![origin.x + size.x / 2.0, origin.y + size.y / 2.0, origin.z];

    let tesselated_mesh = tesselate(input_mesh, args.max_edge_len);

    let warped_mesh = warp_mesh(tesselated_mesh, args.slope_angle * std::f32::consts::PI / 180.0, &center);

    let mut default_output_path = input_path.to_owned();
    default_output_path.set_extension("warped.stl");

    let mut output_file = File::create(args.output_file.unwrap_or(default_output_path.as_os_str().to_owned()))?;
    stl_io::write_stl(&mut output_file, unindex_mesh(warped_mesh).iter())?;

    println!("Dewarp options: --slope-angle={}", args.slope_angle);

    Ok(())
}

fn unindex_mesh(mesh: IndexedMesh) -> Vec<Triangle> {
    mesh.faces.iter().map(|triangle| {
        Triangle {
            normal: triangle.normal,
            vertices: triangle.vertices.map(|i| mesh.vertices[i])
        }
    }).collect()
}

fn calc_aabb(input: &IndexedMesh) -> (Vector3<f32>, Vector3<f32>) {
    let mut min = Vector3::from_element(std::f32::MAX);
    let mut max = Vector3::from_element(std::f32::MIN);

    for vert in input.vertices.iter() {
        min = min.map_with_location(|i, _, e: f32| e.min(vert[i]));
        max = max.map_with_location(|i, _, e: f32| e.max(vert[i]));
    }

    (min, max - min)
}

fn warp_mesh(input: IndexedMesh, slope_angle: f32, center: &Vector3<f32>) -> IndexedMesh {
    let s = slope_angle.tan();

    let vertices = input.vertices.into_iter().map(to_na).map(|vert| {
        let vert = vert - center;
        vector![
            vert.x,
            vert.y,
            vert.z + s * (vert.x * vert.x + vert.y * vert.y).sqrt()
        ]
    });

    IndexedMesh {
        vertices: vertices.map(from_na).collect(),
        faces: input.faces
    }
}