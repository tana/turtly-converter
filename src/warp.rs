use std::{fs::File, path::Path, ffi::OsString};

use clap::Args;
use anyhow::Result;
use stl_io::{IndexedMesh, Triangle};

use crate::tessellation::tesselate;

const DEFAULT_MAX_EDGE_LEN: f32 = 1.0;    // 1 mm

#[derive(Args)]
pub struct WarpArgs {
    input_file: OsString,
    #[arg(short, long)]
    output_file: Option<OsString>,
    #[arg(short, long)]
    max_edge_len: Option<f32>
}

pub fn command_main(args: WarpArgs) -> Result<()> {
    let input_path = Path::new(&args.input_file);
    let input_mesh = stl_io::read_stl(&mut File::open(input_path)?)?;

    let tesselated_mesh = tesselate(input_mesh, args.max_edge_len.unwrap_or(DEFAULT_MAX_EDGE_LEN));

    let mut default_output_path = input_path.to_owned();
    default_output_path.set_extension("warped.stl");

    let mut output_file = File::create(args.output_file.unwrap_or(default_output_path.as_os_str().to_owned()))?;
    stl_io::write_stl(&mut output_file, unindex_mesh(tesselated_mesh).iter())?;

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

// fn warp_mesh(input: IndexedMesh) {

// }