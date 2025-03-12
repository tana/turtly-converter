// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::{
    ffi::OsString,
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
};

use anyhow::Result;
use clap::Args;
use na::{vector, Vector3, Vector4};
use nalgebra as na;

use crate::gcode::command::BEGIN_DEWARP;
use crate::{
    gcode::{
        command::{Command, G0, G1, G92},
        parser::parse_line,
    },
    transform::{Transform, TransformData},
};

const DEFAULT_MAX_LINE_LEN: f64 = 1.0; // 1 mm

#[derive(Args)]
pub struct DewarpArgs {
    input_file: OsString,
    transform_file: OsString,
    #[arg(short, long, default_value_t = DEFAULT_MAX_LINE_LEN)]
    max_line_len: f64,
    #[arg(short, long)]
    output_file: Option<OsString>,
}

pub fn command_main(args: DewarpArgs) -> Result<()> {
    let input_path = Path::new(&args.input_file);
    let input_file = File::open(input_path)?;

    let transform_file_path = Path::new(&args.transform_file);
    let transform_file = File::open(transform_file_path)?;
    let TransformData {
        transform,
        warped_aabb,
    } = serde_json::de::from_reader(transform_file)?;

    let mut default_output_path = input_path.to_owned();
    default_output_path.set_extension("dewarped.gcode");

    let output_file = File::create(
        args.output_file
            .unwrap_or(default_output_path.as_os_str().to_owned()),
    )?;

    dewarp_gcode(
        input_file,
        output_file,
        transform,
        args.max_line_len,
        warped_aabb.origin.z,
    )?;

    Ok(())
}

fn dewarp_gcode(
    input_file: File,
    output_file: File,
    transform: Transform,
    max_line_len: f64,
    z_offset: f64,
) -> Result<()> {
    let mut writer = BufWriter::new(output_file);

    let mut enabled = false;
    let mut center = Vector3::zeros();
    let mut last_pos = Vector4::zeros();
    let mut corrected_e = 0.0;

    for line in BufReader::new(input_file).lines() {
        let line = line?;

        if let Ok((_, Some(cmd))) = parse_line(&line) {
            match cmd {
                Command::G0(G0 { x, y, z, e, .. })
                | Command::G1(G1 { x, y, z, e, .. }) => {
                    let pos = vector![
                        x.unwrap_or(last_pos.x),
                        y.unwrap_or(last_pos.y),
                        z.map(|z| if enabled { z + z_offset } else { z })
                            .unwrap_or(last_pos.z),
                        e.unwrap_or(last_pos[3])
                    ];

                    if enabled {
                        let mut last_e = last_pos[3];
                        // Split movement into short parts because it may be nonlinear after dewarping
                        for p in interpolate(&last_pos, &pos, max_line_len) {
                            let dewarped = dewarp_point(p.xyz(), transform, center);
                            // Correct extrusion length using the inverse of Jacobian determinant
                            corrected_e += (p[3] - last_e) / extrusion_correction(p.xyz(), transform, center);
                            last_e = p[3];

                            let z = dewarped.z.max(0.0); // Workaround for initial moves
                            match cmd {
                                Command::G0(ref cmd) => writeln!(
                                    &mut writer,
                                    "{}",
                                    (G0 {
                                        x: Some(dewarped.x),
                                        y: Some(dewarped.y),
                                        z: Some(z),
                                        e: Some(corrected_e),
                                        ..cmd.clone()
                                    })
                                    .to_string()
                                )?,
                                Command::G1(ref cmd) => writeln!(
                                    &mut writer,
                                    "{}",
                                    (G1 {
                                        x: Some(dewarped.x),
                                        y: Some(dewarped.y),
                                        z: Some(z),
                                        e: Some(corrected_e),
                                        ..cmd.clone()
                                    })
                                    .to_string()
                                )?,
                                _ => unreachable!()
                            }
                        }
                    } else {
                        writeln!(&mut writer, "{}", line)?;
                    }

                    last_pos = pos;
                }
                Command::G92(G92 { x, y, z, e, .. }) => {
                    let pos = vector![
                        x.unwrap_or(last_pos.x),
                        y.unwrap_or(last_pos.y),
                        z.unwrap_or(last_pos.z),
                        e.unwrap_or(last_pos[3])
                    ];

                    // TODO:
                    if pos[3].abs() > 0.0 {
                        panic!("G92 to |E|>0 is currently not supported");
                    }
                    corrected_e = pos[3];

                    writeln!(&mut writer, "{}", line)?;

                    last_pos = pos;
                }
                Command::BEGIN_DEWARP(BEGIN_DEWARP { x, y }) => {
                    enabled = true;
                    center = vector![x.unwrap_or(0.0) / 2.0, y.unwrap_or(0.0) / 2.0, 0.0];
                }
                Command::END_DEWARP(_) => {
                    enabled = false;
                }
            }
        } else {
            // Unrecognized or comment-only line is left unchanged
            writeln!(&mut writer, "{}", line)?;
        }
    }

    Ok(())
}

fn dewarp_point(point: Vector3<f64>, transform: Transform, center: Vector3<f64>) -> Vector3<f64> {
    transform.apply_inverse(point - center) + center
}

fn extrusion_correction(point: Vector3<f64>, transform: Transform, center: Vector3<f64>) -> f64 {
    transform.jacobian(transform.apply_inverse(point - center))
}

fn interpolate(from: &Vector4<f64>, to: &Vector4<f64>, max_step: f64) -> Vec<Vector4<f64>> {
    let distance = (to.xyz() - from.xyz()).norm();
    let div = ((distance / max_step).floor() as usize).max(1);

    (1..=div)
        .map(move |i| {
            let t = (i as f64) / (div as f64);
            from.lerp(&to, t)
        })
        .collect()
}
