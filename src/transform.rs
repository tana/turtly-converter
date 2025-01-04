// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use clap::ValueEnum;
use na::{vector, Vector3};
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum)]
pub enum TransformType {
    Conical,
    Sinusoidal,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum Transform {
    Conical { slope_angle: f64 },
    Sinusoidal { height: f64, pitch: f64 },
}

impl Transform {
    pub fn apply(&self, point: Vector3<f64>) -> Vector3<f64> {
        match self {
            &Transform::Conical { slope_angle } => {
                let s = slope_angle.tan();
                vector![
                    point.x,
                    point.y,
                    point.z + s * (point.x * point.x + point.y * point.y).sqrt()
                ]
            }
            &Transform::Sinusoidal { height, pitch } => {
                vector![
                    point.x,
                    point.y,
                    point.z
                        + height
                            * ((2.0 * PI * point.x / pitch).sin()
                                * (2.0 * PI * point.y / pitch).cos()
                                + 1.0)
                            / 2.0
                ]
            }
        }
    }

    pub fn apply_inverse(&self, point: Vector3<f64>) -> Vector3<f64> {
        match self {
            &Transform::Conical { slope_angle } => {
                let s = slope_angle.tan();
                vector![
                    point.x,
                    point.y,
                    point.z - s * (point.x * point.x + point.y * point.y).sqrt()
                ]
            }
            &Transform::Sinusoidal { height, pitch } => {
                vector![
                    point.x,
                    point.y,
                    point.z
                        - height
                            * ((2.0 * PI * point.x / pitch).sin()
                                * (2.0 * PI * point.y / pitch).cos()
                                + 1.0)
                            / 2.0
                ]
            }
        }
    }
}
