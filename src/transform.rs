// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use clap::ValueEnum;
use na::{vector, Vector3};
use nalgebra as na;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

use crate::utils::Aabb;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct TransformData {
    pub transform: Transform,
    pub warped_aabb: Aabb,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum)]
pub enum TransformType {
    Conical,
    Sinusoidal,
    Spherical,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum Transform {
    /// z' = z + tan(slope_angle)*sqrt(x^2 + y^2)
    Conical { slope_angle: f64, flat_bottom: f64 },
    /// z' = z + height*(sin(2*π*x/pitch)*cos(2*π*y/pitch)+1)/2
    Sinusoidal {
        height: f64,
        pitch: f64,
        flat_bottom: f64,
    },
    /// z' = z + r - sqrt(r^2 - x^2 - y^2) (for r>0)
    Spherical { radius: f64, flat_bottom: f64 },
}

impl Transform {
    pub fn apply(&self, point: Vector3<f64>) -> Vector3<f64> {
        match self {
            &Transform::Conical {
                slope_angle,
                flat_bottom,
            } => {
                vector![
                    point.x,
                    point.y,
                    apply_flat_bottom(
                        point.z,
                        conical_offset(point.x, point.y, slope_angle),
                        flat_bottom
                    )
                ]
            }
            &Transform::Sinusoidal {
                height,
                pitch,
                flat_bottom,
            } => {
                vector![
                    point.x,
                    point.y,
                    apply_flat_bottom(
                        point.z,
                        sinusoidal_offset(point.x, point.y, height, pitch),
                        flat_bottom
                    )
                ]
            }
            &Transform::Spherical {
                radius,
                flat_bottom,
            } => {
                vector![
                    point.x,
                    point.y,
                    apply_flat_bottom(
                        point.z,
                        spherical_offset(point.x, point.y, radius),
                        flat_bottom
                    )
                ]
            }
        }
    }

    pub fn apply_inverse(&self, point: Vector3<f64>) -> Vector3<f64> {
        match self {
            &Transform::Conical {
                slope_angle,
                flat_bottom,
            } => {
                vector![
                    point.x,
                    point.y,
                    apply_flat_bottom_inverse(
                        point.z,
                        conical_offset(point.x, point.y, slope_angle),
                        flat_bottom
                    )
                ]
            }
            &Transform::Sinusoidal {
                height,
                pitch,
                flat_bottom,
            } => {
                vector![
                    point.x,
                    point.y,
                    apply_flat_bottom_inverse(
                        point.z,
                        sinusoidal_offset(point.x, point.y, height, pitch),
                        flat_bottom
                    )
                ]
            }
            &Transform::Spherical {
                radius,
                flat_bottom,
            } => {
                vector![
                    point.x,
                    point.y,
                    apply_flat_bottom_inverse(
                        point.z,
                        spherical_offset(point.x, point.y, radius),
                        flat_bottom
                    )
                ]
            }
        }
    }

    /// Jacobian determinant of forward transform i.e. Ratio of volume magnification
    pub fn jacobian(&self, point: Vector3<f64>) -> f64 {
        match self {
            &Transform::Conical {
                slope_angle,
                flat_bottom,
            } => jacobian_flat_bottom(
                point.z,
                conical_offset(point.x, point.y, slope_angle),
                flat_bottom,
            ),
            &Transform::Sinusoidal {
                height,
                pitch,
                flat_bottom,
            } => jacobian_flat_bottom(
                point.z,
                sinusoidal_offset(point.x, point.y, height, pitch),
                flat_bottom,
            ),
            &Transform::Spherical {
                radius,
                flat_bottom,
            } => jacobian_flat_bottom(
                point.z,
                spherical_offset(point.x, point.y, radius),
                flat_bottom,
            ),
        }
    }
}

fn conical_offset(x: f64, y: f64, slope_angle: f64) -> f64 {
    let s = slope_angle.tan();
    s * (x * x + y * y).sqrt()
}

fn sinusoidal_offset(x: f64, y: f64, height: f64, pitch: f64) -> f64 {
    height * ((2.0 * PI * x / pitch).sin() * (2.0 * PI * y / pitch).cos() + 1.0) / 2.0
}

fn spherical_offset(x: f64, y: f64, radius: f64) -> f64 {
    radius - (radius * radius - x * x - y * y).sqrt()
}

fn apply_flat_bottom(z: f64, offset: f64, flat_bottom: f64) -> f64 {
    let strength = if flat_bottom != 0.0 {
        (z / flat_bottom).min(1.0)
    } else {
        1.0
    };

    z + strength * offset
}

fn apply_flat_bottom_inverse(z: f64, offset: f64, flat_bottom: f64) -> f64 {
    if flat_bottom != 0.0 && z <= flat_bottom + offset {
        (flat_bottom / (flat_bottom + offset)) * z
    } else {
        z - offset
    }
}

fn jacobian_flat_bottom(z: f64, offset: f64, flat_bottom: f64) -> f64 {
    // Derivative of `strength` respect to z
    let strength_deriv = if flat_bottom != 0.0 && z <= flat_bottom {
        1.0 / flat_bottom
    } else {
        0.0
    };

    1.0 + strength_deriv * offset
}
