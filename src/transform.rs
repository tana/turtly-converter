use clap::ValueEnum;
use na::{vector, Vector3};
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum)]
pub enum TransformType {
    Conical,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum Transform {
    Conical { slope_angle: f64 },
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
        }
    }
}
