// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::f64::consts;

use nalgebra::{vector, Point2, Point3, Rotation3, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, PartialEq, Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum Printer {
    ThreeDoF {
        center: Point2<f64>,
    },
    DancingBed {
        tilt: f64,
        pivot: Point3<f64>,
        rot_offset: Vector3<f64>,
    },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum PrinterType {
    /// Conventional 3D printer with 3-DoF toolhead movement
    ThreeDoF,
    DancingBed,
}

#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub struct GenericCoords {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub a: Option<f64>,
    pub b: Option<f64>,
    pub c: Option<f64>,
}

impl Printer {
    pub fn printer_type(&self) -> PrinterType {
        match self {
            &Printer::ThreeDoF { .. } => PrinterType::ThreeDoF,
            &Printer::DancingBed { .. } => PrinterType::DancingBed,
        }
    }

    pub fn calc_ik(&self, coords: GenericCoords) -> GenericCoords {
        match self {
            &Printer::ThreeDoF { .. } => coords,
            &Printer::DancingBed {
                tilt,
                pivot,
                rot_offset,
            } => {
                // Rotation in x-y-z (roll-pitch-yaw) order
                let offset_rot =
                    Rotation3::from_euler_angles(rot_offset.x, rot_offset.y, rot_offset.z);

                let c = coords.c.expect("C coord is necessary");
                let c_rad = c * consts::PI / 180.0;
                let c_rot = Rotation3::from_axis_angle(&Vector3::z_axis(), c_rad);

                let tilt_rad = tilt * consts::PI / 180.0;
                let tilt_rot = Rotation3::from_axis_angle(&Vector3::x_axis(), tilt_rad);

                let rotated = offset_rot * c_rot * tilt_rot * vector![coords.x, coords.y, coords.z];

                GenericCoords {
                    x: rotated.x + pivot.x,
                    y: rotated.y + pivot.y,
                    z: rotated.z + pivot.z,
                    c: Some(c + rot_offset.z),
                    ..Default::default()
                }
            }
        }
    }

    pub fn center(&self) -> Point2<f64> {
        match self {
            &Printer::ThreeDoF { center } => center,
            &Printer::DancingBed { .. } => Point2::origin(),
        }
    }
}
