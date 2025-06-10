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
    DifferentialTiltingBed {
        pivot: Point3<f64>,
        bed_pivot: f64,
        rot_offset: Vector3<f64>,
    },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum PrinterType {
    /// Conventional 3D printer with 3-DoF toolhead movement
    Xyz,
    /// XYZ + rotation around X and Y axes
    Xyzab,
}

#[derive(Clone, Copy, Default, PartialEq, Debug)]
pub struct GenericCoords {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub i: Option<f64>,
    pub j: Option<f64>,
    pub k: Option<f64>,
    pub a: Option<f64>,
    pub b: Option<f64>,
}

impl Printer {
    pub fn printer_type(&self) -> PrinterType {
        match self {
            &Printer::ThreeDoF { .. } => PrinterType::Xyz,
            &Printer::DifferentialTiltingBed { .. } => PrinterType::Xyzab,
        }
    }

    pub fn calc_ik(&self, coords: GenericCoords) -> GenericCoords {
        match self {
            &Printer::ThreeDoF { .. } => coords,
            &Printer::DifferentialTiltingBed {
                pivot,
                bed_pivot,
                rot_offset,
            } => {
                let rot_offset_rad = rot_offset * consts::PI / 180.0;
                // Rotation in x-y-z (roll-pitch-yaw) order
                let offset_rot = Rotation3::from_euler_angles(
                    rot_offset_rad.x,
                    rot_offset_rad.y,
                    rot_offset_rad.z,
                );

                let a = coords.a.expect("A coord is necessary");
                let a_rad = a * consts::PI / 180.0;
                let a_rot = Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);

                let b = coords.b.expect("B coord is necessary");
                let b_rad = b * consts::PI / 180.0;
                let b_rot = Rotation3::from_axis_angle(&Vector3::y_axis(), b_rad);

                let machine_xyz =
                    offset_rot * b_rot * a_rot * vector![coords.x, coords.y, coords.z - bed_pivot]
                        + pivot.coords;

                GenericCoords {
                    x: machine_xyz.x,
                    y: machine_xyz.y,
                    z: machine_xyz.z,
                    i: Some(a + b),
                    j: Some(a - b),
                    ..Default::default()
                }
            }
        }
    }

    pub fn center(&self) -> Point2<f64> {
        match self {
            &Printer::ThreeDoF { center } => center,
            &Printer::DifferentialTiltingBed { .. } => Point2::origin(),
        }
    }
}
