// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::f64::consts;

use na::{Point2, Point3};
use nalgebra as na;
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
        c_offset: f64,
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

    #[rustfmt::skip]
    pub fn calc_ik(&self, coords: GenericCoords) -> GenericCoords {
        match self {
            &Printer::ThreeDoF { .. } => coords,
            &Printer::DancingBed {
                tilt,
                pivot,
                c_offset,
            } => {
                let c = coords.c.expect("C coord is necessary") - c_offset;
                let c_rad = c * consts::PI / 180.0;
                let tilt_rad = tilt * consts::PI / 180.0;
                let sin_c = c_rad.sin();
                let cos_c = c_rad.cos();
                let sin_tilt = tilt_rad.sin();
                let cos_tilt = tilt_rad.cos();
                GenericCoords {
                    /*
                    x: cos_c * coords.x - sin_c * coords.y
                        + pivot.x,
                    y: cos_tilt * sin_c * coords.x + cos_tilt * cos_c * coords.y - sin_tilt * coords.z
                        + pivot.y,
                    z: sin_tilt * sin_c * coords.x + sin_tilt * cos_c * coords.y + cos_tilt * coords.z
                        + pivot.z,
                    */
                    x: cos_c * coords.x - sin_c * cos_tilt * coords.y + sin_c * sin_tilt * coords.z
                        + pivot.x,
                    y: sin_c * coords.x + cos_c * cos_tilt * coords.y - cos_c * sin_tilt * coords.z
                        + pivot.y,
                    z: sin_tilt * coords.y + cos_tilt * coords.z
                        + pivot.z,
                    c: Some(c),
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
