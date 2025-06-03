// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use na::Point2;
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, PartialEq, Debug, Serialize, Deserialize)]
pub enum Printer {
    ThreeDoF { center: Point2<f64> },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum PrinterType {
    /// Conventional 3D printer with 3-DoF toolhead movement
    ThreeDoF,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct GenericCoords {
    x: f64,
    y: f64,
    z: f64,
    a: Option<f64>,
    b: Option<f64>,
    c: Option<f64>,
}

impl Printer {
    pub fn printer_type(&self) -> PrinterType {
        match self {
            &Printer::ThreeDoF { .. } => PrinterType::ThreeDoF,
        }
    }

    pub fn calc_ik(&self, coords: GenericCoords) -> GenericCoords {
        match self {
            &Printer::ThreeDoF { .. } => coords,
        }
    }

    pub fn center(&self) -> Point2<f64> {
        match self {
            &Printer::ThreeDoF { center } => center,
        }
    }
}
