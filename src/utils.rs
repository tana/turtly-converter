use nalgebra as na;
use na::vector;

pub fn to_na(v: stl_io::Vector<f32>) -> na::Vector3<f32> {
    vector![v[0], v[1], v[2]]
}

pub fn from_na(v: na::Vector3<f32>) -> stl_io::Vector<f32> {
    stl_io::Vector::new([v.x, v.y, v.z])
}