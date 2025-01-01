use anyhow::Result;
use na::vector;
use nalgebra as na;

pub fn to_na(v: stl_io::Vector<f32>) -> na::Vector3<f32> {
    vector![v[0], v[1], v[2]]
}

pub fn from_na(v: na::Vector3<f32>) -> stl_io::Vector<f32> {
    stl_io::Vector::new([v.x, v.y, v.z])
}

pub fn parse_vector(s: &str) -> Result<na::Vector3<f32>> {
    let mut numbers = s.split(",").map(|e| e.parse::<f32>());

    let x = numbers.next().transpose()?.unwrap_or_default();
    let y = numbers.next().transpose()?.unwrap_or_default();
    let z = numbers.next().transpose()?.unwrap_or_default();

    Ok(vector![x, y, z])
}
