use anyhow::Result;
use na::vector;
use nalgebra as na;

pub struct Mesh {
    pub vertices: Vec<na::Vector3<f64>>,
    pub triangles: Vec<[usize; 3]>,
}

impl From<stl_io::IndexedMesh> for Mesh {
    fn from(value: stl_io::IndexedMesh) -> Self {
        Self {
            vertices: value.vertices.into_iter().map(to_na).collect(),
            triangles: value.faces.into_iter().map(|tri| tri.vertices).collect(),
        }
    }
}

impl From<Mesh> for stl_io::IndexedMesh {
    fn from(value: Mesh) -> stl_io::IndexedMesh {
        let faces = value
            .triangles
            .iter()
            .map(|tri_idx| {
                let tri = tri_idx.map(|i| value.vertices[i]);
                stl_io::IndexedTriangle {
                    normal: from_na((tri[1] - tri[0]).cross(&(tri[2] - tri[1]))),
                    vertices: tri_idx.clone(),
                }
            })
            .collect();

        stl_io::IndexedMesh {
            vertices: value.vertices.into_iter().map(from_na).collect(),
            faces,
        }
    }
}

pub fn to_na(v: stl_io::Vector<f32>) -> na::Vector3<f64> {
    vector![v[0] as f64, v[1] as f64, v[2] as f64]
}

pub fn from_na(v: na::Vector3<f64>) -> stl_io::Vector<f32> {
    stl_io::Vector::new([v.x as f32, v.y as f32, v.z as f32])
}

pub fn parse_vector(s: &str) -> Result<na::Vector3<f64>> {
    let mut numbers = s.split(",").map(|e| e.parse::<f64>());

    let x = numbers.next().transpose()?.unwrap_or_default();
    let y = numbers.next().transpose()?.unwrap_or_default();
    let z = numbers.next().transpose()?.unwrap_or_default();

    Ok(vector![x, y, z])
}
