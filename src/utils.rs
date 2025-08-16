// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use anyhow::Result;
use na::vector;
use nalgebra as na;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Aabb {
    pub origin: na::Vector3<f64>,
    pub size: na::Vector3<f64>,
}

pub struct Mesh {
    pub vertices: Vec<na::Vector3<f64>>,
    pub triangles: Vec<[usize; 3]>,
}

impl Mesh {
    pub fn calc_aabb(&self) -> Aabb {
        let mut min = na::Vector3::from_element(std::f64::MAX);
        let mut max = na::Vector3::from_element(std::f64::MIN);

        for vert in self.vertices.iter() {
            min = min.map_with_location(|i, _, e: f64| e.min(vert[i]));
            max = max.map_with_location(|i, _, e: f64| e.max(vert[i]));
        }

        Aabb {
            origin: min,
            size: max - min,
        }
    }

    pub fn calc_vert_normals(&self) -> Vec<na::Vector3<f64>> {
        // Vertex normals
        let mut vert_normals = vec![na::Vector3::zeros(); self.vertices.len()];
        // Number of triangles containing the vertex
        let mut vert_num_tri = vec![0; self.vertices.len()];
        // Calculate vertex normals from triangles
        for tri in self.triangles.iter() {
            let [v1, v2, v3] = tri.map(|idx| self.vertices[idx]);
            // Normal vector of the triangle
            let tri_normal = (v2 - v1).cross(&(v3 - v2)).normalize();

            // Vertex normal is the average of the normals of all triangles containing the vertex
            for &v_idx in tri {
                vert_normals[v_idx] = (vert_num_tri[v_idx] as f64 * vert_normals[v_idx]
                    + tri_normal)
                    / (vert_num_tri[v_idx] + 1) as f64;
                vert_num_tri[v_idx] += 1;
            }
        }

        vert_normals
    }
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
