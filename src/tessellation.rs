use nalgebra as na;
use na::Vector3;
use stl_io::{IndexedMesh, IndexedTriangle};

use crate::utils::{to_na, from_na};

pub fn tesselate(input: IndexedMesh, max_edge_len: f32) -> IndexedMesh {
    let mut vertices: Vec<Vector3<f32>> = input.vertices.into_iter().map(to_na).collect();
    let mut divided_triangles: Vec<IndexedTriangle> = vec![];

    // Recursively divide triangles using depth-first search
    let mut triangles_to_divide = input.faces.clone();
    while let Some(triangle) = triangles_to_divide.pop() {
        let edges = [
            (triangle.vertices[0], triangle.vertices[1]),
            (triangle.vertices[1], triangle.vertices[2]),
            (triangle.vertices[2], triangle.vertices[0]),
        ];
        let edges_len = edges.map(|(i, j)| {
            (vertices[j] - vertices[i]).norm()
        });

        // Divide if the triangle has at least one edge longer than threshold
        if edges_len.iter().any(|&l| l > max_edge_len) {
            // Create new vertices on the midpoints of each edge
            let midpoints = edges.map(|(i, j)| {
                vertices.push((vertices[i] + vertices[j]) / 2.0);
                vertices.len() - 1
            });
            
            // Create new triangles
            triangles_to_divide.push(IndexedTriangle {
                normal: triangle.normal,
                vertices: [triangle.vertices[0], midpoints[0], midpoints[2]]
            });
            triangles_to_divide.push(IndexedTriangle {
                normal: triangle.normal,
                vertices: [triangle.vertices[1], midpoints[1], midpoints[0]]
            });
            triangles_to_divide.push(IndexedTriangle {
                normal: triangle.normal,
                vertices: [triangle.vertices[2], midpoints[2], midpoints[1]]
            });
            triangles_to_divide.push(IndexedTriangle {
                normal: triangle.normal,
                vertices: [midpoints[0], midpoints[1], midpoints[2]]
            });
        } else {
            divided_triangles.push(triangle);
        }
    }

    IndexedMesh {
        vertices: vertices.into_iter().map(from_na).collect(),
        faces: divided_triangles
    }
}