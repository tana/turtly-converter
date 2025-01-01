use na::Vector3;
use nalgebra as na;

use crate::utils::Mesh;

pub fn tesselate(input: Mesh, max_edge_len: f64) -> Mesh {
    let mut vertices: Vec<Vector3<f64>> = input.vertices;
    let mut divided_triangles: Vec<[usize; 3]> = vec![];

    // Recursively divide triangles using depth-first search
    let mut triangles_to_divide = input.triangles.clone();
    while let Some(triangle) = triangles_to_divide.pop() {
        let edges = [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ];
        let edges_len = edges.map(|(i, j)| (vertices[j] - vertices[i]).norm());

        // Divide if the triangle has at least one edge longer than threshold
        if edges_len.iter().any(|&l| l > max_edge_len) {
            // Create new vertices on the midpoints of each edge
            let midpoints = edges.map(|(i, j)| {
                vertices.push((vertices[i] + vertices[j]) / 2.0);
                vertices.len() - 1
            });

            // Create new triangles
            triangles_to_divide.push([triangle[0], midpoints[0], midpoints[2]]);
            triangles_to_divide.push([triangle[1], midpoints[1], midpoints[0]]);
            triangles_to_divide.push([triangle[2], midpoints[2], midpoints[1]]);
            triangles_to_divide.push([midpoints[0], midpoints[1], midpoints[2]]);
        } else {
            divided_triangles.push(triangle);
        }
    }

    Mesh {
        vertices,
        triangles: divided_triangles,
    }
}
