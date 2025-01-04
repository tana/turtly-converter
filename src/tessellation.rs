// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::collections::HashMap;

use na::Vector3;
use nalgebra as na;

use crate::utils::Mesh;

pub fn tesselate(input: Mesh, max_edge_len: f64) -> Mesh {
    let mut vertices: Vec<Vector3<f64>> = input.vertices;

    let mut divided_edges: HashMap<(usize, usize), usize> = HashMap::new();

    // Recursively divide triangles using depth-first search
    let mut triangles = input.triangles;
    loop {
        // Terminate when all triangles are small enough
        if triangles
            .iter()
            .all(|triangle| triangle_needs_division(triangle, &vertices, max_edge_len))
        {
            break;
        }

        let mut new_triangles = Vec::new();
        for triangle in triangles {
            let edges = [
                (triangle[0], triangle[1]),
                (triangle[1], triangle[2]),
                (triangle[2], triangle[0]),
            ];

            // Create new vertices on the midpoints of each edge
            let midpoints = edges.map(|(i, j)| {
                if let Some(midpoint) = divided_edges.get(&normalize_edge(&(i, j))) {
                    // Reuse midpoint if the edge is already divided (to make watertight)
                    *midpoint
                } else {
                    vertices.push((vertices[i] + vertices[j]) / 2.0);
                    let midpoint = vertices.len() - 1;
                    divided_edges.insert(normalize_edge(&(i, j)), midpoint);

                    midpoint
                }
            });

            // Create new triangles
            new_triangles.push([triangle[0], midpoints[0], midpoints[2]]);
            new_triangles.push([triangle[1], midpoints[1], midpoints[0]]);
            new_triangles.push([triangle[2], midpoints[2], midpoints[1]]);
            new_triangles.push([midpoints[0], midpoints[1], midpoints[2]]);
        }

        triangles = new_triangles;
    }

    Mesh {
        vertices,
        triangles,
    }
}

fn triangle_needs_division(
    triangle: &[usize; 3],
    vertices: &Vec<Vector3<f64>>,
    max_edge_len: f64,
) -> bool {
    let edges = [
        (triangle[0], triangle[1]),
        (triangle[1], triangle[2]),
        (triangle[2], triangle[0]),
    ];

    edges
        .iter()
        .all(|&(i, j)| (vertices[j] - vertices[i]).norm() < max_edge_len)
}

fn normalize_edge(edge: &(usize, usize)) -> (usize, usize) {
    if edge.0 > edge.1 {
        (edge.1, edge.0)
    } else {
        (edge.0, edge.1)
    }
}
