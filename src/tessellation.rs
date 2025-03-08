// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::collections::HashMap;

use na::Vector3;
use nalgebra as na;

use crate::utils::Mesh;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct HalfEdgeKey {
    origin: usize,
    destination: usize,
}

impl HalfEdgeKey {
    fn reverse(&self) -> HalfEdgeKey {
        HalfEdgeKey {
            origin: self.destination,
            destination: self.origin,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct HalfEdge {
    triangle: usize,
    prev: HalfEdgeKey,
    next: HalfEdgeKey,
}

struct HalfEdgeMesh {
    vertices: Vec<Vector3<f64>>,
    triangles: Vec<[usize; 3]>,
    half_edges: HashMap<HalfEdgeKey, HalfEdge>,
}

impl HalfEdgeMesh {
    fn new(input: Mesh) -> Self {
        let mut half_edges = HashMap::new();
        // Create half-edge data structure
        for (tri_idx, tri) in input.triangles.iter().enumerate() {
            half_edges.insert(
                HalfEdgeKey {
                    origin: tri[0],
                    destination: tri[1],
                },
                HalfEdge {
                    triangle: tri_idx,
                    prev: HalfEdgeKey {
                        origin: tri[2],
                        destination: tri[0],
                    },
                    next: HalfEdgeKey {
                        origin: tri[1],
                        destination: tri[2],
                    },
                },
            );
            half_edges.insert(
                HalfEdgeKey {
                    origin: tri[1],
                    destination: tri[2],
                },
                HalfEdge {
                    triangle: tri_idx,
                    prev: HalfEdgeKey {
                        origin: tri[0],
                        destination: tri[1],
                    },
                    next: HalfEdgeKey {
                        origin: tri[2],
                        destination: tri[0],
                    },
                },
            );
            half_edges.insert(
                HalfEdgeKey {
                    origin: tri[2],
                    destination: tri[0],
                },
                HalfEdge {
                    triangle: tri_idx,
                    prev: HalfEdgeKey {
                        origin: tri[1],
                        destination: tri[2],
                    },
                    next: HalfEdgeKey {
                        origin: tri[0],
                        destination: tri[1],
                    },
                },
            );
        }

        Self {
            vertices: input.vertices,
            triangles: input.triangles,
            half_edges,
        }
    }

    fn split_edge(&mut self, origin: usize, destination: usize) {
        let he_key = HalfEdgeKey {
            origin,
            destination,
        };

        // Create a new vertex at the midpoint of the edge
        self.vertices
            .push((self.vertices[he_key.origin] + self.vertices[he_key.destination]) / 2.0);
        let midpoint = self.vertices.len() - 1;

        // Remove the divided edge
        let he = self.half_edges.remove(&he_key).unwrap();
        let he_rev_key = he_key.reverse();
        let he_rev = self.half_edges.remove(&he_rev_key).unwrap();

        // Naming convention: Splitting a vertical edge between two triangles.
        // `he` is upward and `he_rev` is downward.
        let he_top_key = HalfEdgeKey {
            origin: midpoint,
            destination: he_key.destination,
        };
        let he_bottom_key = HalfEdgeKey {
            origin: he_key.origin,
            destination: midpoint,
        };
        let he_left_key = HalfEdgeKey {
            origin: he.next.destination,
            destination: midpoint,
        };
        let he_right_key = HalfEdgeKey {
            origin: midpoint,
            destination: he_rev.next.destination,
        };

        // Left top triangle (reusing left triangle)
        self.triangles[he.triangle] = [midpoint, he_key.destination, he.next.destination];
        self.insert_triangle_half_edges(he.triangle, he_left_key, he_top_key, he.next);
        // Left bottom triangle (adding new triangle)
        self.triangles
            .push([midpoint, he.next.destination, he_key.origin]);
        self.insert_triangle_half_edges(
            self.triangles.len() - 1,
            he_left_key.reverse(),
            he.prev,
            he_bottom_key,
        );
        // Right bottom triangle (reusing right triangle)
        self.triangles[he_rev.triangle] =
            [midpoint, he_rev_key.destination, he_rev.next.destination];
        self.insert_triangle_half_edges(
            he_rev.triangle,
            he_right_key.reverse(),
            he_bottom_key.reverse(),
            he_rev.next,
        );
        // Right top triangle (adding new triangle)
        self.triangles
            .push([midpoint, he_rev.next.destination, he_rev_key.origin]);
        self.insert_triangle_half_edges(
            self.triangles.len() - 1,
            he_right_key,
            he_rev.prev,
            he_top_key.reverse(),
        );
    }

    fn insert_triangle_half_edges(
        &mut self,
        triangle: usize,
        a: HalfEdgeKey,
        b: HalfEdgeKey,
        c: HalfEdgeKey,
    ) {
        self.half_edges.insert(
            a,
            HalfEdge {
                triangle,
                prev: c,
                next: b,
            },
        );
        self.half_edges.insert(
            b,
            HalfEdge {
                triangle,
                prev: a,
                next: c,
            },
        );
        self.half_edges.insert(
            c,
            HalfEdge {
                triangle,
                prev: b,
                next: a,
            },
        );
    }

    fn edges(&self) -> impl Iterator<Item = (usize, usize)> + use<'_> {
        self.half_edges
            .keys()
            .filter(|k| k.origin < k.destination)
            .map(|k| (k.origin, k.destination))
    }
}

pub fn tesselate(input: Mesh, max_edge_len: f64) -> Mesh {
    let mut mesh = HalfEdgeMesh::new(input);

    // Recursively split edges
    loop {
        let mut should_stop = true;

        let edges: Vec<_> = mesh.edges().collect();

        for (origin, destination) in edges {
            if (mesh.vertices[destination] - mesh.vertices[origin]).norm_squared()
                > (max_edge_len * max_edge_len)
            {
                mesh.split_edge(origin, destination);

                should_stop = false;
            }
        }

        if should_stop {
            break;
        }
    }

    Mesh {
        vertices: mesh.vertices,
        triangles: mesh.triangles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use na::vector;
    use nalgebra as na;

    #[test]
    fn tesselate_test() {
        let max_edge_len = 1.0;

        // tetrahedron
        let mesh = Mesh {
            vertices: vec![
                vector![-5.0, 5.0, 0.0],
                vector![5.0, 5.0, 0.0],
                vector![5.0, -5.0, 0.0],
                vector![-5.0, -5.0, 0.0],
                vector![0.0, 0.0, 10.0],
            ],
            triangles: vec![
                [0, 1, 2],
                [0, 2, 3],
                [1, 0, 4],
                [0, 3, 4],
                [3, 2, 4],
                [2, 1, 4],
            ],
        };

        let mesh = tesselate(mesh, max_edge_len);

        // Check whether all edges are shorter than `max_edge_len`
        for tri in mesh.triangles {
            assert!((mesh.vertices[tri[0]] - mesh.vertices[tri[1]]).norm() <= max_edge_len);
            assert!((mesh.vertices[tri[1]] - mesh.vertices[tri[2]]).norm() <= max_edge_len);
            assert!((mesh.vertices[tri[2]] - mesh.vertices[tri[0]]).norm() <= max_edge_len);
        }
    }
}
