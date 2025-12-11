//! Barnes-Hut quadtree used for approximate n-body force calculation.

use bevy::prelude::*;

use crate::resources::SOFTENING;

/// Axis-aligned square region.
#[derive(Clone, Copy, Debug)]
pub struct Rect {
    pub center: Vec2,
    pub size: Vec2,
}

impl Rect {
    /// Returns the quadrant index (0..=3) that contains `point`.
    pub fn get_quadrant_index(&self, point: Vec2) -> usize {
        let right = point.x > self.center.x;
        let top = point.y > self.center.y;
        match (right, top) {
            (false, true) => 0,
            (true, true) => 1,
            (false, false) => 2,
            (true, false) => 3,
        }
    }

    /// Returns the sub-rectangle corresponding to `index`.
    pub fn sub_quadrant(&self, index: usize) -> Rect {
        let quarter_size = self.size / 2.0;
        let offset = quarter_size / 2.0;
        let center = match index {
            0 => self.center + vec2(-offset.x, offset.y),
            1 => self.center + vec2(offset.x, offset.y),
            2 => self.center + vec2(-offset.x, -offset.y),
            3 => self.center + vec2(offset.x, -offset.y),
            _ => self.center,
        };
        Rect {
            center,
            size: quarter_size,
        }
    }
}

/// Logical shape of a quadtree node.
#[derive(Clone, Copy)]
pub enum NodeKind {
    Empty,
    Leaf { entity: Entity, position: Vec2 },
    Internal { children: [Option<usize>; 4] },
}

/// Barnes-Hut quadtree node.
pub struct Node {
    pub bounds: Rect,
    pub center_of_mass: Vec2,
    pub mass: f32,
    pub kind: NodeKind,
}

impl Node {
    /// Creates an empty node covering `bounds`.
    pub fn empty(bounds: Rect) -> Self {
        Self {
            bounds,
            center_of_mass: Vec2::ZERO,
            mass: 0.0,
            kind: NodeKind::Empty,
        }
    }
}

/// Resource storing the quadtree used for force approximation.
#[derive(Resource, Default)]
pub struct QuadTreeResource {
    pub nodes: Vec<Node>,
    pub root: Option<usize>,
}

impl QuadTreeResource {
    /// Clears the tree and inserts a new root covering `bounds`.
    pub fn reset(&mut self, bounds: Rect) {
        self.nodes.clear();
        self.root = Some(self.nodes.len());
        self.nodes.push(Node::empty(bounds));
    }

    /// Inserts a body entity with position and mass into the quadtree.
    pub fn insert(&mut self, entity: Entity, position: Vec2, mass: f32) {
        let root_index = match self.root {
            Some(index) => index,
            None => return,
        };
        self.insert_recursive(root_index, entity, position, mass);
    }

    fn insert_recursive(&mut self, index: usize, entity: Entity, position: Vec2, mass: f32) {
        let bounds = self.nodes[index].bounds;
        match self.nodes[index].kind {
            NodeKind::Empty => {
                // Nothing here yet: place a leaf.
                self.nodes[index].kind = NodeKind::Leaf { entity, position };
                self.nodes[index].mass = mass;
                self.nodes[index].center_of_mass = position;
            }
            NodeKind::Leaf {
                entity: existing_entity,
                position: existing_position,
            } => {
                let existing_mass = self.nodes[index].mass;
                if (existing_position - position).length_squared() < 0.0001 {
                    // Same location: merge mass and update center of mass.
                    let total_mass = existing_mass + mass;
                    self.nodes[index].mass = total_mass;
                    self.nodes[index].center_of_mass =
                        (existing_position * existing_mass + position * mass) / total_mass;
                    return;
                }

                // Subdivide and reinsert both the existing leaf and the new body.
                let mut children = [None, None, None, None];
                self.subdivide(index, &mut children);

                let existing_index = bounds.get_quadrant_index(existing_position);
                let new_index = bounds.get_quadrant_index(position);

                if let Some(child_idx) = children[existing_index] {
                    self.insert_recursive(
                        child_idx,
                        existing_entity,
                        existing_position,
                        existing_mass,
                    );
                }

                if let Some(child_idx) = children[new_index] {
                    self.insert_recursive(child_idx, entity, position, mass);
                }

                let total_mass = existing_mass + mass;
                let com = (existing_position * existing_mass + position * mass) / total_mass;

                self.nodes[index].kind = NodeKind::Internal { children };
                self.nodes[index].mass = total_mass;
                self.nodes[index].center_of_mass = com;
            }
            NodeKind::Internal { children } => {
                // Descend into the child that contains the new position.
                let child_idx = bounds.get_quadrant_index(position);
                if let Some(idx) = children[child_idx] {
                    self.insert_recursive(idx, entity, position, mass);
                }

                // Update mass and center of mass on the way back up.
                let total_mass = self.nodes[index].mass + mass;
                let com = (self.nodes[index].center_of_mass * self.nodes[index].mass
                    + position * mass)
                    / total_mass;
                self.nodes[index].mass = total_mass;
                self.nodes[index].center_of_mass = com;
                self.nodes[index].kind = NodeKind::Internal { children };
            }
        }
    }

    fn subdivide(&mut self, index: usize, children: &mut [Option<usize>; 4]) {
        for (quadrant, child) in children.iter_mut().enumerate() {
            // Create an empty child node for each quadrant.
            let child_bounds = self.nodes[index].bounds.sub_quadrant(quadrant);
            let child_index = self.nodes.len();
            self.nodes.push(Node::empty(child_bounds));
            *child = Some(child_index);
        }
    }

    /// Returns the net gravitational force on `target` using Barnes-Hut approximation.
    pub fn calculate_force(
        &self,
        target: Entity,
        position: Vec2,
        config: &crate::resources::SimConfig,
    ) -> Vec2 {
        let root = match self.root {
            Some(idx) => idx,
            None => return Vec2::ZERO,
        };

        self.calculate_force_recursive(root, target, position, config)
    }

    fn calculate_force_recursive(
        &self,
        index: usize,
        target: Entity,
        position: Vec2,
        config: &crate::resources::SimConfig,
    ) -> Vec2 {
        let node = &self.nodes[index];
        match node.kind {
            NodeKind::Empty => Vec2::ZERO,
            NodeKind::Leaf {
                entity,
                position: pos,
            } => {
                if entity == target {
                    return Vec2::ZERO;
                }
                // Direct body-body interaction.
                let delta = pos - position;
                let dist_sq = delta.length_squared() + SOFTENING * SOFTENING;
                let dist = dist_sq.sqrt();
                let force_mag = (config.g * node.mass) / dist_sq;
                delta / dist * force_mag
            }
            NodeKind::Internal { children } => {
                let delta = node.center_of_mass - position;
                let dist = delta.length().max(0.0001);
                let width = node.bounds.size.x;

                if width / dist < config.theta {
                    // Accept approximation: treat node as a single mass at its center of mass.
                    let dist_sq = dist * dist + SOFTENING * SOFTENING;
                    let force_mag = (config.g * node.mass) / dist_sq;
                    delta / dist * force_mag
                } else {
                    // Otherwise recurse into children and accumulate force.
                    let mut total_force = Vec2::ZERO;
                    for child in children.iter().flatten() {
                        total_force +=
                            self.calculate_force_recursive(*child, target, position, config);
                    }
                    total_force
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resources::SimConfig;

    fn assert_vec2_close(a: Vec2, b: Vec2, tolerance: f32) {
        let diff = (a - b).length();
        assert!(
            diff <= tolerance,
            "expected {:?} to be within {} of {:?}, diff {}",
            a,
            tolerance,
            b,
            diff
        );
    }

    #[test]
    fn quadrants_and_subdivision_are_consistent() {
        let rect = Rect {
            center: Vec2::ZERO,
            size: Vec2::splat(4.0),
        };

        assert_eq!(rect.get_quadrant_index(vec2(-1.0, 1.0)), 0);
        assert_eq!(rect.get_quadrant_index(vec2(1.0, 1.0)), 1);
        assert_eq!(rect.get_quadrant_index(vec2(-1.0, -1.0)), 2);
        assert_eq!(rect.get_quadrant_index(vec2(1.0, -1.0)), 3);

        let centers = [
            vec2(-1.0, 1.0),
            vec2(1.0, 1.0),
            vec2(-1.0, -1.0),
            vec2(1.0, -1.0),
        ];
        for (i, expected_center) in centers.iter().enumerate() {
            let quad = rect.sub_quadrant(i);
            assert_vec2_close(quad.center, *expected_center, 0.0001);
            assert_vec2_close(quad.size, Vec2::splat(2.0), 0.0001);
        }
    }

    #[test]
    fn insert_combines_overlapping_positions() {
        let mut quadtree = QuadTreeResource::default();
        let bounds = Rect {
            center: Vec2::ZERO,
            size: Vec2::splat(10.0),
        };
        quadtree.reset(bounds);

        let entity_a = Entity::from_bits(1);
        let entity_b = Entity::from_bits(2);
        let position = vec2(1.0, 1.0);

        quadtree.insert(entity_a, position, 2.0);
        quadtree.insert(entity_b, position, 3.0);

        let root = quadtree.root.unwrap();
        assert_eq!(quadtree.nodes.len(), 1);
        let node = &quadtree.nodes[root];
        assert!(matches!(node.kind, NodeKind::Leaf { .. }));
        assert!((node.mass - 5.0).abs() < 0.0001);
        assert_vec2_close(node.center_of_mass, position, 0.0001);
    }

    #[test]
    fn calculate_force_ignores_target_entity() {
        let mut quadtree = QuadTreeResource::default();
        let bounds = Rect {
            center: Vec2::ZERO,
            size: Vec2::splat(10.0),
        };
        quadtree.reset(bounds);

        let entity = Entity::from_bits(1);
        quadtree.insert(entity, Vec2::ZERO, 5.0);

        let config = SimConfig::default();
        let force = quadtree.calculate_force(entity, Vec2::ZERO, &config);
        assert_vec2_close(force, Vec2::ZERO, 0.0001);
    }

    #[test]
    fn calculate_force_uses_approximation_for_distant_nodes() {
        let mut quadtree = QuadTreeResource::default();
        let bounds = Rect {
            center: Vec2::ZERO,
            size: Vec2::splat(10.0),
        };

        let com = vec2(50.0, 0.0);
        quadtree.nodes.push(Node {
            bounds,
            center_of_mass: com,
            mass: 8.0,
            kind: NodeKind::Internal {
                children: [None, None, None, None],
            },
        });
        quadtree.root = Some(0);

        let config = SimConfig {
            theta: 0.5,
            ..Default::default()
        };
        let target = Entity::from_bits(99);
        let position = Vec2::ZERO;

        let force = quadtree.calculate_force(target, position, &config);
        let delta = com - position;
        let dist = delta.length().max(0.0001);
        let dist_sq = dist * dist + SOFTENING * SOFTENING;
        let expected_mag = (config.g * 8.0) / dist_sq;
        let expected = delta / dist * expected_mag;
        assert_vec2_close(force, expected, 0.0001);
    }

    #[test]
    fn calculate_force_recurses_into_children() {
        let mut quadtree = QuadTreeResource::default();
        let bounds = Rect {
            center: Vec2::ZERO,
            size: Vec2::splat(20.0),
        };

        let child_a = Node {
            bounds: bounds.sub_quadrant(1),
            center_of_mass: vec2(5.0, 0.0),
            mass: 2.0,
            kind: NodeKind::Leaf {
                entity: Entity::from_bits(1),
                position: vec2(5.0, 0.0),
            },
        };
        let child_b = Node {
            bounds: bounds.sub_quadrant(0),
            center_of_mass: vec2(-5.0, 0.0),
            mass: 3.0,
            kind: NodeKind::Leaf {
                entity: Entity::from_bits(2),
                position: vec2(-5.0, 0.0),
            },
        };

        let child_a_mass = child_a.mass;
        let child_a_pos = child_a.center_of_mass;
        let child_b_mass = child_b.mass;
        let child_b_pos = child_b.center_of_mass;

        quadtree.nodes.push(Node {
            bounds,
            center_of_mass: Vec2::ZERO,
            mass: child_a.mass + child_b.mass,
            kind: NodeKind::Internal {
                children: [Some(2), Some(1), None, None],
            },
        });
        quadtree.nodes.push(child_a);
        quadtree.nodes.push(child_b);
        quadtree.root = Some(0);

        let config = SimConfig {
            theta: 0.0,
            ..Default::default()
        };
        let target = Entity::from_bits(99);
        let position = Vec2::ZERO;
        let force = quadtree.calculate_force(target, position, &config);

        let compute_leaf_force = |mass: f32, pos: Vec2| {
            let delta = pos - position;
            let dist_sq = delta.length_squared() + SOFTENING * SOFTENING;
            let dist = dist_sq.sqrt();
            let force_mag = (config.g * mass) / dist_sq;
            delta / dist * force_mag
        };
        let expected = compute_leaf_force(child_a_mass, child_a_pos)
            + compute_leaf_force(child_b_mass, child_b_pos);

        assert_vec2_close(force, expected, 0.0001);
    }
}
