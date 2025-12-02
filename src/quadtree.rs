use bevy::prelude::*;

use crate::resources::SOFTENING;

#[derive(Clone, Copy, Debug)]
pub struct Rect {
    pub center: Vec2,
    pub size: Vec2,
}

impl Rect {
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

#[derive(Clone, Copy)]
pub enum NodeKind {
    Empty,
    Leaf { entity: Entity, position: Vec2 },
    Internal { children: [Option<usize>; 4] },
}

pub struct Node {
    pub bounds: Rect,
    pub center_of_mass: Vec2,
    pub mass: f32,
    pub kind: NodeKind,
}

impl Node {
    pub fn empty(bounds: Rect) -> Self {
        Self {
            bounds,
            center_of_mass: Vec2::ZERO,
            mass: 0.0,
            kind: NodeKind::Empty,
        }
    }
}

#[derive(Resource, Default)]
pub struct QuadTreeResource {
    pub nodes: Vec<Node>,
    pub root: Option<usize>,
}

impl QuadTreeResource {
    pub fn reset(&mut self, bounds: Rect) {
        self.nodes.clear();
        self.root = Some(self.nodes.len());
        self.nodes.push(Node::empty(bounds));
    }

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
                    let total_mass = existing_mass + mass;
                    self.nodes[index].mass = total_mass;
                    self.nodes[index].center_of_mass =
                        (existing_position * existing_mass + position * mass) / total_mass;
                    return;
                }

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
                let child_idx = bounds.get_quadrant_index(position);
                if let Some(idx) = children[child_idx] {
                    self.insert_recursive(idx, entity, position, mass);
                }

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
            let child_bounds = self.nodes[index].bounds.sub_quadrant(quadrant);
            let child_index = self.nodes.len();
            self.nodes.push(Node::empty(child_bounds));
            *child = Some(child_index);
        }
    }

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
                    let dist_sq = dist * dist + SOFTENING * SOFTENING;
                    let force_mag = (config.g * node.mass) / dist_sq;
                    delta / dist * force_mag
                } else {
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
