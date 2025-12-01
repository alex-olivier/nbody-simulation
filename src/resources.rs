use bevy::prelude::*;

// --- Simulation Defaults ---
pub const DEFAULT_G: f32 = 100.0;
pub const DEFAULT_THETA: f32 = 0.5;
pub const DEFAULT_DT: f32 = 1.0 / 60.0;
pub const SOFTENING: f32 = 5.0;
pub const NUM_BODIES: usize = 2000;
pub const TRAIL_LENGTH: usize = 20;
pub const CULL_DISTANCE: f32 = 1500.0;
pub const MIN_GIZMO_NODE_SIZE: f32 = 2.0;

#[derive(Resource)]
pub struct SimConfig {
    pub g: f32,
    pub theta: f32,
    pub dt: f32,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            g: DEFAULT_G,
            theta: DEFAULT_THETA,
            dt: DEFAULT_DT,
        }
    }
}

#[derive(Resource, Clone, Copy)]
pub struct SimulationBounds {
    pub root: crate::quadtree::Rect,
}

impl Default for SimulationBounds {
    fn default() -> Self {
        Self {
            root: crate::quadtree::Rect {
                center: Vec2::ZERO,
                size: Vec2::splat(2000.0),
            },
        }
    }
}

#[derive(Resource)]
pub struct SimSettings {
    pub time_scale: f32,
    pub enable_trails: bool,
    pub enable_culling: bool,
    pub follow_com: bool,
    pub show_gizmos: bool,
}

impl Default for SimSettings {
    fn default() -> Self {
        Self {
            time_scale: 1.0,
            enable_trails: false,
            enable_culling: false,
            follow_com: false,
            show_gizmos: false,
        }
    }
}

#[derive(Resource, Default)]
pub struct ResetSimulation {
    pub pending: bool,
}
