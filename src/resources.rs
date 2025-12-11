use bevy::prelude::*;

/// Tunable runtime simulation parameters (G, theta, timestep).
#[derive(Resource)]
pub struct SimConfig {
    pub g: f32,
    pub theta: f32,
    pub dt: f32,
}

// --- Simulation Defaults ---
/// Default gravitational constant.
pub const DEFAULT_G: f32 = 100.0;
/// Default Barnes-Hut theta threshold.
pub const DEFAULT_THETA: f32 = 0.5;
/// Default fixed timestep for physics.
pub const DEFAULT_DT: f32 = 1.0 / 60.0;
/// Distance softening to prevent singularities.
pub const SOFTENING: f32 = 5.0;
/// Number of dynamic bodies spawned at reset.
pub const NUM_BODIES: usize = 2000;
/// Maximum stored points per trail.
pub const TRAIL_LENGTH: usize = 20;
/// Distance from origin after which culling will despawn bodies.
pub const CULL_DISTANCE: f32 = 1500.0;
/// Smallest quadtree node size that will be drawn as a gizmo.
pub const MIN_GIZMO_NODE_SIZE: f32 = 2.0;

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            g: DEFAULT_G,
            theta: DEFAULT_THETA,
            dt: DEFAULT_DT,
        }
    }
}

/// Bounds used to initialize the quadtree root.
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

/// User-facing toggles that drive rendering and simulation behavior.
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

/// Marker resource to request a simulation reset from the UI.
#[derive(Resource, Default)]
pub struct ResetSimulation {
    pub pending: bool,
}
