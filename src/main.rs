mod components;
mod quadtree;
mod resources;
mod systems;

use bevy::prelude::*;
use bevy::window::WindowResolution;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

use crate::quadtree::QuadTreeResource;
use crate::resources::ResetSimulation;
use crate::resources::{DEFAULT_DT, SimulationBounds};
use crate::resources::{SimConfig, SimSettings};
use crate::systems::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Barnes-Hut N-Body".into(),
                resolution: WindowResolution::new(1000, 1000),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(ClearColor(Color::BLACK))
        .init_resource::<SimConfig>()
        .init_resource::<SimulationBounds>()
        .init_resource::<QuadTreeResource>()
        .init_resource::<SimSettings>()
        .init_resource::<ResetSimulation>()
        .add_systems(EguiPrimaryContextPass, ui_controls)
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                apply_reset_request,
                (
                    camera_controls,
                    draw_quadtree_gizmos,
                    draw_trails,
                    cull_bodies,
                    update_camera_follow,
                )
                    .chain()
                    .after(apply_reset_request),
            ),
        )
        .add_systems(
            FixedUpdate,
            (reset_and_build_tree, calculate_forces, integrate_motion).chain(),
        )
        .insert_resource(Time::<Fixed>::from_seconds(DEFAULT_DT as f64))
        .run();
}
