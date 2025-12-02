use bevy::ecs::system::SystemParam;
use bevy::input::mouse::MouseWheel;
use bevy::prelude::MessageReader;
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use rand::{Rng, SeedableRng, rngs::StdRng};

use crate::components::*;
use crate::quadtree::{NodeKind, QuadTreeResource, Rect};
use crate::resources::*;

#[derive(SystemParam)]
pub struct ResetParams<'w, 's> {
    pub commands: Commands<'w, 's>,
    pub meshes: ResMut<'w, Assets<Mesh>>,
    pub materials: ResMut<'w, Assets<ColorMaterial>>,
    pub settings: ResMut<'w, SimSettings>,
    pub sim_config: ResMut<'w, SimConfig>,
    pub bounds: ResMut<'w, SimulationBounds>,
    pub quadtree: ResMut<'w, QuadTreeResource>,
}

pub fn spawn_simulation_bodies(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<ColorMaterial>,
) {
    let mesh_handle = meshes.add(Circle::new(2.0));
    let mut rng = StdRng::from_os_rng();

    for _ in 0..NUM_BODIES {
        let angle = rng.random_range(0.0..std::f32::consts::TAU);
        let dist = rng.random_range(50.0..400.0);
        let arm_offset = (dist / 100.0) * 2.0;
        let final_angle = angle + arm_offset;
        let position = Vec2::new(final_angle.cos() * dist, final_angle.sin() * dist);

        let center_mass = 10000.0;
        let velocity_mag = (DEFAULT_G * center_mass / dist).sqrt();
        let velocity_dir = Vec2::new(-final_angle.sin(), final_angle.cos());

        let mass = rng.random_range(1.0..5.0);
        let color = Color::hsl(200.0 + mass * 20.0, 0.8, 0.6);
        let mat = materials.add(ColorMaterial::from(color));

        commands.spawn((
            Mesh2d(mesh_handle.clone()),
            MeshMaterial2d(mat),
            Transform::from_translation(position.extend(0.0)),
            Position(position),
            Velocity(velocity_dir * velocity_mag),
            Acceleration(Vec2::ZERO),
            Mass(mass),
            Trail::default(),
        ));
    }

    commands.spawn((
        Mesh2d(meshes.add(Circle::new(5.0))),
        MeshMaterial2d(materials.add(ColorMaterial::from(Color::srgb(1.0, 0.0, 0.5)))),
        Transform::from_translation(Vec3::ZERO),
        Position(Vec2::ZERO),
        Velocity(Vec2::ZERO),
        Acceleration(Vec2::ZERO),
        Mass(10000.0),
        Trail::default(),
    ));
}

pub fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2d);
    spawn_simulation_bodies(&mut commands, &mut meshes, &mut materials);
}

pub fn reset_and_build_tree(
    mut quadtree: ResMut<QuadTreeResource>,
    mut bounds: ResMut<SimulationBounds>,
    query: Query<(Entity, &Position, &Mass)>,
) {
    let mut min = Vec2::splat(f32::INFINITY);
    let mut max = Vec2::splat(f32::NEG_INFINITY);

    for (_, pos, _) in query.iter() {
        min = min.min(**pos);
        max = max.max(**pos);
    }

    if !min.x.is_finite() {
        quadtree.reset(bounds.root);
        return;
    }

    let root_bounds = {
        let size = (max - min).max(Vec2::splat(1.0));
        let max_dim = size.x.max(size.y) * 1.1;
        let center = (min + max) / 2.0;
        Rect {
            center,
            size: Vec2::splat(max_dim),
        }
    };

    bounds.root = root_bounds;
    quadtree.reset(root_bounds);

    for (entity, pos, mass) in query.iter() {
        quadtree.insert(entity, **pos, **mass);
    }
}

pub fn calculate_forces(
    mut query: Query<(Entity, &Position, &Mass, &mut Acceleration)>,
    quadtree: Res<QuadTreeResource>,
    config: Res<SimConfig>,
) {
    query
        .par_iter_mut()
        .for_each(|(entity, pos, mass, mut acc)| {
            let force = quadtree.calculate_force(entity, **pos, &config);
            acc.0 = force / **mass;
        });
}

pub fn integrate_motion(
    mut query: Query<(
        &mut Position,
        &mut Velocity,
        &mut Acceleration,
        &mut Transform,
    )>,
    settings: Res<SimSettings>,
    config: Res<SimConfig>,
) {
    let dt = config.dt * settings.time_scale;

    for (mut pos, mut vel, mut acc, mut transform) in query.iter_mut() {
        vel.0 += acc.0 * dt;
        pos.0 += vel.0 * dt;
        acc.0 = Vec2::ZERO;
        transform.translation = pos.extend(0.0);
    }
}

pub fn draw_trails(
    mut gizmos: Gizmos,
    mut query: Query<(&Position, &mut Trail)>,
    time: Res<Time>,
    settings: Res<SimSettings>,
) {
    if !settings.enable_trails {
        return;
    }

    for (pos, mut trail) in query.iter_mut() {
        trail.timer.tick(time.delta());
        if trail.timer.just_finished() {
            trail.history.push_back(**pos);
            if trail.history.len() > TRAIL_LENGTH {
                trail.history.pop_front();
            }
        }

        if trail.history.len() >= 2 {
            gizmos.linestrip_2d(
                trail.history.iter().copied(),
                Color::srgba(0.5, 0.8, 1.0, 0.3),
            );
        }
    }
}

pub fn cull_bodies(
    mut commands: Commands,
    query: Query<(Entity, &Position)>,
    settings: Res<SimSettings>,
) {
    if !settings.enable_culling {
        return;
    }

    for (entity, pos) in query.iter() {
        if pos.length() > CULL_DISTANCE {
            commands.entity(entity).despawn();
        }
    }
}

pub fn update_camera_follow(
    mut camera_query: Query<&mut Transform, (With<Camera>, Without<Position>)>,
    body_query: Query<(&Position, &Mass)>,
    settings: Res<SimSettings>,
    time: Res<Time>,
) {
    if !settings.follow_com {
        return;
    }

    let mut total_mass = 0.0;
    let mut weighted_pos = Vec2::ZERO;

    for (pos, mass) in body_query.iter() {
        total_mass += **mass;
        weighted_pos += **pos * **mass;
    }

    if total_mass > 0.0 {
        let com = weighted_pos / total_mass;
        if let Ok(mut cam_transform) = camera_query.single_mut() {
            let target = com.extend(0.0);
            let current = cam_transform.translation;
            let smooth_speed = 5.0 * time.delta_secs();

            cam_transform.translation = current.lerp(target, smooth_speed);
        }
    }
}

pub fn camera_controls(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut mouse_wheel: MessageReader<MouseWheel>,
    mut query: Query<&mut Transform, With<Camera>>,
    time: Res<Time>,
    settings: Res<SimSettings>,
    egui_input: Res<EguiWantsInput>,
) {
    if egui_input.wants_any_pointer_input() {
        return;
    }

    let manual_pan_enabled = !settings.follow_com;

    if let Ok(mut transform) = query.single_mut() {
        let mut scale = transform.scale.x;

        if manual_pan_enabled {
            let mut direction = Vec3::ZERO;
            if keyboard.pressed(KeyCode::ArrowLeft) || keyboard.pressed(KeyCode::KeyA) {
                direction.x -= 1.0;
            }
            if keyboard.pressed(KeyCode::ArrowRight) || keyboard.pressed(KeyCode::KeyD) {
                direction.x += 1.0;
            }
            if keyboard.pressed(KeyCode::ArrowUp) || keyboard.pressed(KeyCode::KeyW) {
                direction.y += 1.0;
            }
            if keyboard.pressed(KeyCode::ArrowDown) || keyboard.pressed(KeyCode::KeyS) {
                direction.y -= 1.0;
            }

            if direction.length_squared() > 0.0 {
                transform.translation += direction.normalize() * 500.0 * scale * time.delta_secs();
            }
        }

        for event in mouse_wheel.read() {
            if event.y.abs() == 0.0 {
                continue;
            }
            let zoom_factor = 1.1;
            if event.y > 0.0 {
                scale /= zoom_factor;
            } else {
                scale *= zoom_factor;
            }
        }

        let zoom_speed = 1.0 * time.delta_secs();
        if keyboard.pressed(KeyCode::KeyZ) {
            scale *= 1.0 - zoom_speed;
        }
        if keyboard.pressed(KeyCode::KeyX) {
            scale *= 1.0 + zoom_speed;
        }

        scale = scale.clamp(0.1, 10.0);
        transform.scale = Vec3::splat(scale);
    }
}

pub fn draw_quadtree_gizmos(
    mut gizmos: Gizmos,
    quadtree: Res<QuadTreeResource>,
    settings: Res<SimSettings>,
) {
    if !settings.show_gizmos {
        return;
    }

    let root_index = match quadtree.root {
        Some(idx) => idx,
        None => return,
    };

    let nodes = &quadtree.nodes;
    if nodes.is_empty() {
        return;
    }

    let color = Color::srgba(0.0, 1.0, 0.0, 0.1);
    let mut stack = Vec::new();
    stack.push(root_index);

    while let Some(idx) = stack.pop() {
        if let Some(node) = nodes.get(idx)
            && let NodeKind::Internal { children } = &node.kind
        {
            if node.bounds.size.x.max(node.bounds.size.y) >= MIN_GIZMO_NODE_SIZE {
                gizmos.rect_2d(
                    Isometry2d::from_translation(node.bounds.center),
                    node.bounds.size,
                    color,
                );
            }
            for child in children.iter().flatten() {
                stack.push(*child);
            }
        }
    }
}

pub fn apply_reset_request(
    params: ResetParams,
    mut reset: ResMut<ResetSimulation>,
    query: Query<Entity, With<Mass>>,
) {
    if !reset.pending {
        return;
    }
    reset.pending = false;

    let ResetParams {
        mut commands,
        mut meshes,
        mut materials,
        mut settings,
        mut sim_config,
        mut bounds,
        mut quadtree,
    } = params;

    *settings = SimSettings::default();
    *sim_config = SimConfig::default();
    *bounds = SimulationBounds::default();
    quadtree.reset(bounds.root);

    for entity in query.iter() {
        commands.entity(entity).despawn();
    }

    spawn_simulation_bodies(&mut commands, &mut meshes, &mut materials);
}
