use bevy::ecs::system::SystemParam;
use bevy::input::mouse::MouseWheel;
use bevy::prelude::MessageReader;
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use rand::{Rng, SeedableRng, rngs::StdRng};

use crate::components::*;
use crate::quadtree::{NodeKind, QuadTreeResource, Rect};
use crate::resources::*;

/// Bundled system params used when resetting the simulation.
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

/// Spawns the initial galaxy of orbiting bodies plus a central massive body.
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

/// Sets up camera and populates the simulation with initial bodies.
pub fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2d);
    spawn_simulation_bodies(&mut commands, &mut meshes, &mut materials);
}

/// Recomputes quadtree bounds and inserts all current bodies.
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

/// Uses the quadtree to approximate gravitational forces and updates accelerations.
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

/// Integrates positions and velocities with a fixed timestep scaled by user settings.
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

/// Draws motion trails for bodies when enabled in settings.
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

/// Despawns bodies that travel beyond the configured culling distance.
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

/// Moves the camera toward the weighted center of mass when follow mode is enabled.
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

/// Handles manual camera pan/zoom input unless blocked by UI focus.
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

/// Renders quadtree node bounds as rectangles when gizmo display is enabled.
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

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::system::SystemState;
    use bevy::tasks::{ComputeTaskPool, TaskPoolBuilder};

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
    fn reset_and_build_tree_updates_bounds_and_root() {
        let mut world = World::new();
        world.insert_resource(QuadTreeResource::default());
        world.insert_resource(SimulationBounds::default());

        let entities = [
            (vec2(-10.0, -5.0), 2.0),
            (vec2(20.0, 15.0), 3.0),
        ];
        for (pos, mass) in entities {
            world.spawn((Position(pos), Mass(mass)));
        }

        let mut system_state: SystemState<(
            ResMut<QuadTreeResource>,
            ResMut<SimulationBounds>,
            Query<(Entity, &Position, &Mass)>,
        )> = SystemState::new(&mut world);

        {
            let (quadtree, bounds, query) = system_state.get_mut(&mut world);
            reset_and_build_tree(quadtree, bounds, query);
        }
        system_state.apply(&mut world);

        let quadtree = world.resource::<QuadTreeResource>();
        let bounds = world.resource::<SimulationBounds>();
        assert!(quadtree.root.is_some());
        assert!(quadtree.nodes.len() >= 3);

        let root_node = &quadtree.nodes[quadtree.root.unwrap()];
        assert_vec2_close(root_node.bounds.center, vec2(5.0, 5.0), 0.0001);
        assert!(
            (bounds.root.size.x - 33.0).abs() < 0.0001,
            "root size should expand to cover all bodies"
        );
    }

    #[test]
    fn calculate_forces_sets_acceleration_from_quadtree() {
        let mut world = World::new();
        ComputeTaskPool::get_or_init(|| TaskPoolBuilder::default().build());
        let mut quadtree = QuadTreeResource::default();
        let bounds = Rect {
            center: Vec2::ZERO,
            size: Vec2::splat(10.0),
        };
        quadtree.reset(bounds);

        let entity_a = world
            .spawn((Position(Vec2::ZERO), Mass(1.0), Acceleration(Vec2::ZERO)))
            .id();
        let entity_b = world
            .spawn((Position(vec2(3.0, 0.0)), Mass(2.0), Acceleration(Vec2::ZERO)))
            .id();

        quadtree.insert(entity_a, Vec2::ZERO, 1.0);
        quadtree.insert(entity_b, vec2(3.0, 0.0), 2.0);

        world.insert_resource(quadtree);
        world.insert_resource(SimConfig::default());

        let mut system_state: SystemState<(
            Query<(Entity, &Position, &Mass, &mut Acceleration)>,
            Res<QuadTreeResource>,
            Res<SimConfig>,
        )> = SystemState::new(&mut world);

        {
            let (query, quadtree, config) = system_state.get_mut(&mut world);
            calculate_forces(query, quadtree, config);
        }
        system_state.apply(&mut world);

        let acc_a = world
            .get::<Acceleration>(entity_a)
            .expect("acceleration present for entity A")
            .0;
        let acc_b = world
            .get::<Acceleration>(entity_b)
            .expect("acceleration present for entity B")
            .0;

        // Expected accelerations derived from one other body each.
        let expected_a_x = {
            let delta = vec2(3.0, 0.0);
            let dist_sq = delta.length_squared() + SOFTENING * SOFTENING;
            let dist = dist_sq.sqrt();
            let force_mag = (DEFAULT_G * 2.0) / dist_sq;
            (delta / dist * force_mag).x
        };
        let expected_b_x = {
            let delta = vec2(-3.0, 0.0);
            let dist_sq = delta.length_squared() + SOFTENING * SOFTENING;
            let dist = dist_sq.sqrt();
            let force_mag = (DEFAULT_G * 1.0) / dist_sq;
            (delta / dist * force_mag).x / 2.0
        };

        assert!((acc_a.x - expected_a_x).abs() < 0.0001);
        assert!((acc_b.x - expected_b_x).abs() < 0.0001);
    }
}

/// Responds to a pending reset: clears entities, resets resources, and respawns bodies.
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
