use bevy::{input::mouse::MouseWheel, prelude::*, window::WindowResolution};
use bevy_egui::{EguiContexts, EguiPlugin, egui};
use rand::Rng;
use std::collections::VecDeque;

// --- Simulation Constants ---
const G_CONSTANT: f32 = 100.0;
const SOFTENING: f32 = 5.0;
const THETA: f32 = 0.5;
const NUM_BODIES: usize = 2000;
const SIMULATION_STEP: f32 = 1.0 / 60.0;
const TRAIL_LENGTH: usize = 20; // How many past positions to store
const CULL_DISTANCE: f32 = 1500.0; // Distance at which bodies are despawned

// --- Resources ---

#[derive(Resource)]
struct SimSettings {
    enable_trails: bool,
    enable_culling: bool,
    follow_com: bool, // Center of Mass
    show_gizmos: bool,
}

impl Default for SimSettings {
    fn default() -> Self {
        Self {
            enable_trails: false,
            enable_culling: false,
            follow_com: false,
            show_gizmos: false,
        }
    }
}

// --- Main ---

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
        .init_resource::<SimSettings>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                camera_controls,
                draw_ui,
                handle_settings_input,
                draw_quadtree_gizmos,
                draw_trails,
                cull_bodies,
                update_camera_follow,
            ),
        )
        .add_systems(FixedUpdate, step_simulation)
        .insert_resource(Time::<Fixed>::from_seconds(SIMULATION_STEP as f64))
        .run();
}

// --- Components ---

#[derive(Component)]
struct Body {
    mass: f32,
}

#[derive(Component, Deref, DerefMut)]
struct Velocity(Vec2);

#[derive(Component, Deref, DerefMut)]
struct Acceleration(Vec2);

#[derive(Component)]
struct Trail {
    history: VecDeque<Vec2>,
    timer: Timer,
}

impl Default for Trail {
    fn default() -> Self {
        Self {
            history: VecDeque::new(),
            timer: Timer::from_seconds(0.05, TimerMode::Repeating), // Record point every 0.05s
        }
    }
}

// --- Quadtree Implementation ---

#[derive(Clone, Copy, Debug)]
struct Rect {
    center: Vec2,
    size: Vec2,
}

impl Rect {
    fn get_quadrant_index(&self, point: Vec2) -> usize {
        let right = point.x > self.center.x;
        let top = point.y > self.center.y;
        match (right, top) {
            (false, true) => 0,  // NW
            (true, true) => 1,   // NE
            (false, false) => 2, // SW
            (true, false) => 3,  // SE
        }
    }

    fn sub_quadrant(&self, index: usize) -> Rect {
        let quarter_size = self.size / 2.0;
        let offset = quarter_size / 2.0;
        let center = match index {
            0 => self.center + vec2(-offset.x, offset.y),  // NW
            1 => self.center + vec2(offset.x, offset.y),   // NE
            2 => self.center + vec2(-offset.x, -offset.y), // SW
            3 => self.center + vec2(offset.x, -offset.y),  // SE
            _ => panic!("Invalid quadrant"),
        };
        Rect {
            center,
            size: quarter_size,
        }
    }
}

// Renamed to BhNode (Barnes-Hut Node) to avoid conflict with Bevy's UI Node
enum BhNode {
    Empty,
    Leaf {
        entity: Entity,
        pos: Vec2,
        mass: f32,
    },
    Internal {
        children: Box<[BhNode; 4]>,
        center_of_mass: Vec2,
        total_mass: f32,
    },
}

struct Quadtree {
    root: BhNode,
    bounds: Rect,
}

impl BhNode {
    fn insert(&mut self, entity: Entity, pos: Vec2, mass: f32, bounds: Rect) {
        match self {
            BhNode::Empty => {
                *self = BhNode::Leaf { entity, pos, mass };
            }
            BhNode::Leaf {
                entity: e_old,
                pos: p_old,
                mass: m_old,
            } => {
                if (*p_old - pos).length_squared() < 0.0001 {
                    return;
                }

                let old_entity = *e_old;
                let old_pos = *p_old;
                let old_mass = *m_old;

                let mut children =
                    Box::new([BhNode::Empty, BhNode::Empty, BhNode::Empty, BhNode::Empty]);

                let old_idx = bounds.get_quadrant_index(old_pos);
                children[old_idx].insert(
                    old_entity,
                    old_pos,
                    old_mass,
                    bounds.sub_quadrant(old_idx),
                );

                let new_idx = bounds.get_quadrant_index(pos);
                children[new_idx].insert(entity, pos, mass, bounds.sub_quadrant(new_idx));

                let total_mass = old_mass + mass;
                let com = (old_pos * old_mass + pos * mass) / total_mass;

                *self = BhNode::Internal {
                    children,
                    center_of_mass: com,
                    total_mass,
                };
            }
            BhNode::Internal {
                children,
                center_of_mass,
                total_mass,
            } => {
                let new_total = *total_mass + mass;
                *center_of_mass = (*center_of_mass * *total_mass + pos * mass) / new_total;
                *total_mass = new_total;

                let idx = bounds.get_quadrant_index(pos);
                children[idx].insert(entity, pos, mass, bounds.sub_quadrant(idx));
            }
        }
    }
}

impl Quadtree {
    fn new(bodies: &[(Entity, Vec2, f32)]) -> Self {
        if bodies.is_empty() {
            return Quadtree {
                root: BhNode::Empty,
                bounds: Rect {
                    center: Vec2::ZERO,
                    size: Vec2::ONE,
                },
            };
        }

        let mut min = bodies[0].1;
        let mut max = bodies[0].1;

        for (_, pos, _) in bodies {
            min = min.min(*pos);
            max = max.max(*pos);
        }

        let size = max - min;
        let max_dim = size.x.max(size.y).max(1.0) * 1.1;
        let center = (min + max) / 2.0;
        let bounds = Rect {
            center,
            size: Vec2::splat(max_dim),
        };

        let mut root = BhNode::Empty;
        for (entity, pos, mass) in bodies {
            root.insert(*entity, *pos, *mass, bounds);
        }

        Quadtree { root, bounds }
    }

    fn calculate_force(&self, point: Vec2, mass: f32, target_entity: Entity) -> Vec2 {
        self.calculate_force_recursive(&self.root, &self.bounds, point, mass, target_entity)
    }

    fn calculate_force_recursive(
        &self,
        node: &BhNode,
        bounds: &Rect,
        point: Vec2,
        _mass: f32,
        target_entity: Entity,
    ) -> Vec2 {
        match node {
            BhNode::Empty => Vec2::ZERO,
            BhNode::Leaf {
                entity,
                pos,
                mass: leaf_mass,
            } => {
                if *entity == target_entity {
                    return Vec2::ZERO;
                }
                let delta = *pos - point;
                let dist_sq = delta.length_squared() + SOFTENING * SOFTENING;
                let dist = dist_sq.sqrt();
                let f = (G_CONSTANT * leaf_mass) / dist_sq;
                delta / dist * f
            }
            BhNode::Internal {
                children,
                center_of_mass,
                total_mass,
            } => {
                let delta = *center_of_mass - point;
                let dist = delta.length();
                let width = bounds.size.x;

                if width / dist < THETA {
                    let dist_sq = dist * dist + SOFTENING * SOFTENING;
                    let f = (G_CONSTANT * total_mass) / dist_sq;
                    delta / dist * f
                } else {
                    let mut force = Vec2::ZERO;
                    for i in 0..4 {
                        force += self.calculate_force_recursive(
                            &children[i],
                            &bounds.sub_quadrant(i),
                            point,
                            _mass,
                            target_entity,
                        );
                    }
                    force
                }
            }
        }
    }
}

// --- Systems ---

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // Camera
    commands.spawn(Camera2d);

    let mesh_handle = meshes.add(Circle::new(2.0));

    // Updated for rand 0.9.x
    let mut rng = rand::rng();

    // Spawn Galaxy
    for _ in 0..NUM_BODIES {
        // Updated for rand 0.9.x
        let angle = rng.random_range(0.0..std::f32::consts::TAU);
        let dist = rng.random_range(50.0..400.0);
        let arm_offset = (dist / 100.0) * 2.0;
        let final_angle = angle + arm_offset;
        let position = Vec2::new(final_angle.cos() * dist, final_angle.sin() * dist);

        let center_mass = 10000.0;
        let velocity_mag = (G_CONSTANT * center_mass / dist).sqrt();
        let velocity_dir = Vec2::new(-final_angle.sin(), final_angle.cos());

        let mass = rng.random_range(1.0..5.0);
        let color = Color::hsl(200.0 + mass * 20.0, 0.8, 0.6);
        let mat = materials.add(ColorMaterial::from(color));

        commands.spawn((
            Mesh2d(mesh_handle.clone()),
            MeshMaterial2d(mat),
            Transform::from_translation(position.extend(0.0)),
            Body { mass },
            Velocity(velocity_dir * velocity_mag),
            Acceleration(Vec2::ZERO),
            Trail::default(),
        ));
    }

    // Black Hole
    commands.spawn((
        Mesh2d(meshes.add(Circle::new(5.0))),
        MeshMaterial2d(materials.add(ColorMaterial::from(Color::srgb(1.0, 0.0, 0.5)))),
        Transform::from_translation(Vec3::ZERO),
        Body { mass: 10000.0 },
        Velocity(Vec2::ZERO),
        Acceleration(Vec2::ZERO),
        Trail::default(),
    ));
}

fn handle_settings_input(keyboard: Res<ButtonInput<KeyCode>>, mut settings: ResMut<SimSettings>) {
    if keyboard.just_pressed(KeyCode::KeyT) {
        settings.enable_trails = !settings.enable_trails;
    }
    if keyboard.just_pressed(KeyCode::KeyC) {
        settings.enable_culling = !settings.enable_culling;
    }
    if keyboard.just_pressed(KeyCode::KeyM) {
        settings.follow_com = !settings.follow_com;
    }
    if keyboard.just_pressed(KeyCode::KeyG) {
        settings.show_gizmos = !settings.show_gizmos;
    }
}

fn draw_ui(
    mut contexts: EguiContexts,
    mut settings: ResMut<SimSettings>,
    mut frames_rendered: Local<usize>, // FIXED: Frame counter to prevent early panic
) {
    // Prevent panic on first frame(s) where egui::begin_frame hasn't run yet
    *frames_rendered += 1;
    if *frames_rendered < 5 {
        return;
    }

    // FIXED: Unwrapping the context Result safely
    if let Ok(ctx) = contexts.ctx_mut() {
        egui::Window::new("Simulation Controls")
            .default_pos(egui::pos2(10.0, 10.0))
            .show(ctx, |ui| {
                ui.heading("Settings");
                ui.checkbox(&mut settings.enable_trails, "Enable Trails [T]");
                ui.checkbox(
                    &mut settings.enable_culling,
                    "Enable Culling (>1500 units) [C]",
                );
                ui.checkbox(&mut settings.follow_com, "Follow Center of Mass [M]");
                ui.checkbox(&mut settings.show_gizmos, "Show Quadtree Grid [G]");

                ui.separator();
                ui.heading("Controls");
                ui.label("Pan: Arrow Keys / WASD");
                ui.label("Zoom: Scroll Wheel / Z & X");
            });
    }
}

fn step_simulation(
    mut query: Query<(
        Entity,
        &mut Transform,
        &mut Velocity,
        &mut Acceleration,
        &Body,
    )>,
) {
    let bodies_snapshot: Vec<(Entity, Vec2, f32)> = query
        .iter()
        .map(|(e, t, _, _, b)| (e, t.translation.truncate(), b.mass))
        .collect();

    let tree = Quadtree::new(&bodies_snapshot);

    query
        .par_iter_mut()
        .for_each(|(entity, _, _, mut acc, body)| {
            if let Some((_, pos, _)) = bodies_snapshot.iter().find(|(e, _, _)| *e == entity) {
                let force = tree.calculate_force(*pos, body.mass, entity);
                acc.0 = force / body.mass;
            }
        });

    for (_, mut transform, mut velocity, acceleration, _) in query.iter_mut() {
        velocity.0 += acceleration.0 * SIMULATION_STEP;
        transform.translation += velocity.0.extend(0.0) * SIMULATION_STEP;
    }
}

fn draw_trails(
    mut gizmos: Gizmos,
    mut query: Query<(&Transform, &mut Trail)>,
    time: Res<Time>,
    settings: Res<SimSettings>,
) {
    if !settings.enable_trails {
        return;
    }

    for (transform, mut trail) in query.iter_mut() {
        trail.timer.tick(time.delta());
        if trail.timer.just_finished() {
            trail.history.push_back(transform.translation.truncate());
            if trail.history.len() > TRAIL_LENGTH {
                trail.history.pop_front();
            }
        }

        // Draw the trail
        if trail.history.len() >= 2 {
            gizmos.linestrip_2d(
                trail.history.iter().copied(),
                Color::srgba(0.5, 0.8, 1.0, 0.3),
            );
        }
    }
}

fn cull_bodies(
    mut commands: Commands,
    query: Query<(Entity, &Transform)>,
    settings: Res<SimSettings>,
) {
    if !settings.enable_culling {
        return;
    }

    for (entity, transform) in query.iter() {
        if transform.translation.length() > CULL_DISTANCE {
            commands.entity(entity).despawn();
        }
    }
}

fn update_camera_follow(
    mut camera_query: Query<&mut Transform, (With<Camera>, Without<Body>)>,
    body_query: Query<(&Transform, &Body)>,
    settings: Res<SimSettings>,
    time: Res<Time>,
) {
    if !settings.follow_com {
        return;
    }

    let mut total_mass = 0.0;
    let mut weighted_pos = Vec2::ZERO;

    for (transform, body) in body_query.iter() {
        total_mass += body.mass;
        weighted_pos += transform.translation.truncate() * body.mass;
    }

    if total_mass > 0.0 {
        let com = weighted_pos / total_mass;
        if let Ok(mut cam_transform) = camera_query.single_mut() {
            // Smoothly interpolate camera position
            let target = com.extend(0.0);
            let current = cam_transform.translation;
            let smooth_speed = 5.0 * time.delta_secs();

            cam_transform.translation = current.lerp(target, smooth_speed);
        }
    }
}

fn camera_controls(
    mut contexts: EguiContexts,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut mouse_wheel: EventReader<MouseWheel>,
    mut query: Query<&mut Transform, With<Camera>>,
    time: Res<Time>,
    settings: Res<SimSettings>,
    frames_rendered: Local<usize>, // FIXED: Also prevent input check on first frames
) {
    // Prevent access before init
    if *frames_rendered < 5 {
        return;
    }

    // FIXED: Safely check if Egui wants input by unwrapping the context
    if let Ok(ctx) = contexts.ctx_mut() {
        if ctx.wants_pointer_input() || ctx.is_pointer_over_area() {
            return;
        }
    }

    // Disable manual pan if following mass
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

        // Zoom Controls
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

fn draw_quadtree_gizmos(
    mut gizmos: Gizmos,
    query: Query<(&Transform, &Body)>,
    settings: Res<SimSettings>,
) {
    if !settings.show_gizmos {
        return;
    }

    let bodies: Vec<(Vec2, f32)> = query
        .iter()
        .map(|(t, b)| (t.translation.truncate(), b.mass))
        .collect();

    if bodies.is_empty() {
        return;
    }

    let mut min = bodies[0].0;
    let mut max = bodies[0].0;
    for (pos, _) in &bodies {
        min = min.min(*pos);
        max = max.max(*pos);
    }

    let center = (min + max) / 2.0;
    let size = max - min;

    gizmos.rect_2d(
        Isometry2d::from_translation(center),
        size,
        Color::srgba(0.0, 1.0, 0.0, 0.1),
    );
}
