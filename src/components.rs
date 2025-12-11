use bevy::prelude::*;
use std::collections::VecDeque;

/// Position of a body in world space (2D).
#[derive(Component, Deref, DerefMut)]
pub struct Position(pub Vec2);

/// Linear velocity of a body.
#[derive(Component, Deref, DerefMut)]
pub struct Velocity(pub Vec2);

/// Linear acceleration of a body.
#[derive(Component, Deref, DerefMut)]
pub struct Acceleration(pub Vec2);

/// Mass of a body.
#[derive(Component, Deref, DerefMut)]
pub struct Mass(pub f32);

/// Trail history for rendering motion lines.
#[derive(Component)]
pub struct Trail {
    pub history: VecDeque<Vec2>,
    pub timer: Timer,
}

impl Default for Trail {
    fn default() -> Self {
        Self {
            history: VecDeque::new(),
            timer: Timer::from_seconds(0.05, TimerMode::Repeating),
        }
    }
}
