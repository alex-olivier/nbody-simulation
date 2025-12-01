use bevy::prelude::*;
use std::collections::VecDeque;

#[derive(Component, Deref, DerefMut)]
pub struct Position(pub Vec2);

#[derive(Component, Deref, DerefMut)]
pub struct Velocity(pub Vec2);

#[derive(Component, Deref, DerefMut)]
pub struct Acceleration(pub Vec2);

#[derive(Component, Deref, DerefMut)]
pub struct Mass(pub f32);

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
