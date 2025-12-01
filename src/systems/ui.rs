use bevy::prelude::*;
use bevy_egui::EguiContexts;
use bevy_egui::egui;

use crate::resources::{ResetSimulation, SimConfig, SimSettings};

pub fn ui_controls(
    mut contexts: EguiContexts,
    mut settings: ResMut<SimSettings>,
    mut sim_config: ResMut<SimConfig>,
    mut frames_rendered: Local<usize>,
    mut reset: ResMut<ResetSimulation>,
) {
    if *frames_rendered < 5 {
        *frames_rendered += 1;
        return;
    }

    if let Ok(ctx) = contexts.ctx_mut() {
        egui::Window::new("Simulation Controls")
            .default_pos(egui::pos2(10.0, 10.0))
            .max_size([320.0, 310.0])
            .vscroll(true)
            .show(ctx, |ui| {
                ui.heading("Simulation");
                ui.add(
                    egui::Slider::new(&mut settings.time_scale, 0.1..=5.0)
                        .text("Time Scale (Speed)"),
                );
                ui.add(egui::Slider::new(&mut sim_config.g, 10.0..=500.0).text("G (Gravity)"));
                ui.add(
                    egui::Slider::new(&mut sim_config.theta, 0.1..=1.0)
                        .text("Theta (Approximation)"),
                );

                ui.separator();
                ui.heading("Gizmos & Behaviors");
                ui.checkbox(&mut settings.enable_trails, "Enable Trails");
                ui.checkbox(&mut settings.enable_culling, "Enable Culling (>1500 units)");
                ui.checkbox(&mut settings.follow_com, "Follow Center of Mass");
                ui.checkbox(&mut settings.show_gizmos, "Show QuadTree Grid");

                ui.separator();
                ui.heading("Controls");
                ui.label("Pan: Arrow Keys / WASD");
                ui.label("Zoom: Scroll Wheel / Z & X");

                if ui.button("Reset Simulation").clicked() {
                    *settings = SimSettings::default();
                    *sim_config = SimConfig::default();
                    reset.pending = true;
                }
            });
    }
}
