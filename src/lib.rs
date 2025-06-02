mod squared_toroidal;

use std::time::Duration;

use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    ecs::entity::EntityHashSet,
    platform::collections::HashMap,
    prelude::*,
    render::render_resource::{AsBindGroup, ShaderRef},
    sprite::{Material2d, Material2dPlugin},
    window::PrimaryWindow,
};
use bevy_egui::{
    EguiContextPass, EguiContexts, EguiPlugin,
    egui::{self, Slider, style::HandleShape},
};
use itertools::Itertools;
use kiddo::{KdTree, SquaredEuclidean};
use rand::prelude::*;

use crate::squared_toroidal::SquaredToroidal;

#[derive(Component)]
struct Boid {
    heading: Vec2,
}

impl Boid {
    fn quaternion_heading(&self) -> Quat {
        let angle = self.heading.y.atan2(self.heading.x);

        Quat::from_rotation_z(angle)
    }
}

#[derive(Resource)]
struct BoidsParameters {
    alignment_factor: f32,
    cohesion_factor: f32,
    separation_factor: f32,
    separation_threshold: f32,
    local_distance: f32,
    num_boids: usize,
    speed: f32,
    repulsion_factor: f32,
    repulsion_distance: f32,
    min_velocity: f32,
}

#[derive(Resource)]
struct BoidKdTree(KdTree<f32, 2>);

#[derive(Event)]
struct SpawnBoidsEvent;

const BOID_SIZE: f32 = 10.0;
const WINDOW_SIZE: f32 = 800.0;
pub const TOROIDAL_SIZE: f32 = WINDOW_SIZE + BOID_SIZE * 2.0;

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
struct CloudMaterial {}

impl Material2d for CloudMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/clouds.wgsl".into()
    }
}

pub fn run() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Boids".into(),
                    resolution: (WINDOW_SIZE, WINDOW_SIZE).into(),
                    resizable: false,
                    ..default()
                }),
                ..default()
            }),
            Material2dPlugin::<CloudMaterial>::default(),
        ))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(EguiPlugin {
            enable_multipass_for_primary_context: true,
        })
        .insert_resource(BoidsParameters {
            alignment_factor: 1.0,
            cohesion_factor: 0.5,
            separation_factor: 1.5,
            separation_threshold: 200.0,
            local_distance: 400.0,
            num_boids: 1000,
            repulsion_factor: 3.0,
            repulsion_distance: 500.0,
            speed: 25.0,
            min_velocity: 0.5,
        })
        .insert_resource(BoidKdTree(KdTree::new()))
        .add_event::<SpawnBoidsEvent>()
        .add_systems(Startup, (spawn_clouds, spawn_boids, setup_camera))
        .add_systems(EguiContextPass, egui_system)
        .add_systems(
            Update,
            (
                (build_kd_tree, update_boids).chain(),
                wrap_boids,
                spawn_boids_on_event,
                animate_sprite,
            ),
        )
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2d::default());
}

fn spawn_clouds(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<CloudMaterial>>,
) {
    commands.spawn((
        Mesh2d(meshes.add(Rectangle::new(WINDOW_SIZE, WINDOW_SIZE))),
        MeshMaterial2d(materials.add(CloudMaterial {})),
    ));
}

#[derive(Component, Clone)]
struct AnimationConfig {
    first: usize,
    last: usize,
    timer: Timer,
}

fn spawn_boids_on_event(
    mut spawn_boids_reader: EventReader<SpawnBoidsEvent>,
    commands: Commands,
    boids_parameters: Res<BoidsParameters>,
    asset_server: Res<AssetServer>,
    texture_atlas_layouts: ResMut<Assets<TextureAtlasLayout>>,
) {
    if !spawn_boids_reader.is_empty() {
        spawn_boids(
            commands,
            boids_parameters,
            asset_server,
            texture_atlas_layouts,
        );
        spawn_boids_reader.clear();
    }
}

fn spawn_boids(
    mut commands: Commands,
    boids_parameters: Res<BoidsParameters>,
    asset_server: Res<AssetServer>,
    mut texture_atlas_layouts: ResMut<Assets<TextureAtlasLayout>>,
) {
    let mut rng = rand::thread_rng();

    let texture = asset_server.load("textures/birds.png");
    let layout = TextureAtlasLayout::from_grid(UVec2::splat(120), 10, 1, None, None);
    let texture_atlas_layout = texture_atlas_layouts.add(layout);

    let fps = 10.0;
    let animation_config = AnimationConfig {
        first: 0,
        last: 9,
        timer: Timer::new(
            Duration::from_secs_f32(1.0 / (fps as f32)),
            TimerMode::Repeating,
        ),
    };

    for _ in 0..boids_parameters.num_boids {
        let translation = Vec3::new(
            rng.gen_range(-WINDOW_SIZE / 2.0..WINDOW_SIZE / 2.0),
            rng.gen_range(-WINDOW_SIZE / 2.0..WINDOW_SIZE / 2.0),
            0.0,
        );

        let boid = Boid {
            heading: Vec2 {
                x: rng.gen_range(-1.0..1.0),
                y: rng.gen_range(-1.0..1.0),
            }
            .normalize(),
        };

        commands.spawn((
            Sprite {
                // color: Srgba::rgb(
                //     // rng.gen_range(0.5..1.0),
                //     // rng.gen_range(0.5..1.0),
                //     // rng.gen_range(0.5..1.0),
                //     0.0, 0.0, 0.0,
                // )
                image: texture.clone(),
                texture_atlas: Some(TextureAtlas {
                    layout: texture_atlas_layout.clone(),
                    index: rng.gen_range(0..=9),
                    // index: animation_config.first,
                }),
                custom_size: Some(Vec2::splat(BOID_SIZE)),
                ..default()
            },
            Transform {
                translation,
                rotation: boid.quaternion_heading(),
                ..Default::default()
            },
            animation_config.clone(),
            boid,
        ));
    }
}

fn animate_sprite(time: Res<Time>, mut query: Query<(&mut AnimationConfig, &mut Sprite)>) {
    for (mut animation_config, mut sprite) in &mut query {
        animation_config.timer.tick(time.delta());

        if animation_config.timer.just_finished() {
            if let Some(atlas) = &mut sprite.texture_atlas {
                atlas.index = if atlas.index == animation_config.last {
                    animation_config.first
                } else {
                    atlas.index + 1
                };
            }
        }
    }
}

#[derive(Debug, Clone)]
struct LocalBoid {
    heading: Vec2,
    position: Vec2,
    distance: f32,
}

fn local_n_boid_query(
    current_boid: Entity,
    current_boid_position: Vec2,
    all_boids: &HashMap<u64, (Vec2, Vec2)>,
    local_distance: f32,
    kd_tree: &KdTree<f32, 2>,
) -> Vec<LocalBoid> {
    let current_boid_array_position = current_boid_position.to_array();

    kd_tree
        .within_unsorted::<SquaredToroidal>(&current_boid_array_position, local_distance)
        .into_iter()
        .filter_map(|neighbour| {
            if neighbour.item == current_boid.index() as u64 {
                None
            } else {
                all_boids
                    .get(&neighbour.item)
                    .map(|(position, heading)| LocalBoid {
                        distance: neighbour.distance,
                        heading: *heading,
                        position: *position,
                    })
            }
        })
        .collect_vec()
}

fn build_kd_tree(
    mut kd_tree: ResMut<BoidKdTree>,
    boids_query: Query<(Entity, &Transform), With<Boid>>,
    boids_parameters: Res<BoidsParameters>,
) {
    kd_tree.0 = KdTree::with_capacity(boids_parameters.num_boids);
    for (entity, transform) in boids_query {
        kd_tree.0.add(
            &transform.translation.xy().to_array(),
            entity.index() as u64,
        )
    }
}

fn calculate_alignment(
    local_query: &Vec<LocalBoid>,
    boid_heading: Vec2,
    alignment_factor: f32,
) -> Vec2 {
    let average_local_heading = if local_query.is_empty() {
        Vec2::splat(0.0)
    } else {
        local_query.iter().map(|q| q.heading).sum::<Vec2>() / local_query.len() as f32
    };

    let alignment_error = (average_local_heading - boid_heading).normalize();

    alignment_error * alignment_factor
}

fn calculate_cohesion(
    local_query: &Vec<LocalBoid>,
    boid_position: Vec2,
    cohesion_factor: f32,
) -> Vec2 {
    let average_position_of_local = if local_query.is_empty() {
        Vec2::splat(0.0)
    } else {
        local_query.iter().map(|q| q.position).sum::<Vec2>() / local_query.len() as f32
    };

    // Clamp to max 1.0 instead of normalising as to reduce cohesion when it is not necessary i.e. already close to a flock
    let cohesion_error = (average_position_of_local - boid_position).clamp_length_max(1.0);

    cohesion_error * cohesion_factor
}

fn calculate_separation(
    local_query: &Vec<LocalBoid>,
    boid_position: Vec2,
    separation_factor: f32,
    separation_threshold: f32,
) -> Vec2 {
    let too_close_boids = local_query
        .iter()
        .filter(|q| q.distance <= separation_threshold)
        .collect_vec();

    if too_close_boids.is_empty() {
        return Vec2::splat(0.0);
    }

    too_close_boids
        .into_iter()
        .map(|too_close_boid| {
            let difference = boid_position - too_close_boid.position;

            if difference.length_squared() > 0.0 {
                // Inversely proportional to the difference i.e. move further away from closer boids
                difference.normalize() / too_close_boid.distance
            } else {
                Vec2::splat(0.0)
            }
        })
        .sum::<Vec2>()
        .try_normalize()
        .unwrap_or(Vec2::splat(0.0))
        * separation_factor
}

fn update_boids(
    mut boids_query: Query<(Entity, &mut Transform, &mut Boid)>,
    time: Res<Time>,
    boids_parameters: Res<BoidsParameters>,
    kd_tree: Res<BoidKdTree>,
    windows: Query<&Window, With<PrimaryWindow>>,
) {
    let delta = time.delta_secs();

    let all_boids = HashMap::from_iter(boids_query.iter().map(|(entity, transform, boid)| {
        (
            entity.index() as u64,
            (transform.translation.xy(), boid.heading),
        )
    }));

    let mouse_position = windows.single().ok().and_then(|window| {
        window.cursor_position().map(|mouse_position| {
            Vec2::new(
                mouse_position.x - WINDOW_SIZE / 2.0,
                -mouse_position.y + WINDOW_SIZE / 2.0,
            )
        })
    });

    let boids_near_cursor = if let Some(mouse_position) = mouse_position {
        let mouse_position_array = mouse_position.to_array();
        EntityHashSet::from_iter(
            kd_tree
                .0
                .within_unsorted::<SquaredEuclidean>(
                    &mouse_position_array,
                    boids_parameters.repulsion_distance,
                )
                .into_iter()
                .map(|neighbour| Entity::from_raw(neighbour.item as u32)),
        )
    } else {
        EntityHashSet::new()
    };

    boids_query
        .par_iter_mut()
        .for_each(|(entity, mut transform, mut boid)| {
            let local_query = local_n_boid_query(
                entity,
                transform.translation.xy(),
                &all_boids,
                boids_parameters.local_distance,
                &kd_tree.0,
            );

            let alignment = calculate_alignment(
                &local_query,
                boid.heading,
                boids_parameters.alignment_factor,
            );

            let cohesion = calculate_cohesion(
                &local_query,
                transform.translation.xy(),
                boids_parameters.cohesion_factor,
            );

            let separation = calculate_separation(
                &local_query,
                transform.translation.xy(),
                boids_parameters.separation_factor,
                boids_parameters.separation_threshold,
            );

            let repulsion = if let Some(mouse_position) = mouse_position {
                if boids_near_cursor.contains(&entity) {
                    (transform.translation.xy() - mouse_position).normalize()
                        * boids_parameters.repulsion_factor
                } else {
                    Vec2::splat(0.0)
                }
            } else {
                Vec2::splat(0.0)
            };

            boid.heading += ((alignment + cohesion + separation).normalize() + repulsion) * delta;
            // Min velocity
            boid.heading = boid.heading.clamp_length_min(boids_parameters.min_velocity);

            transform.translation.x += boid.heading.x * delta * boids_parameters.speed;
            transform.translation.y += boid.heading.y * delta * boids_parameters.speed;
            transform.rotation = boid.quaternion_heading();
        });
}

fn wrap_boids(mut boids_query: Query<&mut Transform, With<Boid>>) {
    for mut boid in &mut boids_query {
        if boid.translation.x < -WINDOW_SIZE / 2.0 - BOID_SIZE {
            boid.translation.x += WINDOW_SIZE + BOID_SIZE * 2.0;
        }

        if boid.translation.x > WINDOW_SIZE / 2.0 + BOID_SIZE {
            boid.translation.x -= WINDOW_SIZE + BOID_SIZE * 2.0;
        }

        if boid.translation.y < -WINDOW_SIZE / 2.0 - BOID_SIZE {
            boid.translation.y += WINDOW_SIZE + BOID_SIZE * 2.0;
        }

        if boid.translation.y > WINDOW_SIZE / 2.0 + BOID_SIZE {
            boid.translation.y -= WINDOW_SIZE + BOID_SIZE * 2.0;
        }
    }
}

fn egui_system(
    mut contexts: EguiContexts,
    mut boids_parameters: ResMut<BoidsParameters>,
    mut commands: Commands,
    boids_query: Query<Entity, With<Boid>>,
    diagnostics: Res<DiagnosticsStore>,
    mut exit_writer: EventWriter<AppExit>,
    mut spawn_boids_writer: EventWriter<SpawnBoidsEvent>,
) {
    egui::Window::new("Boids").show(contexts.ctx_mut(), |ui| {
        ui.label("Performance Statistics");
        ui.label(format!(
            "FPS: {:.1}",
            diagnostics
                .get(&FrameTimeDiagnosticsPlugin::FPS)
                .and_then(|fps| fps.value())
                .unwrap_or(0.0)
        ));
        ui.separator();

        ui.label("Simulation Settings");
        ui.add(
            Slider::new(&mut boids_parameters.num_boids, 0..=5000)
                .integer()
                .step_by(10.0)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Num boids"),
        );
        ui.add(
            Slider::new(&mut boids_parameters.speed, 1.0..=500.0)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Speed"),
        );
        ui.add(
            Slider::new(&mut boids_parameters.repulsion_factor, 0.0..=5.0)
                .step_by(0.2)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Repulsion factor"),
        );
        ui.add(
            Slider::new(
                &mut boids_parameters.repulsion_distance,
                0.0..=(WINDOW_SIZE / 2.0 * WINDOW_SIZE / 2.0),
            )
            .step_by(10.0)
            .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
            .text("Repulsion distance"),
        );

        ui.separator();

        ui.label("Boid Settings");
        ui.add(
            Slider::new(&mut boids_parameters.min_velocity, 0.0..=5.0)
                .step_by(0.5)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Minimum velocity"),
        );
        ui.add(
            Slider::new(&mut boids_parameters.local_distance, 0.0..=1000.0)
                .step_by(10.0)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Local distance"),
        );
        ui.add(
            Slider::new(&mut boids_parameters.alignment_factor, 0.0..=2.0)
                .step_by(0.01)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Alignment factor"),
        );
        ui.add(
            Slider::new(&mut boids_parameters.cohesion_factor, 0.0..=2.0)
                .step_by(0.01)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Cohesion factor"),
        );
        ui.add(
            Slider::new(&mut boids_parameters.separation_factor, 0.0..=2.0)
                .step_by(0.01)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Separation factor"),
        );
        ui.add(
            Slider::new(&mut boids_parameters.separation_threshold, 0.0..=1000.0)
                .step_by(10.0)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Separation threshold"),
        );

        if ui.button("Reset Boids").clicked() {
            for entity in boids_query.iter() {
                commands.entity(entity).despawn();
            }

            spawn_boids_writer.write(SpawnBoidsEvent);
        }

        ui.separator();

        if ui.button("Exit").clicked() {
            exit_writer.write(AppExit::Success);
        }
    });
}
