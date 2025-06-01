use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    platform::collections::HashMap,
    prelude::*,
};
use bevy_egui::{
    EguiContextPass, EguiContexts, EguiPlugin,
    egui::{self, Slider, style::HandleShape},
};
use itertools::Itertools;
use kiddo::{KdTree, traits::DistanceMetric};
use rand::prelude::*;

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
    min_velocity: f32,
}

// type KdTree = kiddo::float::kdtree::KdTree<f32, u32, 2, 32, u32>;

#[derive(Resource)]
struct BoidKdTree(KdTree<f32, 2>);

const BOID_SIZE: f32 = 3.0;
const WINDOW_SIZE: f32 = 800.0;
pub const TOROIDAL_SIZE: f32 = WINDOW_SIZE + BOID_SIZE * 2.0;

pub fn run() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Boids".into(),
                resolution: (WINDOW_SIZE, WINDOW_SIZE).into(),
                resizable: false,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(EguiPlugin {
            enable_multipass_for_primary_context: true,
        })
        .insert_resource(BoidsParameters {
            alignment_factor: 1.0,
            cohesion_factor: 0.5,
            separation_factor: 1.5,
            separation_threshold: 200.0,
            local_distance: 850.0,
            num_boids: 10,
            speed: 25.0,
            min_velocity: 0.5,
        })
        .insert_resource(BoidKdTree(KdTree::new()))
        .add_systems(Startup, (spawn_boids, setup_camera))
        // .add_systems(Startup, spawn_boids)
        .add_systems(EguiContextPass, egui_system)
        .add_systems(Update, ((update_boids, build_kd_tree).chain(), wrap_boids))
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn(Camera2d::default());
}

fn spawn_boids(mut commands: Commands, boids_parameters: Res<BoidsParameters>) {
    let mut rng = rand::thread_rng();

    for _ in 0..boids_parameters.num_boids {
        let position = Vec3::new(
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
                color: Srgba::rgb(
                    rng.gen_range(0.5..1.0),
                    rng.gen_range(0.5..1.0),
                    rng.gen_range(0.5..1.0),
                )
                .into(),
                custom_size: Some(Vec2::splat(BOID_SIZE)),
                ..default()
            },
            Transform {
                translation: position,
                rotation: boid.quaternion_heading(),
                ..Default::default()
            },
            boid,
        ));
    }
}

#[derive(Debug, Clone)]
struct LocalBoid {
    heading: Vec2,
    position: Vec2,
    distance: f32,
}

impl PartialEq for LocalBoid {
    fn eq(&self, other: &Self) -> bool {
        self.distance == other.distance
    }
}
impl Eq for LocalBoid {}

impl PartialOrd for LocalBoid {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.distance.partial_cmp(&other.distance)
    }
}
impl Ord for LocalBoid {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other)
            .expect(format!("NaN DETECTED!!! {:?} {:?}", self, other).as_str())
    }
}

pub struct SquaredToroidal;

fn wrap_delta(delta: f32, size: f32) -> f32 {
    let half = size * 0.5;
    if delta < -half {
        delta + size
    } else if delta > half {
        delta - size
    } else {
        delta
    }
}

// Considers boids on either side of the screen to be close
impl DistanceMetric<f32, 2> for SquaredToroidal {
    fn dist(a: &[f32; 2], b: &[f32; 2]) -> f32 {
        let dx = wrap_delta(a[0] - b[0], TOROIDAL_SIZE);
        let dy = wrap_delta(a[1] - b[1], TOROIDAL_SIZE);
        dx * dx + dy * dy
    }

    fn dist1(a: f32, b: f32) -> f32 {
        (a - b + TOROIDAL_SIZE / 2.0).rem_euclid(TOROIDAL_SIZE) - TOROIDAL_SIZE / 2.0

        // Loses sign but is faster, sign not needed for kd pruning?
        // let delta = (a - b).abs();
        // let wrap = WINDOW_SIZE.max(WINDOW_SIZE);
        // delta.min(wrap - delta)
    }
}

fn local_n_boid_query(
    current_boid: Entity,
    current_boid_position: Vec2,
    all_boids: &HashMap<u64, (Vec2, Vec2)>,
    local_distance: f32,
    kd_tree: &KdTree<f32, 2>,
) -> Vec<LocalBoid> {
    let current_boid_array_position = current_boid_position.to_array();

    let nearest_neighbours = kd_tree
        .within_unsorted_iter::<SquaredToroidal>(&current_boid_array_position, local_distance);

    nearest_neighbours
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
    // dbg!(boids_parameters.num_boids);
    for (entity, transform) in boids_query {
        // dbg!(transform.translation.xy());
        // dbg!(entity.index());
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
) {
    let delta = time.delta_secs();

    let all_boids = HashMap::from_iter(boids_query.iter().map(|(entity, transform, boid)| {
        (
            entity.index() as u64,
            (transform.translation.xy(), boid.heading),
        )
    }));

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

            boid.heading += (alignment + cohesion + separation).normalize() * delta;
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
            Slider::new(&mut boids_parameters.speed, 1.0..=100.0)
                .step_by(5.0)
                .handle_shape(HandleShape::Rect { aspect_ratio: 0.35 })
                .text("Speed"),
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

            spawn_boids(commands, boids_parameters.into());
        }
    });
}
