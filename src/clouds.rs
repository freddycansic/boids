use bevy::{
    prelude::*,
    render::{
        mesh::MeshVertexBufferLayoutRef,
        render_resource::{
            AsBindGroup, BlendState, RenderPipelineDescriptor, ShaderRef, ShaderType,
            SpecializedMeshPipelineError,
        },
    },
    sprite::{AlphaMode2d, Material2d, Material2dKey, Material2dPlugin},
};

use crate::{BoidsParameters, WINDOW_SIZE};

pub struct CloudPlugin;

impl Plugin for CloudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(Material2dPlugin::<BackgroundCloudMaterial>::default())
            .add_plugins(Material2dPlugin::<OverlayCloudMaterial>::default())
            .add_systems(Startup, spawn_clouds)
            .add_systems(Update, animate_clouds);
    }
}

#[derive(Clone, ShaderType, Debug)]
struct PaddedOffset {
    offset: f32,
    _padding: Vec3,
}

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
struct BackgroundCloudMaterial {
    #[uniform(0)]
    padded_offset: PaddedOffset,
}

impl Material2d for BackgroundCloudMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/cloud_background.wgsl".into()
    }
}

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
struct OverlayCloudMaterial {
    #[uniform(0)]
    padded_offset: PaddedOffset,
}

impl Material2d for OverlayCloudMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/cloud_overlay.wgsl".into()
    }

    fn alpha_mode(&self) -> AlphaMode2d {
        AlphaMode2d::Blend
    }

    fn specialize(
        descriptor: &mut RenderPipelineDescriptor,
        _layout: &MeshVertexBufferLayoutRef,
        _key: Material2dKey<Self>,
    ) -> Result<(), SpecializedMeshPipelineError> {
        if let Some(fragment) = &mut descriptor.fragment {
            if let Some(target_state) = &mut fragment.targets[0] {
                target_state.blend = Some(BlendState::ALPHA_BLENDING);
            }
        }

        Ok(())
    }
}

#[derive(Component)]
struct Clouds;

fn spawn_clouds(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut background_material: ResMut<Assets<BackgroundCloudMaterial>>,
    mut overlay_material: ResMut<Assets<OverlayCloudMaterial>>,
) {
    commands.spawn((
        Mesh2d(meshes.add(Rectangle::new(WINDOW_SIZE, WINDOW_SIZE))),
        MeshMaterial2d(background_material.add(BackgroundCloudMaterial {
            padded_offset: PaddedOffset {
                offset: 0.0,
                _padding: Vec3::splat(0.0),
            },
        })),
        Transform::from_xyz(0.0, 0.0, -1.0),
        Clouds,
    ));

    commands.spawn((
        Mesh2d(meshes.add(Rectangle::new(WINDOW_SIZE, WINDOW_SIZE))),
        MeshMaterial2d(overlay_material.add(OverlayCloudMaterial {
            padded_offset: PaddedOffset {
                offset: 0.0,
                _padding: Vec3::splat(0.0),
            },
        })),
        Transform::from_xyz(0.0, 0.0, 1.0),
        Clouds,
    ));
}

fn animate_clouds(
    mut background_cloud_material: ResMut<Assets<BackgroundCloudMaterial>>,
    background_cloud: Single<&MeshMaterial2d<BackgroundCloudMaterial>, With<Clouds>>,
    mut overlay_cloud_material: ResMut<Assets<OverlayCloudMaterial>>,
    overlay_cloud: Single<&MeshMaterial2d<OverlayCloudMaterial>, With<Clouds>>,
    time: Res<Time>,
    boids_parameters: Res<BoidsParameters>,
) {
    const CLOUD_SPEED: f32 = 1.0 / 50.0;
    let timestep = boids_parameters.speed * time.delta_secs() * CLOUD_SPEED;

    if let Some(material) = background_cloud_material.get_mut(background_cloud.0.id()) {
        material.padded_offset.offset += timestep;
    }

    if let Some(material) = overlay_cloud_material.get_mut(overlay_cloud.0.id()) {
        material.padded_offset.offset += timestep;
    }
}
