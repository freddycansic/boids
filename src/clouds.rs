use bevy::{
    prelude::*,
    render::{
        mesh::{MeshVertexBufferLayout, MeshVertexBufferLayoutRef},
        render_resource::{
            AsBindGroup, BlendComponent, BlendFactor, BlendOperation, BlendState, ColorTargetState,
            ColorWrites, RenderPipelineDescriptor, ShaderRef, ShaderType,
            SpecializedMeshPipelineError, TextureFormat,
        },
    },
    sprite::{AlphaMode2d, Material2d, Material2dKey, Material2dPlugin},
};

use crate::{BoidsParameters, WINDOW_SIZE};

pub struct CloudPlugin;

impl Plugin for CloudPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(Material2dPlugin::<CloudMaterial>::default())
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
struct CloudMaterial {
    #[uniform(0)]
    padded_offset: PaddedOffset,
}

impl Material2d for CloudMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/clouds.wgsl".into()
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
    mut materials: ResMut<Assets<CloudMaterial>>,
) {
    commands.spawn((
        Mesh2d(meshes.add(Rectangle::new(WINDOW_SIZE, WINDOW_SIZE))),
        MeshMaterial2d(materials.add(CloudMaterial {
            padded_offset: PaddedOffset {
                offset: 0.0,
                _padding: Vec3::splat(0.0),
            },
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Clouds,
    ));
}

fn animate_clouds(
    mut cloud_material: ResMut<Assets<CloudMaterial>>,
    cloud: Single<&MeshMaterial2d<CloudMaterial>, With<Clouds>>,
    time: Res<Time>,
    boids_parameters: Res<BoidsParameters>,
) {
    const CLOUD_SPEED: f32 = 1.0 / 50.0;

    if let Some(material) = cloud_material.get_mut(cloud.0.id()) {
        material.padded_offset.offset += boids_parameters.speed * time.delta_secs() * CLOUD_SPEED;
    }
}
