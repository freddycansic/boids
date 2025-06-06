// https://www.shadertoy.com/view/NssSWN
// Converted from Darko using https://eliotbo.github.io/glsl2wgsl/
// Modified for use with Bevy

#import bevy_sprite::mesh2d_vertex_output::VertexOutput

@group(2) @binding(0) var<uniform> padded_offset: PaddedOffset;

struct PaddedOffset {
    offset: f32,
    _padding: vec3<f32>
}

fn noise(uv: vec2<f32>) -> f32 {
	return fract(sin(uv.x * 113. + uv.y * 412.) * 6339.);
}

fn noiseSmooth(uv: vec2<f32>) -> vec3<f32> {
	let index: vec2<f32> = floor(uv);
	var pq: vec2<f32> = fract(uv);
	pq = smoothstep(vec2<f32>(0.), vec2<f32>(1.), pq);
	let topLeft: f32 = noise(index);
	let topRight: f32 = noise(index + vec2<f32>(1., 0.));
	let top: f32 = mix(topLeft, topRight, pq.x);
	let bottomLeft: f32 = noise(index + vec2<f32>(0., 1.));
	let bottomRight: f32 = noise(index + vec2<f32>(1., 1.));
	let bottom: f32 = mix(bottomLeft, bottomRight, pq.x);
	return vec3<f32>(mix(top, bottom, pq.y));
}

@fragment
fn fragment(mesh: VertexOutput) -> @location(0) vec4<f32> {
	var uv: vec2<f32> = mesh.uv;
    let offset: f32 = padded_offset.offset;

    uv.x = uv.x + (offset / 200.0);

    var col: vec3<f32> = noiseSmooth(uv * 4.0);
    col += noiseSmooth(uv * 8.0) * 0.5;
    col += noiseSmooth(uv * 16.0) * 0.25;
    col /= 4.;

    col *= smoothstep(vec3<f32>(0.2), vec3<f32>(0.6), col);

    let cloud_intensity = max(max(col.x, col.y), col.z);

    return vec4<f32>(1.0, 1.0, 1.0, cloud_intensity);
}
