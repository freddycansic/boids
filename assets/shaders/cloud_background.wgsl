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

	let sky_colour = vec3<f32>(80./255., 175./255., 228./255.);
	let sky: vec3<f32> = sky_colour * (1. - uv.y + 1.5) / 2.;

	uv.x = uv.x + (offset / 40.);
	var uv2: vec2<f32> = uv;
	uv2.x = uv2.x + (offset / 10.);
	var uv3: vec2<f32> = uv;
	uv3.x = uv3.x + (offset / 30.);
	var col: vec3<f32> = noiseSmooth(uv * 4.);
	col = col + (noiseSmooth(uv * 8.) * 0.5);
	col = col + (noiseSmooth(uv2 * 16.) * 0.25);
	col = col + (noiseSmooth(uv3 * 32.) * 0.125);
	col = col + (noiseSmooth(uv3 * 64.) * 0.0625);
	col = col / (2.);
	col *= smoothstep(vec3<f32>(0.2), vec3<f32>(0.6), col);
	col = mix(vec3<f32>(1. - col / 7.), sky, vec3<f32>(1. - col));

	return vec4<f32>(vec3<f32>(col), 1.);
}
