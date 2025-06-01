use kiddo::traits::DistanceMetric;

use crate::TOROIDAL_SIZE;

pub struct SquaredToroidal;

fn wrap_delta(delta: f32, size: f32) -> f32 {
    let half = size * 0.5;
    // If delta is outside of -half to half range then bring back into range
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
        wrap_delta(a - b, TOROIDAL_SIZE)
    }
}
