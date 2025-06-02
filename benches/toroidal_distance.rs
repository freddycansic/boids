use boids::{TOROIDAL_SIZE, squared_toroidal::SquaredToroidal};
use criterion::{Criterion, black_box, criterion_group, criterion_main};
use kiddo::traits::DistanceMetric;

fn bench_toroidal_dist(c: &mut Criterion) {
    let a = black_box([10.0, 10.0]);
    let b = black_box([TOROIDAL_SIZE - 1.0, TOROIDAL_SIZE - 1.0]);

    c.bench_function("SquaredToroidal::dist", |bencher| {
        bencher.iter(|| SquaredToroidal::dist(&a, &b))
    });
}

criterion_group!(benches, bench_toroidal_dist);
criterion_main!(benches);
