use crate::vector::Vector3;
use crate::physics::Particle;
use crate::quaternary::QuatState;

pub struct CollisionSystem;

impl CollisionSystem {
    /// Resolves sphere-to-sphere and boundary collisions, factoring in quaternary material properties
    pub fn resolve_collisions(particles: &mut [Particle], bounds_min: Vector3, bounds_max: Vector3) {
        let len = particles.len();

        // 1. Boundary Collisions
        for i in 0..len {
            let radius = particles[i].radius;
            let p = &mut particles[i];
            let restitution = Self::get_restitution(p);

            // X boundaries
            if p.position.x - radius < bounds_min.x {
                p.position.x = bounds_min.x + radius;
                p.velocity.x = -p.velocity.x * restitution;
            } else if p.position.x + radius > bounds_max.x {
                p.position.x = bounds_max.x - radius;
                p.velocity.x = -p.velocity.x * restitution;
            }

            // Y boundaries
            if p.position.y - radius < bounds_min.y {
                p.position.y = bounds_min.y + radius;
                p.velocity.y = -p.velocity.y * restitution;
            } else if p.position.y + radius > bounds_max.y {
                p.position.y = bounds_max.y - radius;
                p.velocity.y = -p.velocity.y * restitution;
            }

            // Z boundaries
            if p.position.z - radius < bounds_min.z {
                p.position.z = bounds_min.z + radius;
                p.velocity.z = -p.velocity.z * restitution;
            } else if p.position.z + radius > bounds_max.z {
                p.position.z = bounds_max.z - radius;
                p.velocity.z = -p.velocity.z * restitution;
            }
        }

        // 2. Particle-to-Particle Collisions (Sphere vs Sphere)
        for i in 0..len {
            for j in (i + 1)..len {
                let (p1, p2) = if i < j {
                    let (left, right) = particles.split_at_mut(j);
                    (&mut left[i], &mut right[0])
                } else {
                    continue;
                };

                let delta = p2.position - p1.position;
                let dist_sq = delta.magnitude_squared();
                let min_dist = p1.radius + p2.radius;

                if dist_sq < min_dist * min_dist && dist_sq > 0.0 {
                    let dist = dist_sq.sqrt();
                    let normal = delta / dist;
                    let overlap = min_dist - dist;

                    let total_inv_mass = p1.inverse_mass + p2.inverse_mass;
                    if total_inv_mass > 0.0 {
                        // Positional correction to prevent sinking
                        p1.position -= normal * (overlap * (p1.inverse_mass / total_inv_mass));
                        p2.position += normal * (overlap * (p2.inverse_mass / total_inv_mass));

                        // Calculate combined material restitution from quaternary states
                        let r1 = Self::get_restitution(p1);
                        let r2 = Self::get_restitution(p2);
                        let restitution = (r1 + r2) * 0.5;

                        // Impulse response
                        let relative_velocity = p2.velocity - p1.velocity;
                        let velocity_along_normal = relative_velocity.dot(&normal);

                        if velocity_along_normal < 0.0 {
                            let impulse_scalar = -(1.0 + restitution) * velocity_along_normal / total_inv_mass;
                            let impulse = normal * impulse_scalar;

                            p1.velocity -= impulse * p1.inverse_mass;
                            p2.velocity += impulse * p2.inverse_mass;
                        }
                    }
                }
            }
        }
    }

    /// Maps the 2-bit quaternary state (Index 0) to physical material bounciness (restitution):
    /// - Zero (00): Normal bounciness (0.5)
    /// - One (01): Super bouncy / elastic (0.9)
    /// - Two (10): Damped / sticky (0.1)
    /// - Three (11): Perfectly inelastic / dead stop (0.0)
    fn get_restitution(particle: &Particle) -> f32 {
        match particle.compressed_states.get(0) {
            QuatState::Zero => 0.5,
            QuatState::One => 0.9,
            QuatState::Two => 0.1,
            QuatState::Three => 0.0,
        }
    }
}