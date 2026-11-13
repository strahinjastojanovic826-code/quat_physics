pub mod quaternary;
pub mod vector;
pub mod physics;
pub mod collision;
pub mod ecs;
pub mod spatial;

pub use quaternary::{QuatState, PackedQuads};
pub use vector::Vector3;
pub use physics::{Particle, PhysicsWorld};
pub use collision::CollisionSystem;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::QuatMaterial;

    // ========================================================
    // 1. TEST & BENCHMARK: Quaternary Logic Basics
    // ========================================================
    #[test]
    fn test_01_quaternary_basic() {
        let start = std::time::Instant::now();
        
        let state_zero = QuatState::from(0);
        let state_three = QuatState::from(3);
        assert_eq!(state_zero, QuatState::Zero);
        assert_eq!(state_three, QuatState::Three);

        let duration = start.elapsed();
        println!("[BENCHMARK] test_01_quaternary_basic took: {:?}", duration);
    }

    // ========================================================
    // 2. TEST & BENCHMARK: Packed Quads Array Conversion
    // ========================================================
    #[test]
    fn test_02_packed_quads_array() {
        let start = std::time::Instant::now();

        let states = [QuatState::Zero, QuatState::One, QuatState::Two, QuatState::Three];
        let packed = PackedQuads::from_array(states);
        assert_eq!(packed.to_array(), states);
        assert_eq!(packed.get(0), QuatState::Zero);
        assert_eq!(packed.get(3), QuatState::Three);

        let duration = start.elapsed();
        println!("[BENCHMARK] test_02_packed_quads_array took: {:?}", duration);
    }

    // ========================================================
    // 3. TEST & BENCHMARK: Vector3 Creation & Magnitude
    // ========================================================
    #[test]
    fn test_03_vector_magnitude() {
        let start = std::time::Instant::now();

        let v = Vector3::new(3.0, 4.0, 0.0);
        assert_eq!(v.magnitude(), 5.0);
        assert_eq!(v.magnitude_squared(), 25.0);

        let duration = start.elapsed();
        println!("[BENCHMARK] test_03_vector_magnitude took: {:?}", duration);
    }

    // ========================================================
    // 4. TEST & BENCHMARK: Vector3 Operators & Scaling
    // ========================================================
    #[test]
    fn test_04_vector_operators() {
        let start = std::time::Instant::now();

        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        let sum = v1 + v2;
        assert_eq!(sum, Vector3::new(5.0, 7.0, 9.0));

        let scaled = v1 * 2.0;
        assert_eq!(scaled, Vector3::new(2.0, 4.0, 6.0));

        let duration = start.elapsed();
        println!("[BENCHMARK] test_04_vector_operators took: {:?}", duration);
    }

    // ========================================================
    // 5. TEST & BENCHMARK: Vector3 Dot and Cross Products
    // ========================================================
    #[test]
    fn test_05_vector_products() {
        let start = std::time::Instant::now();

        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let dot = v1.dot(&v2);
        assert_eq!(dot, 0.0);

        let cross = v1.cross(&v2);
        assert_eq!(cross, Vector3::new(0.0, 0.0, 1.0));

        let duration = start.elapsed();
        println!("[BENCHMARK] test_05_vector_products took: {:?}", duration);
    }

    // ========================================================
    // 6. TEST & BENCHMARK: Particle Integration (Force & Acceleration)
    // ========================================================
    #[test]
    fn test_06_particle_integration() {
        let start = std::time::Instant::now();

        let mut p = Particle::new(Vector3::zero(), Vector3::new(1.0, 0.0, 0.0), 1.0, 0.5);
        p.apply_force(Vector3::new(0.0, 10.0, 0.0));
        p.integrate(1.0);

        assert_eq!(p.velocity, Vector3::new(1.0, 10.0, 0.0));
        assert_eq!(p.position, Vector3::new(1.0, 10.0, 0.0));

        let duration = start.elapsed();
        println!("[BENCHMARK] test_06_particle_integration took: {:?}", duration);
    }

    // ========================================================
    // 7. TEST & BENCHMARK: Immovable Particle (Mass Zero / Static Object)
    // ========================================================
    #[test]
    fn test_07_immovable_particle_mass_zero() {
        let start = std::time::Instant::now();

        let mut p = Particle::new(Vector3::new(5.0, 5.0, 5.0), Vector3::new(2.0, 2.0, 2.0), 0.0, 1.0);
        assert_eq!(p.inverse_mass, 0.0);

        p.apply_force(Vector3::new(100.0, 100.0, 100.0));
        p.integrate(1.0);

        // Position and velocity must not change because mass is 0 (immovable wall/object)
        assert_eq!(p.position, Vector3::new(5.0, 5.0, 5.0));
        assert_eq!(p.velocity, Vector3::new(2.0, 2.0, 2.0));

        let duration = start.elapsed();
        println!("[BENCHMARK] test_07_immovable_particle_mass_zero took: {:?}", duration);
    }

    // ========================================================
    // 8. TEST & BENCHMARK: Physics World Global Gravity
    // ========================================================
    #[test]
    fn test_08_physics_world_gravity() {
        let start = std::time::Instant::now();

        let mut world = PhysicsWorld::new(
            Vector3::new(0.0, -9.81, 0.0),
            Vector3::new(-10.0, -10.0, -10.0),
            Vector3::new(10.0, 10.0, 10.0),
        );
        let p = Particle::new(Vector3::new(0.0, 5.0, 0.0), Vector3::zero(), 1.0, 0.5);
        world.add_particle(p);

        world.step(1.0);
        // After 1 second under gravity impact, Y position must fall
        assert!(world.particles[0].position.y < 5.0);

        let duration = start.elapsed();
        println!("[BENCHMARK] test_08_physics_world_gravity took: {:?}", duration);
    }

    // ========================================================
    // 9. TEST & BENCHMARK: World Boundaries + Quaternary Restitution
    // ========================================================
    #[test]
    fn test_09_quaternary_boundary_collision() {
        let start = std::time::Instant::now();

        let mut world = PhysicsWorld::new(
            Vector3::zero(),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 10.0, 10.0),
        );

        // We create a particle hitting the left boundary (x = 0)
        let mut p = Particle::new(Vector3::new(0.4, 5.0, 5.0), Vector3::new(-5.0, 0.0, 0.0), 1.0, 0.5);
        // Set quaternary state 0 to Three (restitution = 0.0 / complete impact absorption / dead stop)
        p.compressed_states.set(0, QuatState::Three);
        world.add_particle(p);

        world.step(0.1);

        // Position must be corrected to boundary (min + radius = 0.5), and velocity set to 0
        assert!(world.particles[0].position.x >= 0.5);
        assert_eq!(world.particles[0].velocity.x, 0.0);

        let duration = start.elapsed();
        println!("[BENCHMARK] test_09_quaternary_boundary_collision took: {:?}", duration);
    }

    // ========================================================
    // 10. TEST & BENCHMARK: High-Stress Multi-Particle Cluster
    // ========================================================
    #[test]
    fn test_10_high_stress_multi_particle_cluster() {
        let start = std::time::Instant::now();

        let mut world = PhysicsWorld::new(
            Vector3::new(0.0, -5.0, 0.0),
            Vector3::new(-5.0, -5.0, -5.0),
            Vector3::new(5.0, 5.0, 5.0),
        );

        // Insert 10 packed particles in a small space to provoke extreme overlapping and collision stress
        for i in 0..10 {
            let offset = (i as f32) * 0.05;
            let mut p = Particle::new(
                Vector3::new(offset, 0.0, 0.0),
                Vector3::new((i as f32 - 5.0) * 3.0, (i as f32 % 3.0) * 4.0, 0.0),
                1.0,
                0.6, // Larger radius to guarantee constant collisions
            );

            // Mix quaternary states so every particle has a different elasticity (restitution)
            let q_state = match i % 4 {
                0 => QuatState::Zero,
                1 => QuatState::One,
                2 => QuatState::Two,
                _ => QuatState::Three,
            };
            p.compressed_states.set(0, q_state);
            world.add_particle(p);
        }

        // Run simulation through 50 heavy steps
        for _ in 0..50 {
            world.step(0.016);
        }

        // Stability check: no value must break to NaN and particles must stay within boundaries
        for p in &world.particles {
            assert!(!p.position.x.is_nan(), "Particle X position became NaN under stress!");
            assert!(!p.position.y.is_nan(), "Particle Y position became NaN under stress!");
            assert!(!p.position.z.is_nan(), "Particle Z position became NaN under stress!");

            assert!(p.position.x >= -5.0 && p.position.x <= 5.0);
            assert!(p.position.y >= -5.0 && p.position.y <= 5.0);
            assert!(p.position.z >= -5.0 && p.position.z <= 5.0);
        }

        let duration = start.elapsed();
        println!("[BENCHMARK] test_10_high_stress_multi_particle_cluster (50 steps, 10 particles) took: {:?}", duration);
    }
    
    // ========================================================
    // 11. TEST & BENCHMARK: Quaternary Logic Operators (AND, OR, XOR, Mul)
    // ========================================================
    #[test]
    fn test_11_quaternary_operators_and_benchmark() {
        let s1 = QuatState::One;   // 01
        let s2 = QuatState::Two;   // 10

        // Logic checks
        assert_eq!(s1.q_and(s2), QuatState::One); // Min(1, 2) -> 1
        assert_eq!(s1.q_or(s2), QuatState::Two);  // Max(1, 2) -> 2
        assert_eq!(s1.q_xor(s2), QuatState::Three); // 01 ^ 10 -> 11 (Three)

        let (val, carry) = QuatState::Three.q_mul(QuatState::Two); // 3 * 2 = 6 -> carry 1 (val 2)
        assert_eq!(val, QuatState::Two);
        assert_eq!(carry, QuatState::One);

        // Inline Benchmark for Quaternary Logic Operations
        let start = std::time::Instant::now();
        let mut dummy = QuatState::Zero;
        for i in 0..100_000 {
            let state_a = QuatState::from((i % 4) as u8);
            let state_b = QuatState::from(((i + 1) % 4) as u8);
            dummy = dummy.q_xor(state_a.q_and(state_b));
        }
        let duration = start.elapsed();
        println!("[BENCHMARK] 100,000 Quaternary Logic Operations took: {:?}. Dummy result: {:?}", duration, dummy);
    }

    // ========================================================
    // 12. TEST & BENCHMARK: SIMD-within-a-register (SWAR) Batched Quads
    // ========================================================
    #[test]
    fn test_12_simd_packed_quads_and_benchmark() {
        let initial_states = [QuatState::Zero, QuatState::One, QuatState::Two, QuatState::Three];
        let packed = PackedQuads::from_array(initial_states);

        // Test SIMD NOT transformation across all 4 packed states concurrently
        let inverted = packed.simd_not();
        assert_eq!(inverted.get(0), QuatState::Three);
        assert_eq!(inverted.get(1), QuatState::Two);
        assert_eq!(inverted.get(2), QuatState::One);
        assert_eq!(inverted.get(3), QuatState::Zero);

        // Inline Benchmark for SIMD / SWAR Batched Operations
        let start = std::time::Instant::now();
        let mut p = packed;
        for _ in 0..100_000 {
            p = p.simd_not();
        }
        let duration = start.elapsed();
        println!("[BENCHMARK] 100,000 SIMD Packed Quads Iterations took: {:?}. Final data: {}", duration, p.raw());
    }

    // ========================================================
    // 13. TEST & BENCHMARK: Quaternary Materials Engine
    // ========================================================
    #[test]
    fn test_13_quat_materials_and_benchmark() {
        // Test mapping
        let mat_zero = QuatMaterial::from_qustate(QuatState::Zero);
        let mat_three = QuatMaterial::from_qustate(QuatState::Three);
        assert_eq!(mat_zero.restitution, 0.5);
        assert_eq!(mat_three.restitution, 0.0); // Dead stop / plastic

        let combined = QuatMaterial::combine(&mat_zero, &mat_three);
        assert_eq!(combined.restitution, 0.25);

        // Inline Benchmark for Materials Mapping and Combination
        let start = std::time::Instant::now();
        let mut total_restitution = 0.0;
        for i in 0..100_000 {
            let st = QuatState::from((i % 4) as u8);
            let m = QuatMaterial::from_qustate(st);
            total_restitution += m.restitution;
        }
        let duration = start.elapsed();
        println!("[BENCHMARK] 100,000 Material Lookups & Combinations took: {:?}. Checksum: {}", duration, total_restitution);
    }
}