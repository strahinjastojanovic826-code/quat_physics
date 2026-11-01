pub mod quaternary;
pub mod vector;
pub mod physics;
pub mod collision;

pub use quaternary::{QuatState, PackedQuads};
pub use vector::Vector3;
pub use physics::{Particle, PhysicsWorld};
pub use collision::CollisionSystem;

#[cfg(test)]
mod tests {
    use super::*;

    // ==========================================
    // 1. NAJLAKŠI TEST: Kvartarna logika (osnove)
    // ==========================================
    #[test]
    fn test_01_quaternary_basic() {
        let state_zero = QuatState::from(0);
        let state_three = QuatState::from(3);
        assert_eq!(state_zero, QuatState::Zero);
        assert_eq!(state_three, QuatState::Three);
    }

    // ==========================================
    // 2. PAKOVANJE: Niz kvartarnih stanja
    // ==========================================
    #[test]
    fn test_02_packed_quads_array() {
        let states = [QuatState::Zero, QuatState::One, QuatState::Two, QuatState::Three];
        let packed = PackedQuads::from_array(states);
        assert_eq!(packed.to_array(), states);
        assert_eq!(packed.get(0), QuatState::Zero);
        assert_eq!(packed.get(3), QuatState::Three);
    }

    // ==========================================
    // 3. VEKTORI: Kreiranje i magnituda
    // ==========================================
    #[test]
    fn test_03_vector_magnitude() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        assert_eq!(v.magnitude(), 5.0);
        assert_eq!(v.magnitude_squared(), 25.0);
    }

    // ==========================================
    // 4. VEKTORI: Operatori (sabiranje i skaliranje)
    // ==========================================
    #[test]
    fn test_04_vector_operators() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        let sum = v1 + v2;
        assert_eq!(sum, Vector3::new(5.0, 7.0, 9.0));

        let scaled = v1 * 2.0;
        assert_eq!(scaled, Vector3::new(2.0, 4.0, 6.0));
    }

    // ==========================================
    // 5. VEKTORI: Skalarni i vektorski proizvod
    // ==========================================
    #[test]
    fn test_05_vector_products() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let dot = v1.dot(&v2);
        assert_eq!(dot, 0.0);

        let cross = v1.cross(&v2);
        assert_eq!(cross, Vector3::new(0.0, 0.0, 1.0));
    }

    // ==========================================
    // 6. FIZIKA: Integracija sile na česticu
    // ==========================================
    #[test]
    fn test_06_particle_integration() {
        let mut p = Particle::new(Vector3::zero(), Vector3::new(1.0, 0.0, 0.0), 1.0, 0.5);
        p.apply_force(Vector3::new(0.0, 10.0, 0.0));
        p.integrate(1.0);
        
        assert_eq!(p.velocity, Vector3::new(1.0, 10.0, 0.0));
        assert_eq!(p.position, Vector3::new(1.0, 10.0, 0.0));
    }

    // ==========================================
    // 7. FIZIKA: Beskonačna masa (nepomični objekat)
    // ==========================================
    #[test]
    fn test_07_immovable_particle_mass_zero() {
        let mut p = Particle::new(Vector3::new(5.0, 5.0, 5.0), Vector3::new(2.0, 2.0, 2.0), 0.0, 1.0);
        assert_eq!(p.inverse_mass, 0.0);
        
        p.apply_force(Vector3::new(100.0, 100.0, 100.0));
        p.integrate(1.0);
        
        // Pozicija i brzina se ne smeju menjati jer je masa 0 (nepomični zid/objekat)
        assert_eq!(p.position, Vector3::new(5.0, 5.0, 5.0));
        assert_eq!(p.velocity, Vector3::new(2.0, 2.0, 2.0));
    }

    // ==========================================
    // 8. SVET FIZIKE: Globalna gravitacija u koraku
    // ==========================================
    #[test]
    fn test_08_physics_world_gravity() {
        let mut world = PhysicsWorld::new(
            Vector3::new(0.0, -9.81, 0.0),
            Vector3::new(-10.0, -10.0, -10.0),
            Vector3::new(10.0, 10.0, 10.0),
        );
        let p = Particle::new(Vector3::new(0.0, 5.0, 0.0), Vector3::zero(), 1.0, 0.5);
        world.add_particle(p);

        world.step(1.0);
        // Nakon 1 sekunde pod uticajem gravitacije, Y pozicija mora pasti
        assert!(world.particles[0].position.y < 5.0);
    }

    // ==========================================
    // 9. SUDARI: Granice sveta + Kvartarna restitucija
    // ==========================================
    #[test]
    fn test_09_quaternary_boundary_collision() {
        let mut world = PhysicsWorld::new(
            Vector3::zero(),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 10.0, 10.0),
        );
        
        // Kreiramo česticu koja udara u levu granicu (x = 0)
        let mut p = Particle::new(Vector3::new(0.4, 5.0, 5.0), Vector3::new(-5.0, 0.0, 0.0), 1.0, 0.5);
        // Postavljamo kvartarno stanje 0 na QuatState::Three (restitucija = 0.0 / potpuno upijanje udara)
        p.compressed_states.set(0, QuatState::Three);
        world.add_particle(p);

        world.step(0.1);
        
        // Pozicija mora biti korigovana na granicu (min + radius = 0.5), a brzina poništena na 0
        assert!(world.particles[0].position.x >= 0.5);
        assert_eq!(world.particles[0].velocity.x, 0.0);
    }

    // ==========================================
    // 10. NAJSTRESNIJI TEST: Gust klaster čestica
    // (Višestruki istovremeni sudari sfera pod gravitacijom,
    // različita 2-bitna kvartarna stanja materijala, 50 iteracija)
    // ==========================================
    #[test]
    fn test_10_high_stress_multi_particle_cluster() {
        let mut world = PhysicsWorld::new(
            Vector3::new(0.0, -5.0, 0.0),
            Vector3::new(-5.0, -5.0, -5.0),
            Vector3::new(5.0, 5.0, 5.0),
        );

        // Ubacujemo 10 zbijenih čestica u mali prostor da izazovemo ekstremne preklope i stres sudara
        for i in 0..10 {
            let offset = (i as f32) * 0.05;
            let mut p = Particle::new(
                Vector3::new(offset, 0.0, 0.0),
                Vector3::new((i as f32 - 5.0) * 3.0, (i as f32 % 3.0) * 4.0, 0.0),
                1.0,
                0.6, // Veći radijus da garantuje konstantne sudare
            );
            
            // Mešamo kvartarna stanja da svaka čestica ima drugačiju elastičnost (restituciju)
            let q_state = match i % 4 {
                0 => QuatState::Zero,
                1 => QuatState::One,
                2 => QuatState::Two,
                _ => QuatState::Three,
            };
            p.compressed_states.set(0, q_state);
            world.add_particle(p);
        }

        // Pokrećemo simulaciju kroz 50 teških koraka
        for _ in 0..50 {
            world.step(0.016);
        }

        // Provera stabilnosti: nijedna vrednost ne sme pući u NaN i čestice moraju ostati unutar granica
        for p in &world.particles {
            assert!(!p.position.x.is_nan(), "Particle X position became NaN under stress!");
            assert!(!p.position.y.is_nan(), "Particle Y position became NaN under stress!");
            assert!(!p.position.z.is_nan(), "Particle Z position became NaN under stress!");
            
            assert!(p.position.x >= -5.0 && p.position.x <= 5.0);
            assert!(p.position.y >= -5.0 && p.position.y <= 5.0);
            assert!(p.position.z >= -5.0 && p.position.z <= 5.0);
        }
    }
}