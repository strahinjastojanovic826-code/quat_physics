use crate::vector::Vector3;
use crate::quaternary::PackedQuads;

/// Represents an advanced physical particle with forces, radius, and compressed quaternary properties
#[derive(Debug, Clone, Copy)]
pub struct Particle {
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
    pub mass: f32,
    pub inverse_mass: f32,
    pub radius: f32,
    /// Packed quaternary flags (e.g. Index 0 controls material restitution/bounciness)
    pub compressed_states: PackedQuads,
}

impl Particle {
    pub fn new(position: Vector3, velocity: Vector3, mass: f32, radius: f32) -> Self {
        let inverse_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        Self {
            position,
            velocity,
            acceleration: Vector3::zero(),
            mass,
            inverse_mass,
            radius,
            compressed_states: PackedQuads::new(),
        }
    }

    pub fn apply_force(&mut self, force: Vector3) {
        if self.inverse_mass > 0.0 {
            self.acceleration += force * self.inverse_mass;
        }
    }

    pub fn integrate(&mut self, dt: f32) {
        if self.inverse_mass == 0.0 {
            return;
        }
        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;
        self.acceleration = Vector3::zero();
    }
}

/// A container managing particles, global gravity, and the simulation loop
pub struct PhysicsWorld {
    pub particles: Vec<Particle>,
    pub gravity: Vector3,
    pub bounds_min: Vector3,
    pub bounds_max: Vector3,
}

impl PhysicsWorld {
    pub fn new(gravity: Vector3, bounds_min: Vector3, bounds_max: Vector3) -> Self {
        Self {
            particles: Vec::new(),
            gravity,
            bounds_min,
            bounds_max,
        }
    }

    pub fn add_particle(&mut self, particle: Particle) {
        self.particles.push(particle);
    }

    /// Advances the global simulation step, applies forces, integration, and collision responses
    pub fn step(&mut self, dt: f32) {
        let g = self.gravity;
        
        // 1. Apply forces and integrate movement
        for particle in &mut self.particles {
            let gravitational_force = g * particle.mass;
            particle.apply_force(gravitational_force);
            particle.integrate(dt);
        }

        // 2. Resolve boundary and particle-to-particle collisions using advanced collision module
        crate::collision::CollisionSystem::resolve_collisions(
            &mut self.particles,
            self.bounds_min,
            self.bounds_max,
        );
    }
}

pub struct TimeStepper {
    accumulator: f32,
    fixed_dt: f32,
}

impl TimeStepper {
    pub fn new(fixed_dt: f32) -> Self {
        Self {
            accumulator: 0.0,
            fixed_dt,
        }
    }

    /// Advances the simulation loop with a fixed timestep accumulator pattern
    pub fn step<F1, F2>(&mut self, frame_dt: f32, mut update_physics: F1, mut interpolate: F2)
    where
        F1: FnMut(f32), // Called at fixed_dt intervals
        F2: FnMut(f32), // Called for rendering interpolation (alpha factor 0.0 to 1.0)
    {
        // Prevent spiral of death if frame rate drops significantly
        let clamped_dt = if frame_dt > 0.25 { 0.25 } else { frame_dt };
        self.accumulator += clamped_dt;

        while self.accumulator >= self.fixed_dt {
            update_physics(self.fixed_dt);
            self.accumulator -= self.fixed_dt;
        }

        // Alpha interpolation factor for smooth rendering between physics ticks
        let alpha = self.accumulator / self.fixed_dt;
        interpolate(alpha);
    }
}