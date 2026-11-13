use crate::vector::Vector3;
use crate::quaternary::PackedQuads;

pub type EntityId = usize;

pub struct World {
    pub positions: Vec<Vector3>,
    pub velocities: Vec<Vector3>,
    pub accelerations: Vec<Vector3>,
    pub masses: Vec<f32>,
    pub inverse_masses: Vec<f32>,
    pub radii: Vec<f32>,
    pub materials: Vec<PackedQuads>,
    entities_count: usize,
}

impl World {
    pub fn new() -> Self {
        Self {
            positions: Vec::new(),
            velocities: Vec::new(),
            accelerations: Vec::new(),
            masses: Vec::new(),
            inverse_masses: Vec::new(),
            radii: Vec::new(),
            materials: Vec::new(),
            entities_count: 0,
        }
    }

    pub fn spawn_particle(&mut self, pos: Vector3, vel: Vector3, mass: f32, radius: f32, material: PackedQuads) -> EntityId {
        let id = self.entities_count;
        self.positions.push(pos);
        self.velocities.push(vel);
        self.accelerations.push(Vector3::zero());
        self.masses.push(mass);
        
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        self.inverse_masses.push(inv_mass);
        
        self.radii.push(radius);
        self.materials.push(material);
        
        self.entities_count += 1;
        id
    }

    pub fn len(&self) -> usize {
        self.entities_count
    }
}