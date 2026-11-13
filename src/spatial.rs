use std::collections::HashMap;
use crate::vector::Vector3;

pub struct SpatialHashGrid {
    cell_size: f32,
    cells: HashMap<(i32, i32, i32), Vec<usize>>,
}

impl SpatialHashGrid {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            cells: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.cells.clear();
    }

    #[inline]
    fn hash(&self, pos: &Vector3) -> (i32, i32, i32) {
        (
            (pos.x / self.cell_size).floor() as i32,
            (pos.y / self.cell_size).floor() as i32,
            (pos.z / self.cell_size).floor() as i32,
        )
    }

    pub fn insert(&mut self, entity_id: usize, pos: &Vector3) {
        let cell_key = self.hash(pos);
        self.cells.entry(cell_key).or_default().push(entity_id);
    }

    // Returns candidate indices from the same and neighboring cells
    pub fn get_neighbors(&self, pos: &Vector3) -> Vec<usize> {
        let center = self.hash(pos);
        let mut neighbors = Vec::new();

        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    let key = (center.0 + x, center.1 + y, center.2 + z);
                    if let Some(bucket) = self.cells.get(&key) {
                        neighbors.extend(bucket);
                    }
                }
            }
        }
        neighbors
    }
}