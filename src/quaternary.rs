/// Represents a quaternary logic state (2 bits: 0 to 3)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuatState {
    Zero = 0,   // 00
    One = 1,    // 01
    Two = 2,    // 10
    Three = 3,  // 11
}

impl From<u8> for QuatState {
    fn from(val: u8) -> Self {
        match val & 0b11 {
            0 => QuatState::Zero,
            1 => QuatState::One,
            2 => QuatState::Two,
            _ => QuatState::Three,
        }
    }
}

/// Packs 4 quaternary values (2 bits each) into a single u8 (75% memory saving).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PackedQuads {
    data: u8,
}

impl PackedQuads {
    pub fn new() -> Self {
        Self { data: 0 }
    }

    pub fn from_array(states: [QuatState; 4]) -> Self {
        let mut packed = Self::new();
        for (i, &state) in states.iter().enumerate() {
            packed.set(i, state);
        }
        packed
    }

    pub fn set(&mut self, index: usize, state: QuatState) {
        assert!(index < 4, "Index out of bounds for PackedQuads (max 3)");
        let shift = index * 2;
        self.data &= !(0b11 << shift);
        self.data |= (state as u8 & 0b11) << shift;
    }

    pub fn get(&self, index: usize) -> QuatState {
        assert!(index < 4, "Index out of bounds for PackedQuads (max 3)");
        let shift = index * 2;
        let val = (self.data >> shift) & 0b11;
        QuatState::from(val)
    }

    pub fn raw(&self) -> u8 {
        self.data
    }

    pub fn to_array(&self) -> [QuatState; 4] {
        [
            self.get(0),
            self.get(1),
            self.get(2),
            self.get(3),
        ]
    }
}