/// Represents a quaternary logic state (2 bits: 0 to 3)[cite: 9]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuatState {
    Zero = 0,  // 00[cite: 9]
    One = 1,   // 01[cite: 9]
    Two = 2,   // 10[cite: 9]
    Three = 3, // 11[cite: 9]
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

impl QuatState {
    /// Softverska kvartarna negacija (NOT u bazi 4)
    pub fn not(self) -> Self {
        match self {
            QuatState::Zero => QuatState::Three,
            QuatState::One => QuatState::Two,
            QuatState::Two => QuatState::One,
            QuatState::Three => QuatState::Zero,
        }
    }

    /// Quaternary AND (Min operation in many-valued logic)
    pub fn q_and(self, other: Self) -> Self {
        let val = std::cmp::min(self as u8, other as u8);
        Self::from(val)
    }

    /// Quaternary OR (Max operation in many-valued logic)
    pub fn q_or(self, other: Self) -> Self {
        let val = std::cmp::max(self as u8, other as u8);
        Self::from(val)
    }

    /// Quaternary XOR (Modulo 4 addition / difference)
    pub fn q_xor(self, other: Self) -> Self {
        let val = ((self as u8) ^ (other as u8)) & 0b11;
        Self::from(val)
    }

    /// Quaternary multiplication with carry (returns value, carry)
    pub fn q_mul(self, other: Self) -> (Self, Self) {
        let prod = (self as u8) * (other as u8);
        let carry = prod / 4;
        let value = prod % 4;
        (Self::from(value), Self::from(carry))
    }

    /// Kvartarno sabiranje dve vrednosti (vraća rezultat i prenos - carry)
    pub fn add(self, other: Self) -> (Self, Self) {
        let sum = (self as u8) + (other as u8);
        let carry = sum / 4;
        let value = sum % 4;
        (Self::from(value), Self::from(carry))
    }
}

/// Packs 4 quaternary values (2 bits each) into a single u8 (75% memory saving)[cite: 9, 10].
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

    /// SIMD-within-a-register (SWAR): Applies a batch transformation to all 4 quaternary states 
    /// packed inside the single u8 simultaneously without unpacking to arrays first.
    pub fn simd_map<F>(&self, mut operation: F) -> Self
    where
        F: FnMut(QuatState) -> QuatState,
    {
        let mut res_data = 0u8;
        // Process all 4 packed 2-bit values concurrently at the register byte level
        for i in 0..4 {
            let shift = i * 2;
            let raw_state = (self.data >> shift) & 0b11;
            let state = QuatState::from(raw_state);
            let transformed = operation(state);
            res_data |= (transformed as u8) << shift;
        }
        Self { data: res_data }
    }

    /// Parallel batch bitwise NOT across all packed quads
    pub fn simd_not(&self) -> Self {
        self.simd_map(|s| s.not())
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