# quat_physics

`quat_physics` is a lightweight, memory-efficient 3D physics library written in Rust. It uniquely combines custom vector mathematics and particle simulation with **quaternary logic (2-bit packing)**, achieving a **75% memory reduction** for particle states and material properties compared to traditional byte storage.

---

## 🚀 Key Features

* **Memory-Efficient Quaternary Logic (`PackedQuads`)**: Packs 4 distinct quaternary states (`00`, `01`, `10`, `11`) into a single `u8` byte.
* **Material-Driven Physics**: 2-bit packed states directly control runtime physical behaviors, such as particle restitution (bounciness, stickiness, or dead stops) during collisions.
* **Robust 3D Vector Math (`Vector3`)**: Complete operator overloading (`+`, `-`, `*`, `/`, `+=`, `-=`, etc.) along with dot product, cross product, normalization, and magnitude calculations.
* **Simulation World (`PhysicsWorld`)**: Semi-implicit Euler integration, Newton's second law force application ($F = ma$), global gravity, and strict world boundary enforcement.
* **Collision Resolution**: Accurate sphere-to-sphere and sphere-to-boundary collision detection and response.
* **Full Quaternary Operators**: Comprehensive many-valued logic operations including `AND`, `OR`, `XOR`, and multiplication with carry.
* **SIMD-Within-A-Register (SWAR) Optimization**: High-performance batch transformations processing 4 packed quaternary states concurrently inside a single `u8` register.
* **Advanced Quaternary Materials Engine**: Maps 2-bit states to multi-parameter physical properties (restitution, friction, drag, and plasticity).

---

## Benchmark

| Test Name | Category | Operations / Iterations | Execution Time | Status |
| :--- | :--- | :--- | :--- | :--- |
| `test_01_quaternary_basic` | Quaternary Logic | Basic state initialization | $1.6\ \mu\text{s}$ | **PASSED**[cite: 18] |
| `test_02_packed_quads_array` | Quaternary Logic | Array to packed conversion & indexing | $5.1\ \mu\text{s}$ | **PASSED**[cite: 18] |
| `test_03_vector_magnitude` | 3D Math / Vectors | Magnitude & squared magnitude | $500\ \text{ns}$ | **PASSED**[cite: 18] |
| `test_04_vector_operators` | 3D Math / Vectors | Addition & scalar multiplication | $200\ \text{ns}$ | **PASSED**[cite: 18] |
| `test_05_vector_products` | 3D Math / Vectors | Dot & Cross products | $100\ \text{ns}$ | **PASSED**[cite: 18] |
| `test_06_particle_integration` | Physics Core | Force application & integration | $500\ \text{ns}$ | **PASSED**[cite: 18] |
| `test_07_immovable_particle_mass_zero` | Physics Core | Zero-mass / static collision check | $400\ \text{ns}$ | **PASSED**[cite: 18] |
| `test_08_physics_world_gravity` | Physics World | Global gravity simulation step | $8.2\ \mu\text{s}$ | **PASSED**[cite: 18] |
| `test_09_quaternary_boundary_collision` | Collision System | Boundary collision & quaternary restitution | $1.5\ \mu\text{s}$ | **PASSED**[cite: 18] |
| `test_10_high_stress_multi_particle_cluster` | Collision & Stress | 10 particles, 50 heavy frames, overlap stress | $129.5\ \mu\text{s}$ | **PASSED**[cite: 18] |
| `test_11_quaternary_operators_and_benchmark` | Quaternary Logic | 100,000 AND/OR/XOR/Mul operations | $3.611\ \text{ms}$ | **PASSED**[cite: 18] |
| `test_12_simd_packed_quads_and_benchmark` | SIMD / SWAR | 100,000 batched register-level operations | $7.748\ \text{ms}$ | **PASSED**[cite: 18] |
| `test_13_quat_materials_and_benchmark` | Materials Engine | 100,000 material lookups & combinations | $1.201\ \text{ms}$ | **PASSED**[cite: 18] |

## Licensing

This project is dual-licensed:

* **Open Source (AGPLv3):** Free to use for open-source projects. If you use or modify this library in your software or network service, your source code must also be open-source under AGPLv3.
* **Commercial License:** Required if you want to use `quat_physics` in closed-source or proprietary commercial software without AGPLv3 restrictions.

For commercial licensing, contact: **strahinjastojanovic826@gmail.com**

## 📂 Project Structure

```text
quat_physics/
├── Cargo.toml
└── src/
    ├── lib.rs          # Main library exports and tests
    ├── quaternary.rs   # 2-bit quaternary logic and packing
    ├── vector.rs       # 3D vector math and operators
    ├── physics.rs      # Particle entity and PhysicsWorld container
    └── collision.rs    # Collision detection and material mapping
```

---

## 📦 Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
quat_physics = "0.4.0"
```

---

## 💡 Quick Start Example

Here is a quick example of how to set up a physics world, add a particle with custom quaternary material properties, and run a simulation step:

```rust
use quat_physics::{PhysicsWorld, Particle, Vector3, QuatState};

fn main() {
    // 1. Create a physics world with gravity and boundaries
    let gravity = Vector3::new(0.0, -9.81, 0.0);
    let bounds_min = Vector3::new(-10.0, -10.0, -10.0);
    let bounds_max = Vector3::new(10.0, 10.0, 10.0);
    
    let mut world = PhysicsWorld::new(gravity, bounds_min, bounds_max);

    // 2. Create a particle at position (0.0, 5.0, 0.0) with mass 1.0 and radius 0.5
    let mut particle = Particle::new(
        Vector3::new(0.0, 5.0, 0.0), 
        Vector3::zero(), 
        1.0, 
        0.5
    );

    // 3. Set a quaternary state (Index 0 = QuatState::One for high bounciness)
    particle.compressed_states.set(0, QuatState::One);

    // 4. Add particle to the world
    world.add_particle(particle);

    // 5. Run simulation steps (e.g., dt = 0.016s for ~60 FPS)
    let dt = 0.016;
    for step in 1..=60 {
        world.step(dt);
        println!("Step {}: Particle Y Position = {:.2}", step, world.particles[0].position.y);
    }
}
```

### 🔬 Advanced Quaternary Logic & SIMD Example

```rust
use quat_physics::quaternary::{QuatState, PackedQuads};

fn main() {
    // 1. Test quaternary operators (AND, OR, XOR, Multiply with carry)
    let s1 = QuatState::One;
    let s2 = QuatState::Two;
    let xor_result = s1.q_xor(s2);
    println!("Quaternary XOR result: {:?}", xor_result);

    // 2. SIMD-within-a-register (SWAR) batched operations
    let packed = PackedQuads::from_array([
        QuatState::Zero, 
        QuatState::One, 
        QuatState::Two, 
        QuatState::Three
    ]);
    
    // Invert all 4 states concurrently inside a single u8 byte
    let inverted = packed.simd_not();
    println!("Inverted packed quads raw data: {}", inverted.raw());
}
```

---

## 🧪 Running Tests

The library includes a comprehensive suite of unit tests ranging from basic quaternary logic to high-stress multi-particle collision clusters. 

To run the tests, execute:

```bash
cargo test
```