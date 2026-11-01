# quat_physics

`quat_physics` is a lightweight, memory-efficient 3D physics library written in Rust. It uniquely combines custom vector mathematics and particle simulation with **quaternary logic (2-bit packing)**, achieving a **75% memory reduction** for particle states and material properties compared to traditional byte storage.

---

## 🚀 Key Features

* **Memory-Efficient Quaternary Logic (`PackedQuads`)**: Packs 4 distinct quaternary states (`00`, `01`, `10`, `11`) into a single `u8` byte.
* **Material-Driven Physics**: 2-bit packed states directly control runtime physical behaviors, such as particle restitution (bounciness, stickiness, or dead stops) during collisions.
* **Robust 3D Vector Math (`Vector3`)**: Complete operator overloading (`+`, `-`, `*`, `/`, `+=`, `-=`, etc.) along with dot product, cross product, normalization, and magnitude calculations.
* **Simulation World (`PhysicsWorld`)**: Semi-implicit Euler integration, Newton's second law force application ($F = ma$), global gravity, and strict world boundary enforcement.
* **Collision Resolution**: Accurate sphere-to-sphere and sphere-to-boundary collision detection and response.

---

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
quat_physics = "0.3.0"
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

---

## 🧪 Running Tests

The library includes a comprehensive suite of unit tests ranging from basic quaternary logic to high-stress multi-particle collision clusters. 

To run the tests, execute:

```bash
cargo test
```