# Evolve Physics

A lightweight, deterministic physics engine for ML/evolution training simulations, built in Rust as an alternative to Godot Physics.

## Features

- **2D Rigid Body Dynamics**: Position, velocity, forces, and impulses
- **Collision Detection**: Support for circles and axis-aligned rectangles
- **Deterministic Simulation**: Fixed timestep for reproducible results
- **Python Bindings**: Easy integration with Python training code via PyO3
- **Headless Operation**: No rendering dependencies, perfect for server training
- **Built on Rapier2D**: Leverages the robust Rapier physics engine

## Installation

### Rust Library

Add to your `Cargo.toml`:

```toml
[dependencies]
evolve-physics = { path = "../evolve-physics" }
```

### Python Module

Build and install the Python module:

```bash
# Build the Python wheel
cd ~/projects/evolve-physics
maturin build --release

# Install the wheel
pip install target/wheels/evolve_physics-*.whl
```

## Usage

### Rust Example

```rust
use evolve_physics::prelude::*;

fn main() {
    // Create a physics world
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, -9.81))
        .timestep(1.0 / 60.0)
        .build();
    
    // Add a dynamic body
    let ball = RigidBody::new(
        Point2::new(0.0, 10.0),
        Shape::Circle(Circle::new(1.0)),
    );
    let ball_handle = world.add_body(&ball);
    
    // Add a static ground
    let ground = RigidBody::new_static(
        Point2::new(0.0, -5.0),
        Shape::Rectangle(Rectangle::from_size(20.0, 1.0)),
    );
    world.add_body(&ground);
    
    // Simulate for 1 second
    for _ in 0..60 {
        world.step(1.0 / 60.0);
        
        if let Some(pos) = world.body_position(ball_handle) {
            println!("Ball position: ({}, {})", pos.x, pos.y);
        }
    }
}
```

### Python Example

```python
import evolve_physics

# Create a physics world
world = evolve_physics.World(
    gravity_x=0.0, 
    gravity_y=-9.81,
    timestep=1.0/60.0
)

# Add a creature (circle)
creature = world.add_circle(
    x=0.0, y=10.0,      # position
    radius=1.0,         # size
    vx=5.0, vy=0.0     # initial velocity
)

# Add ground (rectangle)
ground = world.add_rectangle(
    x=0.0, y=-5.0,      # position
    width=40.0,         # width
    height=2.0,         # height
    vx=0.0, vy=0.0     # velocity (static)
)

# Run simulation
for step in range(600):  # 10 seconds at 60Hz
    world.step(1.0/60.0)
    
    # Get creature position
    pos = world.get_position(creature)
    if pos:
        x, y = pos
        print(f"Step {step}: creature at ({x:.2f}, {y:.2f})")
    
    # Apply random forces (simulate neural network outputs)
    if step % 30 == 0:
        world.apply_force(creature, fx=10.0, fy=20.0)
```

## Comparison to Godot Physics

| Feature | Godot Physics | Evolve Physics |
|---------|--------------|----------------|
| 2D Rigid Bodies | ✓ | ✓ |
| 3D Physics | ✓ | ✗ |
| Complex Shapes | ✓ | Limited (circle, rect) |
| Joints | ✓ | ✗ (planned) |
| Rendering | ✓ | ✗ (headless) |
| Python Bindings | Via GDScript | Native PyO3 |
| Deterministic | With settings | Always |
| Performance | Good | Excellent |
| Memory Usage | Higher | Lower |

## Architecture

The engine is built on top of Rapier2D, providing:

1. **World**: Manages the simulation, bodies, and collision detection
2. **RigidBody**: Represents physical objects with mass, position, and velocity
3. **Shapes**: Simple collision shapes (circles and rectangles)
4. **Collision Events**: Reports when bodies start/stop touching

## Performance

- Optimized for many small bodies (typical in evolution simulations)
- Parallel collision detection via Rapier's SIMD optimizations
- Minimal allocations during simulation
- Fixed timestep ensures consistent performance

## Future Enhancements

- [ ] Joint constraints (hinge, ball-socket, fixed)
- [ ] More collision shapes (capsules, polygons)
- [ ] Sensor bodies (collision detection without response)
- [ ] Ray casting for creature sensors
- [ ] Spatial queries for neighbor detection
- [ ] Custom collision filters
- [ ] State serialization for checkpointing

## Development

```bash
# Build (default includes Godot integration)
cargo build --release

# Build Python module only
cargo build --features python --no-default-features --release

# Build Godot plugin only
cargo build --features godot --no-default-features --release

# Build + install Python module
maturin develop              # for development
maturin build --release      # for distribution wheel

# Run tests
cargo test

# Lint
cargo fmt --check
cargo clippy -- -D warnings
```

### Module Breakdown

| File | Purpose |
|------|---------|
| `world.rs` | Manages Rapier2D pipeline, body/collider tracking, `step()` fixed-timestep advance |
| `body.rs` | `RigidBody` struct, `BodyHandle` unique IDs, static/dynamic constructors |
| `shapes.rs` | `Circle` and `Rectangle` shape definitions |
| `collision.rs` | `CollisionEvent`, `ContactPair`, `Contact` structs |
| `python.rs` | PyO3 `PyWorld` class wrapping the full API |
| `godot_plugin.rs` | `PhysicsServer2DExtension` (shapes, bodies, forces; ray casting stubbed) |

### Design Decisions

- **Fixed timestep** for deterministic, reproducible ML simulations
- **Bodies never sleep** (`can_sleep(false)`) to maintain determinism
- **Builder pattern** for World configuration (`WorldBuilder`)
- **Release profile**: `opt-level=3`, LTO, `codegen-units=1` for maximum performance

## License

MIT License - feel free to use in your evolution experiments!