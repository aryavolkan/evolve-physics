# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Evolve Physics is a deterministic 2D physics engine for ML/evolution training simulations. Built on Rapier2D, it provides headless rigid body physics with Python bindings (PyO3) and Godot 4 integration (gdext).

## Tech Stack

- **Language**: Rust (core), Python (bindings via PyO3 0.23), GDScript (Godot autoload)
- **Physics**: Rapier2D 0.22, nalgebra 0.33
- **Features**: `godot` (default, gdext), `python` (PyO3)

## Build, Test, Lint

```bash
# Build
cargo build --release                                    # Default (Godot)
cargo build --features python --no-default-features      # Python module only

# Python module (via maturin)
maturin develop                    # Dev install
maturin build --release            # Wheel

# Test
cargo test

# Lint
cargo fmt --check
cargo clippy -- -D warnings
```

## Architecture

| Module | Purpose |
|--------|---------|
| `world.rs` | Core simulation: manages Rapier2D pipeline, body/collider tracking, `step()` for fixed-timestep advance |
| `body.rs` | `RigidBody` struct (position, velocity, shape, mass, is_static), `BodyHandle` unique IDs |
| `shapes.rs` | `Circle` and `Rectangle` shape definitions |
| `collision.rs` | `CollisionEvent`, `ContactPair`, `Contact` structs |
| `python.rs` | `PyWorld` class: `add_circle/rectangle()`, `get/set_position/velocity()`, `apply_force/impulse()`, `step()` |
| `godot_plugin.rs` | `PhysicsServer2DExtension` implementation (shape creation, body management, forces; ray casting stubbed) |

### Key design decisions

- **Fixed timestep** for deterministic, reproducible simulations
- **Bodies never sleep** (`can_sleep(false)`) to maintain ML determinism
- **Builder pattern** for World configuration (`WorldBuilder`)
- **Release profile**: opt-level=3, LTO, codegen-units=1 for maximum performance

## CI

GitHub Actions (`.github/workflows/rust.yml`): fmt check, clippy, build, tests.
