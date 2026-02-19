# Development Log

## 2026-02-18: PhysicsServer2DExtension GDExtension Backend

### What was built
Implemented a Godot 4 GDExtension that provides a custom `PhysicsServer2DExtension` backed by Rapier2D. This allows Godot to delegate all 2D physics calls to our deterministic Rapier2D engine instead of its built-in physics.

### Architecture
- **`src/godot_plugin.rs`** — `EvolvePhysicsServer` class extending `PhysicsServer2DExtension`
- Feature-gated: `cargo build --features godot --no-default-features` for Godot, default features for Python
- Pinned to gdext commit `003b41c` (same as evolve-native) for compatibility

### Implementation status

**Fully implemented (Phase 1 — Core):**
- Shape creation (circle, rectangle, capsule, world boundary, segment, convex/concave polygon)
- Shape data get/set
- Space create, activate, step (runs Rapier2D pipeline)
- Body create, set space, set mode (static/kinematic/rigid)
- Body state get/set (transform, linear velocity, angular velocity)
- Body parameters (mass, linear/angular damping, gravity scale)
- Body shapes (add/set/remove/clear)

**Fully implemented (Phase 2 — Forces & Collisions):**
- `body_apply_impulse`, `body_apply_central_impulse`, `body_apply_torque_impulse`
- `body_apply_force`, `body_apply_central_force`, `body_apply_torque`
- Constant forces and torques
- Collision layer/mask storage
- CCD toggle

**Stubbed (implement later):**
- `space_get_direct_state()` — needs PhysicsDirectSpaceState2DExtension
- `body_get_direct_state()` — needs PhysicsDirectBodyState2DExtension
- `body_test_motion` / `shape_collide` — ray/shape casting
- Joints (pin, groove, damped spring) — data structures present, no Rapier mapping yet
- Areas — data storage works, no physics effect yet
- Collision exceptions

### How to use
1. Build: `cargo build --features godot --no-default-features`
2. The `.gdextension` file is symlinked into `~/Projects/evolve/`
3. To register as the physics server, add to project settings or use `PhysicsServer2DManager`

### Notes
- Determinism priority: fixed timestep Rapier2D, `can_sleep(false)` for ML training
- Python bindings require PyO3 0.23 which doesn't support Python 3.14 yet; use `PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1` or Python ≤3.13
