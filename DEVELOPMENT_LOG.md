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

---

## 2026-02-18: Phase 2 — Direct Body State, Collider Wiring, Autoload

### What was added

1. **`EvolveDirectBodyState` (PhysicsDirectBodyState2DExtension)**
   - Snapshot-based: populated when `body_get_direct_state()` is called
   - Returns transform, linear/angular velocity, inverse mass/inertia, gravity, step size
   - Contact methods stubbed (return 0/empty) — no contact reporting yet
   - Write-back methods (apply_impulse, etc.) are no-ops for now; forces should be applied through the server API

2. **Shape → Collider wiring**
   - When `body_set_space()` is called and a body enters a space, Rapier colliders are now created for all attached shapes
   - Supports Circle (ball) and Rectangle (cuboid) colliders
   - Capsule shape partially supported (uses default dimensions — needs proper Variant parsing)
   - Collider handles stored in `BodyShapeEntry` for cleanup on removal
   - `build_collider_for_shape()` and `sync_body_colliders()` helper methods added

3. **Autoload registration script** (`godot/register_physics.gd`)
   - Registers "EvolvePhysics" with `PhysicsServer2DManager`
   - Detects whether the extension class is available
   - `godot/README.md` with setup instructions

### Still stubbed
- `PhysicsDirectSpaceState2DExtension` (raycasts, shape queries)
- Joints (data structures exist, no Rapier mapping)
- Areas (data storage only, no physics effect)
- Collision exceptions
- Contact reporting in direct body state
- Convex/concave polygon colliders
- Body test motion
- Write-back from direct body state to Rapier (impulses, velocity changes during `_integrate_forces`)

### Notes
- Determinism priority: fixed timestep Rapier2D, `can_sleep(false)` for ML training
- Python bindings require PyO3 0.23 which doesn't support Python 3.14 yet; use `PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1` or Python ≤3.13
