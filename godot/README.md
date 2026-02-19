# EvolvePhysics — Godot Integration

## Setup

1. **Copy the `.gdextension` file** to your Godot project root (or symlink it).

2. **Add the autoload script**: In Godot, go to `Project > Project Settings > Autoload`, add `register_physics.gd` with any name (e.g. `EvolvePhysicsRegistrar`).

3. **Select the physics engine**: Go to `Project Settings > Physics > 2D > Physics Engine` and choose `EvolvePhysics` from the dropdown.

4. **Restart the editor** to activate the new physics server.

## What works

- Rigid body simulation (dynamic, static, kinematic)
- Circle and rectangle collider shapes
- Forces, impulses, torques
- Gravity
- `body.get_direct_state()` returns position, velocity, transform, etc.
- Deterministic Rapier2D stepping

## What's still stubbed

- `PhysicsDirectSpaceState2D` (raycasts, shape queries)
- Joints (pin, groove, damped spring)
- Areas (gravity overrides, monitoring)
- Collision exceptions
- Contact reporting in direct body state
- Convex/concave polygon colliders
- Body test motion
