use crate::{BodyHandle, CollisionEvent, RigidBody, Shape};
use nalgebra::{Point2, Vector2};
use rapier2d::prelude::*;
use std::collections::HashMap;

/// The main physics world that manages all bodies and simulates physics
pub struct World {
    // Rapier physics pipeline components
    gravity: Vector2<f32>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,

    // Custom tracking
    body_map: HashMap<BodyHandle, RigidBodyHandle>,
    collider_to_body: HashMap<ColliderHandle, RigidBodyHandle>,
    collision_events: Vec<CollisionEvent>,
    timestep: f32,
    accumulator: f32,
}

impl World {
    /// Create a new physics world with default settings
    pub fn new() -> Self {
        WorldBuilder::new().build()
    }

    /// Get the fixed timestep
    pub fn timestep(&self) -> f32 {
        self.timestep
    }

    /// Set gravity for the world
    pub fn set_gravity(&mut self, gravity: Vector2<f32>) {
        self.gravity = gravity;
    }

    /// Add a rigid body to the world
    pub fn add_body(&mut self, body: &RigidBody) -> BodyHandle {
        // Create Rapier rigid body based on type
        let rigid_body = if body.is_static() {
            RigidBodyBuilder::fixed()
                .translation(Vector2::new(body.position().x, body.position().y))
                .build()
        } else {
            RigidBodyBuilder::dynamic()
                .translation(Vector2::new(body.position().x, body.position().y))
                .linvel(body.velocity())
                .can_sleep(false) // Keep bodies always active for ML training
                .build()
        };

        let rb_handle = self.rigid_body_set.insert(rigid_body);

        // Create collider based on shape
        let collider = match body.shape() {
            Shape::Circle(circle) => ColliderBuilder::ball(circle.radius).density(1.0).build(),
            Shape::Rectangle(rect) => ColliderBuilder::cuboid(rect.half_width, rect.half_height)
                .density(1.0)
                .build(),
        };

        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, rb_handle, &mut self.rigid_body_set);

        // Track the mappings
        let handle = BodyHandle::new();
        self.body_map.insert(handle, rb_handle);
        self.collider_to_body.insert(collider_handle, rb_handle);

        handle
    }

    /// Remove a body from the world
    pub fn remove_body(&mut self, handle: BodyHandle) -> Option<()> {
        let rb_handle = self.body_map.remove(&handle)?;

        // Remove colliders first
        let colliders: Vec<_> = self
            .rigid_body_set
            .get(rb_handle)?
            .colliders()
            .iter()
            .copied()
            .collect();

        for collider_handle in colliders {
            self.collider_to_body.remove(&collider_handle);
            self.collider_set.remove(
                collider_handle,
                &mut self.island_manager,
                &mut self.rigid_body_set,
                true,
            );
        }

        // Remove the rigid body
        self.rigid_body_set.remove(
            rb_handle,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );

        Some(())
    }

    /// Get body position
    pub fn body_position(&self, handle: BodyHandle) -> Option<Point2<f32>> {
        let rb_handle = self.body_map.get(&handle)?;
        let rb = self.rigid_body_set.get(*rb_handle)?;
        Some(Point2::new(rb.translation().x, rb.translation().y))
    }

    /// Set body position
    pub fn set_body_position(&mut self, handle: BodyHandle, position: Point2<f32>) -> Option<()> {
        let rb_handle = self.body_map.get(&handle)?;
        let rb = self.rigid_body_set.get_mut(*rb_handle)?;
        rb.set_translation(Vector2::new(position.x, position.y), true);
        Some(())
    }

    /// Get body velocity
    pub fn body_velocity(&self, handle: BodyHandle) -> Option<Vector2<f32>> {
        let rb_handle = self.body_map.get(&handle)?;
        let rb = self.rigid_body_set.get(*rb_handle)?;
        Some(*rb.linvel())
    }

    /// Set body velocity
    pub fn set_body_velocity(&mut self, handle: BodyHandle, velocity: Vector2<f32>) -> Option<()> {
        let rb_handle = self.body_map.get(&handle)?;
        let rb = self.rigid_body_set.get_mut(*rb_handle)?;
        rb.set_linvel(velocity, true);
        Some(())
    }

    /// Apply force to a body
    pub fn apply_force(&mut self, handle: BodyHandle, force: Vector2<f32>) -> Option<()> {
        let rb_handle = self.body_map.get(&handle)?;
        let rb = self.rigid_body_set.get_mut(*rb_handle)?;
        rb.add_force(force, true);
        Some(())
    }

    /// Apply impulse to a body
    pub fn apply_impulse(&mut self, handle: BodyHandle, impulse: Vector2<f32>) -> Option<()> {
        let rb_handle = self.body_map.get(&handle)?;
        let rb = self.rigid_body_set.get_mut(*rb_handle)?;
        rb.apply_impulse(impulse, true);
        Some(())
    }

    /// Step the simulation forward by a variable delta time
    /// Uses fixed timestep internally for determinism
    pub fn step(&mut self, dt: f32) {
        self.accumulator += dt;

        // Fixed timestep simulation for determinism
        while self.accumulator >= self.timestep {
            self.fixed_step();
            self.accumulator -= self.timestep;
        }
    }

    /// Perform one fixed timestep
    fn fixed_step(&mut self) {
        // Clear previous collision events
        self.collision_events.clear();

        // Step the simulation
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );

        // Collect collision events after the step
        // Note: For now, we'll leave collision event collection for a future enhancement
        // The basic physics simulation works without it
    }

    /// Get collision events from the last step
    pub fn collision_events(&self) -> &[CollisionEvent] {
        &self.collision_events
    }

    /// Clear all bodies from the world
    pub fn clear(&mut self) {
        // Collect all handles to remove
        let handles: Vec<_> = self.body_map.keys().cloned().collect();

        // Remove all bodies
        for handle in handles {
            self.remove_body(handle);
        }

        self.collision_events.clear();
        self.accumulator = 0.0;
    }
}

/// Builder for creating a physics world with custom settings
pub struct WorldBuilder {
    gravity: Vector2<f32>,
    timestep: f32,
}

impl WorldBuilder {
    /// Create a new world builder with default settings
    pub fn new() -> Self {
        Self {
            gravity: Vector2::new(0.0, -9.81),
            timestep: 1.0 / 60.0, // 60 Hz
        }
    }

    /// Set the gravity vector
    pub fn gravity(mut self, gravity: Vector2<f32>) -> Self {
        self.gravity = gravity;
        self
    }

    /// Set the fixed timestep for simulation
    pub fn timestep(mut self, timestep: f32) -> Self {
        self.timestep = timestep;
        self
    }

    /// Build the world
    pub fn build(self) -> World {
        let integration_parameters = IntegrationParameters {
            dt: self.timestep,
            ..Default::default()
        };

        World {
            gravity: self.gravity,
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            body_map: HashMap::new(),
            collider_to_body: HashMap::new(),
            collision_events: Vec::new(),
            timestep: self.timestep,
            accumulator: 0.0,
        }
    }
}

impl Default for WorldBuilder {
    fn default() -> Self {
        Self::new()
    }
}

// TODO: Implement collision event handling
// For now, collision detection is available through the narrow_phase after each step
// but we don't expose events through our API yet

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Circle;

    #[test]
    fn test_world_creation() {
        let world = WorldBuilder::new()
            .gravity(Vector2::new(0.0, -10.0))
            .timestep(1.0 / 120.0)
            .build();

        assert_eq!(world.timestep(), 1.0 / 120.0);
    }

    #[test]
    fn test_add_remove_body() {
        let mut world = World::new();

        let body = RigidBody::new(Point2::new(0.0, 0.0), Shape::Circle(Circle { radius: 1.0 }));

        let handle = world.add_body(&body);
        assert!(world.body_position(handle).is_some());

        world.remove_body(handle);
        assert!(world.body_position(handle).is_none());
    }

    #[test]
    fn test_body_movement() {
        let mut world = World::new();

        let body = RigidBody::new(Point2::new(0.0, 0.0), Shape::Circle(Circle { radius: 1.0 }));

        let handle = world.add_body(&body);

        // Set velocity
        world.set_body_velocity(handle, Vector2::new(1.0, 0.0));

        // Step simulation
        world.step(1.0 / 60.0);

        // Check position changed
        let pos = world.body_position(handle).unwrap();
        assert!(pos.x > 0.0);
    }
}
