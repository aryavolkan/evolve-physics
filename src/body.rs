use crate::shapes::Shape;
use nalgebra::{Point2, Vector2};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};

/// Handle to a rigid body in the physics world
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BodyHandle(u64);

impl BodyHandle {
    /// Create a new unique body handle
    pub fn new() -> Self {
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        Self(COUNTER.fetch_add(1, Ordering::Relaxed))
    }
}

impl Default for BodyHandle {
    fn default() -> Self {
        Self::new()
    }
}

/// A rigid body that can be added to the physics world
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RigidBody {
    position: Point2<f32>,
    velocity: Vector2<f32>,
    shape: Shape,
    mass: f32,
    is_static: bool,
}

impl RigidBody {
    /// Create a new dynamic rigid body
    pub fn new(position: Point2<f32>, shape: Shape) -> Self {
        Self {
            position,
            velocity: Vector2::zeros(),
            shape,
            mass: 1.0,
            is_static: false,
        }
    }

    /// Create a new static rigid body (doesn't move)
    pub fn new_static(position: Point2<f32>, shape: Shape) -> Self {
        Self {
            position,
            velocity: Vector2::zeros(),
            shape,
            mass: f32::INFINITY,
            is_static: true,
        }
    }

    /// Builder method to set initial velocity
    pub fn with_velocity(mut self, velocity: Vector2<f32>) -> Self {
        self.velocity = velocity;
        self
    }

    /// Builder method to set mass
    pub fn with_mass(mut self, mass: f32) -> Self {
        self.mass = mass;
        self
    }

    /// Get the position
    pub fn position(&self) -> Point2<f32> {
        self.position
    }

    /// Get the velocity
    pub fn velocity(&self) -> Vector2<f32> {
        self.velocity
    }

    /// Get the shape
    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    /// Get the mass
    pub fn mass(&self) -> f32 {
        self.mass
    }

    /// Check if body is static
    pub fn is_static(&self) -> bool {
        self.is_static
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::{Circle, Rectangle};

    #[test]
    fn test_body_creation() {
        let body = RigidBody::new(
            Point2::new(10.0, 20.0),
            Shape::Circle(Circle { radius: 5.0 }),
        );

        assert_eq!(body.position(), Point2::new(10.0, 20.0));
        assert_eq!(body.velocity(), Vector2::zeros());
        assert!(!body.is_static());
    }

    #[test]
    fn test_body_builder() {
        let body = RigidBody::new(
            Point2::new(0.0, 0.0),
            Shape::Rectangle(Rectangle {
                half_width: 10.0,
                half_height: 5.0,
            }),
        )
        .with_velocity(Vector2::new(5.0, -3.0))
        .with_mass(2.5);

        assert_eq!(body.velocity(), Vector2::new(5.0, -3.0));
        assert_eq!(body.mass(), 2.5);
    }

    #[test]
    fn test_static_body() {
        let body = RigidBody::new_static(
            Point2::new(0.0, -50.0),
            Shape::Rectangle(Rectangle {
                half_width: 100.0,
                half_height: 5.0,
            }),
        );

        assert!(body.is_static());
        assert_eq!(body.mass(), f32::INFINITY);
    }

    #[test]
    fn test_handle_uniqueness() {
        let h1 = BodyHandle::new();
        let h2 = BodyHandle::new();
        let h3 = BodyHandle::new();

        assert_ne!(h1, h2);
        assert_ne!(h2, h3);
        assert_ne!(h1, h3);
    }
}
