#![doc = include_str!("../README.md")]

pub mod body;
pub mod collision;
pub mod world;
pub mod shapes;

// Python bindings module
pub mod python;

pub use body::{RigidBody, BodyHandle};
pub use collision::{CollisionEvent, ContactPair as ContactPairInfo};
pub use shapes::{Shape, Circle, Rectangle};
pub use world::{World, WorldBuilder};

// Re-export commonly used types from nalgebra
pub use nalgebra::{Point2, Vector2};

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::{
        World, WorldBuilder,
        RigidBody, BodyHandle,
        Shape, Circle, Rectangle,
        CollisionEvent, ContactPairInfo,
        Point2, Vector2,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_world_creation() {
        let world = WorldBuilder::new()
            .gravity(Vector2::new(0.0, -9.81))
            .timestep(1.0 / 60.0)
            .build();
        
        assert_eq!(world.timestep(), 1.0 / 60.0);
    }
}