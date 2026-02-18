use pyo3::prelude::*;
use pyo3::types::PyDict;
use nalgebra::{Point2, Vector2};
use crate::{World, WorldBuilder, RigidBody, Shape, Circle, Rectangle, BodyHandle, CollisionEvent};

/// Python wrapper for the physics World
#[pyclass(name = "World")]
struct PyWorld {
    world: World,
}

#[pymethods]
impl PyWorld {
    /// Create a new physics world
    #[new]
    #[pyo3(signature = (gravity_x=0.0, gravity_y=-9.81, timestep=0.016667))]
    fn new(gravity_x: f32, gravity_y: f32, timestep: f32) -> Self {
        let world = WorldBuilder::new()
            .gravity(Vector2::new(gravity_x, gravity_y))
            .timestep(timestep)
            .build();
            
        PyWorld { world }
    }
    
    /// Set gravity
    fn set_gravity(&mut self, x: f32, y: f32) {
        self.world.set_gravity(Vector2::new(x, y));
    }
    
    /// Add a circular body
    fn add_circle(&mut self, x: f32, y: f32, radius: f32, vx: f32, vy: f32) -> u64 {
        let body = RigidBody::new(
            Point2::new(x, y),
            Shape::Circle(Circle::new(radius)),
        )
        .with_velocity(Vector2::new(vx, vy));
        
        let handle = self.world.add_body(&body);
        // Return the raw handle value for Python
        unsafe { std::mem::transmute(handle) }
    }
    
    /// Add a rectangular body
    fn add_rectangle(&mut self, x: f32, y: f32, width: f32, height: f32, vx: f32, vy: f32) -> u64 {
        let body = RigidBody::new(
            Point2::new(x, y),
            Shape::Rectangle(Rectangle::from_size(width, height)),
        )
        .with_velocity(Vector2::new(vx, vy));
        
        let handle = self.world.add_body(&body);
        unsafe { std::mem::transmute(handle) }
    }
    
    /// Remove a body
    fn remove_body(&mut self, handle: u64) -> bool {
        let handle: BodyHandle = unsafe { std::mem::transmute(handle) };
        self.world.remove_body(handle).is_some()
    }
    
    /// Get body position
    fn get_position(&self, handle: u64) -> Option<(f32, f32)> {
        let handle: BodyHandle = unsafe { std::mem::transmute(handle) };
        self.world.body_position(handle)
            .map(|p| (p.x, p.y))
    }
    
    /// Set body position
    fn set_position(&mut self, handle: u64, x: f32, y: f32) -> bool {
        let handle: BodyHandle = unsafe { std::mem::transmute(handle) };
        self.world.set_body_position(handle, Point2::new(x, y)).is_some()
    }
    
    /// Get body velocity
    fn get_velocity(&self, handle: u64) -> Option<(f32, f32)> {
        let handle: BodyHandle = unsafe { std::mem::transmute(handle) };
        self.world.body_velocity(handle)
            .map(|v| (v.x, v.y))
    }
    
    /// Set body velocity
    fn set_velocity(&mut self, handle: u64, vx: f32, vy: f32) -> bool {
        let handle: BodyHandle = unsafe { std::mem::transmute(handle) };
        self.world.set_body_velocity(handle, Vector2::new(vx, vy)).is_some()
    }
    
    /// Apply force to a body
    fn apply_force(&mut self, handle: u64, fx: f32, fy: f32) -> bool {
        let handle: BodyHandle = unsafe { std::mem::transmute(handle) };
        self.world.apply_force(handle, Vector2::new(fx, fy)).is_some()
    }
    
    /// Apply impulse to a body
    fn apply_impulse(&mut self, handle: u64, ix: f32, iy: f32) -> bool {
        let handle: BodyHandle = unsafe { std::mem::transmute(handle) };
        self.world.apply_impulse(handle, Vector2::new(ix, iy)).is_some()
    }
    
    /// Step the simulation
    fn step(&mut self, dt: f32) {
        self.world.step(dt);
    }
    
    /// Get collision events from last step
    fn get_collision_events(&self) -> Vec<PyCollisionEvent> {
        self.world.collision_events()
            .iter()
            .map(|e| PyCollisionEvent::from(e))
            .collect()
    }
    
    /// Clear all bodies
    fn clear(&mut self) {
        self.world.clear();
    }
    
    /// Get the fixed timestep
    #[getter]
    fn timestep(&self) -> f32 {
        self.world.timestep()
    }
}

/// Python wrapper for collision events
#[pyclass(name = "CollisionEvent")]
#[derive(Clone)]
struct PyCollisionEvent {
    #[pyo3(get)]
    body_a: u64,
    #[pyo3(get)]
    body_b: u64,
    #[pyo3(get)]
    contact_x: f32,
    #[pyo3(get)]
    contact_y: f32,
    #[pyo3(get)]
    normal_x: f32,
    #[pyo3(get)]
    normal_y: f32,
    #[pyo3(get)]
    depth: f32,
    #[pyo3(get)]
    is_started: bool,
    #[pyo3(get)]
    is_stopped: bool,
}

impl From<&CollisionEvent> for PyCollisionEvent {
    fn from(event: &CollisionEvent) -> Self {
        PyCollisionEvent {
            body_a: unsafe { std::mem::transmute(event.body_a) },
            body_b: unsafe { std::mem::transmute(event.body_b) },
            contact_x: event.contact_point.x,
            contact_y: event.contact_point.y,
            normal_x: event.normal.x,
            normal_y: event.normal.y,
            depth: event.depth,
            is_started: event.is_started(),
            is_stopped: event.is_stopped(),
        }
    }
}

/// Python module definition
#[pymodule]
#[pyo3(name = "evolve_physics")]
fn evolve_physics_python(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyWorld>()?;
    m.add_class::<PyCollisionEvent>()?;
    
    // Add version info
    m.add("__version__", "0.1.0")?;
    
    // Add some constants
    let physics_constants = PyDict::new(m.py());
    physics_constants.set_item("DEFAULT_GRAVITY", -9.81)?;
    physics_constants.set_item("DEFAULT_TIMESTEP", 1.0 / 60.0)?;
    m.add("constants", physics_constants)?;
    
    Ok(())
}