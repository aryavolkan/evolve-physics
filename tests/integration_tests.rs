use approx::assert_relative_eq;
use evolve_physics::prelude::*;

#[test]
fn test_gravity_simulation() {
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, -10.0))
        .timestep(0.01)
        .build();

    // Drop a ball from height
    let ball = RigidBody::new(Point2::new(0.0, 10.0), Shape::Circle(Circle::new(1.0)));
    let handle = world.add_body(&ball);

    // Simulate for 1 second (100 steps at 0.01s timestep)
    for _ in 0..100 {
        world.step(0.01);
    }

    // Check position after 1 second
    // y = y0 + v0*t + 0.5*g*t^2 = 10 + 0 + 0.5*(-10)*1^2 = 5
    let pos = world.body_position(handle).unwrap();
    assert_relative_eq!(pos.y, 5.0, epsilon = 0.1);
}

#[test]
fn test_collision_with_ground() {
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, -10.0))
        .timestep(0.01)
        .build();

    // Ball above ground
    let ball = RigidBody::new(Point2::new(0.0, 5.0), Shape::Circle(Circle::new(1.0)));
    let ball_handle = world.add_body(&ball);

    // Static ground at y=0 (so top surface is at y=5 due to half_height=5)
    let ground = RigidBody::new_static(
        Point2::new(0.0, 0.0),
        Shape::Rectangle(Rectangle::from_size(40.0, 10.0)),
    );
    world.add_body(&ground);

    // Track ball position over time
    let mut final_y = 0.0;
    for i in 0..300 {
        world.step(0.01);
        if let Some(pos) = world.body_position(ball_handle) {
            final_y = pos.y;
            // Print every 50 steps for debugging
            if i % 50 == 0 {
                println!("Step {}: Ball at y={}", i, pos.y);
            }
        }
    }

    // Ball should be resting on ground (ground top at y=5 + ball radius 1 = 6)
    println!("Final ball position: y={}", final_y);
    assert!(final_y > 5.8, "Ball y position {} is too low", final_y);
    assert!(final_y < 6.2, "Ball y position {} is too high", final_y);
}

#[test]
fn test_horizontal_motion() {
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, 0.0)) // No gravity
        .timestep(0.01)
        .build();

    // Moving ball
    let ball = RigidBody::new(Point2::new(0.0, 0.0), Shape::Circle(Circle::new(1.0)))
        .with_velocity(Vector2::new(5.0, 0.0));

    let handle = world.add_body(&ball);

    // Simulate for 2 seconds
    for _ in 0..200 {
        world.step(0.01);
    }

    // x = x0 + vx*t = 0 + 5*2 = 10
    let pos = world.body_position(handle).unwrap();
    assert_relative_eq!(pos.x, 10.0, epsilon = 0.1);
    assert_relative_eq!(pos.y, 0.0, epsilon = 0.01);
}

#[test]
fn test_force_application() {
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, 0.0))
        .timestep(0.01)
        .build();

    let body = RigidBody::new(Point2::new(0.0, 0.0), Shape::Circle(Circle::new(1.0)));
    let handle = world.add_body(&body);

    // Apply constant force for 1 second
    for _ in 0..100 {
        world.apply_force(handle, Vector2::new(10.0, 0.0));
        world.step(0.01);
    }

    // Force = mass * acceleration, F=10, m=1 (density*area), so a=10
    // v = a*t = 10*1 = 10
    let vel = world.body_velocity(handle).unwrap();
    assert!(vel.x > 5.0); // Should have gained significant velocity
}

#[test]
fn test_impulse_application() {
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, 0.0))
        .timestep(0.01)
        .build();

    let body = RigidBody::new(Point2::new(0.0, 0.0), Shape::Circle(Circle::new(1.0)));
    let handle = world.add_body(&body);

    // Apply single impulse
    world.apply_impulse(handle, Vector2::new(10.0, 0.0));
    world.step(0.01);

    // Impulse = mass * velocity change
    let vel = world.body_velocity(handle).unwrap();
    assert!(vel.x > 1.0); // Should have immediate velocity
}

#[test]
fn test_multiple_bodies() {
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, -10.0))
        .timestep(0.01)
        .build();

    let mut handles = vec![];

    // Create 10 balls in a row
    for i in 0..10 {
        let ball = RigidBody::new(
            Point2::new(i as f32 * 3.0, 10.0),
            Shape::Circle(Circle::new(0.5)),
        );
        handles.push(world.add_body(&ball));
    }

    // Simulate
    for _ in 0..100 {
        world.step(0.01);
    }

    // All balls should have fallen
    for handle in handles {
        let pos = world.body_position(handle).unwrap();
        assert!(pos.y < 10.0);
    }
}

#[test]
fn test_body_removal() {
    let mut world = World::new();

    let body1 = RigidBody::new(Point2::new(0.0, 0.0), Shape::Circle(Circle::new(1.0)));
    let body2 = RigidBody::new(Point2::new(5.0, 0.0), Shape::Circle(Circle::new(1.0)));

    let handle1 = world.add_body(&body1);
    let handle2 = world.add_body(&body2);

    // Both bodies exist
    assert!(world.body_position(handle1).is_some());
    assert!(world.body_position(handle2).is_some());

    // Remove first body
    world.remove_body(handle1);

    // First gone, second still exists
    assert!(world.body_position(handle1).is_none());
    assert!(world.body_position(handle2).is_some());
}

#[test]
fn test_world_clear() {
    let mut world = World::new();

    // Add multiple bodies
    for i in 0..5 {
        let body = RigidBody::new(Point2::new(i as f32, 0.0), Shape::Circle(Circle::new(1.0)));
        world.add_body(&body);
    }

    world.clear();

    // Try adding new body after clear
    let new_body = RigidBody::new(Point2::new(0.0, 0.0), Shape::Circle(Circle::new(1.0)));
    let handle = world.add_body(&new_body);

    assert!(world.body_position(handle).is_some());
}

#[test]
fn test_rectangle_shapes() {
    let mut world = World::new();

    let rect1 = RigidBody::new(
        Point2::new(0.0, 0.0),
        Shape::Rectangle(Rectangle::from_size(4.0, 2.0)),
    );
    let rect2 = RigidBody::new(
        Point2::new(10.0, 0.0),
        Shape::Rectangle(Rectangle::new(3.0, 1.5)), // half-extents
    );

    let h1 = world.add_body(&rect1);
    let h2 = world.add_body(&rect2);

    assert_eq!(world.body_position(h1).unwrap(), Point2::new(0.0, 0.0));
    assert_eq!(world.body_position(h2).unwrap(), Point2::new(10.0, 0.0));
}
