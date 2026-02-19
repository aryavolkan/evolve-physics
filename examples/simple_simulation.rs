use evolve_physics::prelude::*;

fn main() {
    println!("=== Evolve Physics Engine Demo ===\n");

    // Create a physics world with gravity
    let mut world = WorldBuilder::new()
        .gravity(Vector2::new(0.0, -9.81))
        .timestep(1.0 / 60.0)
        .build();

    println!("Created world with gravity: -9.81 m/s²");
    println!("Fixed timestep: {} seconds", world.timestep());

    // Add a bouncing ball
    let ball = RigidBody::new(Point2::new(0.0, 10.0), Shape::Circle(Circle::new(1.0)))
        .with_velocity(Vector2::new(5.0, 0.0));

    let ball_handle = world.add_body(&ball);
    println!("\nAdded ball at (0, 10) with velocity (5, 0)");

    // Add a static ground
    let ground = RigidBody::new_static(
        Point2::new(0.0, 0.0),
        Shape::Rectangle(Rectangle::from_size(50.0, 2.0)),
    );
    world.add_body(&ground);
    println!("Added ground at y=0");

    // Add some obstacles
    for i in 0..3 {
        let obstacle = RigidBody::new_static(
            Point2::new(10.0 + i as f32 * 10.0, 3.0 + i as f32 * 2.0),
            Shape::Rectangle(Rectangle::from_size(4.0, 4.0)),
        );
        world.add_body(&obstacle);
    }
    println!("Added 3 obstacles");

    // Simulate for 5 seconds
    println!("\nSimulating...\n");

    let total_steps = 300; // 5 seconds at 60 Hz
    for step in 0..total_steps {
        // Apply some random forces to make it interesting
        if step % 60 == 0 && step > 0 {
            world.apply_impulse(ball_handle, Vector2::new(-3.0, 10.0));
            println!("Applied upward impulse at t={:.1}s", step as f32 / 60.0);
        }

        // Step the simulation
        world.step(1.0 / 60.0);

        // Print position every second
        if step % 60 == 0 {
            if let Some(pos) = world.body_position(ball_handle) {
                if let Some(vel) = world.body_velocity(ball_handle) {
                    println!(
                        "t={:.1}s: Ball at ({:.2}, {:.2}), velocity ({:.2}, {:.2})",
                        step as f32 / 60.0,
                        pos.x,
                        pos.y,
                        vel.x,
                        vel.y
                    );
                }
            }
        }
    }

    println!("\nSimulation complete!");
}
