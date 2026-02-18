#!/usr/bin/env python3
"""
Example of using evolve-physics for ML/evolution training.
This simulates simple creatures learning to navigate towards food.
"""

import evolve_physics
import random
import math

class Creature:
    """Simple creature with basic sensors and movement."""
    
    def __init__(self, world, x, y):
        self.world = world
        # Create physics body
        self.body = world.add_circle(
            x=x, y=y,
            radius=0.5,
            vx=0.0, vy=0.0
        )
        self.fitness = 0.0
        self.energy = 100.0
        
        # Simple neural network weights (random initialization)
        self.weights = [random.uniform(-1, 1) for _ in range(6)]
    
    def sense_environment(self, food_positions):
        """Get sensory input about nearby food."""
        pos = self.world.get_position(self.body)
        if not pos:
            return [0, 0, 0, 0]
        
        my_x, my_y = pos
        
        # Find closest food
        min_dist = float('inf')
        closest_food = None
        
        for food_x, food_y in food_positions:
            dist = math.sqrt((food_x - my_x)**2 + (food_y - my_y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_food = (food_x, food_y)
        
        if not closest_food:
            return [0, 0, 0, 0]
        
        # Calculate relative position and distance
        fx, fy = closest_food
        dx = fx - my_x
        dy = fy - my_y
        
        # Normalize inputs
        return [
            dx / 10.0,  # Relative X (normalized)
            dy / 10.0,  # Relative Y (normalized)
            min_dist / 10.0,  # Distance (normalized)
            self.energy / 100.0  # Current energy level
        ]
    
    def think(self, inputs):
        """Simple neural network to decide movement."""
        # 4 inputs -> 2 outputs (force X, force Y)
        # Using tanh activation
        
        force_x = math.tanh(
            inputs[0] * self.weights[0] +
            inputs[1] * self.weights[1] +
            inputs[2] * self.weights[2] +
            inputs[3] * self.weights[3]
        )
        
        force_y = math.tanh(
            inputs[0] * self.weights[2] +
            inputs[1] * self.weights[3] +
            inputs[2] * self.weights[4] +
            inputs[3] * self.weights[5]
        )
        
        return force_x * 5.0, force_y * 5.0
    
    def update(self, food_positions):
        """Sense environment and apply movement forces."""
        # Get sensory input
        inputs = self.sense_environment(food_positions)
        
        # Decide movement
        fx, fy = self.think(inputs)
        
        # Apply forces
        self.world.apply_force(self.body, fx=fx, fy=fy)
        
        # Consume energy for movement
        self.energy -= 0.1 + abs(fx) * 0.05 + abs(fy) * 0.05
        
        # Check food collection
        pos = self.world.get_position(self.body)
        if pos:
            my_x, my_y = pos
            for i, (food_x, food_y) in enumerate(food_positions):
                dist = math.sqrt((food_x - my_x)**2 + (food_y - my_y)**2)
                if dist < 1.0:  # Close enough to eat
                    self.fitness += 10.0
                    self.energy += 20.0
                    return i  # Return index of eaten food
        
        return None
    
    def is_alive(self):
        return self.energy > 0


def run_simulation(creature_weights=None, visualize=True):
    """Run a single simulation with given creature weights."""
    
    # Create physics world (top-down view, no gravity)
    world = evolve_physics.World(
        gravity_x=0.0,
        gravity_y=0.0,
        timestep=1.0/30.0
    )
    
    # Add arena walls
    walls = [
        # x, y, width, height
        (0, -10, 40, 1),    # Bottom
        (0, 10, 40, 1),     # Top
        (-20, 0, 1, 20),    # Left
        (20, 0, 1, 20),     # Right
    ]
    
    for x, y, w, h in walls:
        world.add_rectangle(x, y, w, h, 0, 0)
    
    # Create creature
    creature = Creature(world, 0, 0)
    if creature_weights:
        creature.weights = creature_weights
    
    # Place food randomly
    food_positions = []
    for _ in range(5):
        fx = random.uniform(-15, 15)
        fy = random.uniform(-8, 8)
        food_positions.append((fx, fy))
    
    # Run simulation
    steps = 0
    max_steps = 1000
    
    if visualize:
        print("Starting simulation...")
        print(f"Initial food positions: {food_positions}")
    
    while creature.is_alive() and steps < max_steps:
        # Update creature
        eaten_food = creature.update(food_positions)
        
        # Replace eaten food
        if eaten_food is not None:
            fx = random.uniform(-15, 15)
            fy = random.uniform(-8, 8)
            food_positions[eaten_food] = (fx, fy)
            if visualize:
                print(f"Food eaten! New food at ({fx:.1f}, {fy:.1f})")
        
        # Step physics
        world.step(1.0/30.0)
        
        # Print status occasionally
        if visualize and steps % 100 == 0:
            pos = world.get_position(creature.body)
            if pos:
                print(f"Step {steps}: Creature at ({pos[0]:.1f}, {pos[1]:.1f}), "
                      f"Energy: {creature.energy:.1f}, Fitness: {creature.fitness}")
        
        steps += 1
    
    if visualize:
        print(f"\nSimulation ended after {steps} steps")
        print(f"Final fitness: {creature.fitness}")
        print(f"Reason: {'Out of energy' if creature.energy <= 0 else 'Time limit'}")
    
    return creature.fitness


def evolve_creatures(generations=20, population_size=50):
    """Simple genetic algorithm to evolve creature behaviors."""
    
    print("=== Evolution Training with Evolve Physics ===\n")
    
    # Initialize population with random weights
    population = []
    for _ in range(population_size):
        weights = [random.uniform(-1, 1) for _ in range(6)]
        population.append(weights)
    
    best_fitness_history = []
    
    for gen in range(generations):
        # Evaluate fitness for each creature
        fitness_scores = []
        
        print(f"\nGeneration {gen + 1}/{generations}")
        
        for i, weights in enumerate(population):
            fitness = run_simulation(weights, visualize=False)
            fitness_scores.append(fitness)
            
            if i % 10 == 0:
                print(f"  Evaluated {i+1}/{population_size} creatures...", end='\r')
        
        print(f"  Evaluated {population_size}/{population_size} creatures")
        
        # Find best performers
        sorted_pop = sorted(zip(fitness_scores, population), reverse=True)
        best_fitness = sorted_pop[0][0]
        best_fitness_history.append(best_fitness)
        
        avg_fitness = sum(fitness_scores) / len(fitness_scores)
        print(f"  Best fitness: {best_fitness:.1f}")
        print(f"  Average fitness: {avg_fitness:.1f}")
        
        # Select top performers for breeding
        elite_count = population_size // 4
        elite = [weights for _, weights in sorted_pop[:elite_count]]
        
        # Create new population
        new_population = elite[:]  # Keep elite
        
        # Breed new creatures
        while len(new_population) < population_size:
            # Select two parents
            parent1 = random.choice(elite)
            parent2 = random.choice(elite)
            
            # Crossover
            child = []
            for i in range(len(parent1)):
                if random.random() < 0.5:
                    child.append(parent1[i])
                else:
                    child.append(parent2[i])
            
            # Mutation
            for i in range(len(child)):
                if random.random() < 0.1:  # 10% mutation rate
                    child[i] += random.uniform(-0.3, 0.3)
                    child[i] = max(-1, min(1, child[i]))  # Clamp to [-1, 1]
            
            new_population.append(child)
        
        population = new_population
    
    print("\n=== Evolution Complete ===")
    print(f"Best fitness progression: {best_fitness_history}")
    
    # Run best creature with visualization
    print("\nRunning best creature with visualization...")
    best_weights = sorted_pop[0][1]
    run_simulation(best_weights, visualize=True)


if __name__ == "__main__":
    # Check if module is available
    try:
        print(f"Evolve Physics version: {evolve_physics.__version__}")
    except AttributeError:
        print("Module loaded successfully")
    
    # Run evolution
    evolve_creatures(generations=10, population_size=20)