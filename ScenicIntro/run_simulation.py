import scenic
from scenic.simulators.newtonian import NewtonianSimulator

def run_local_scenario(scenario_path: str, num_runs: int = 3):
    print(f"Loading and compiling {scenario_path}...")
    scenario = scenic.scenarioFromFile(scenario_path)

    # Initialize Scenic's native Newtonian simulator
    simulator = NewtonianSimulator(render=True)

    for i in range(num_runs):
        print(f"\n--- Running Scene Sample {i + 1}/{num_runs} ---")
        
        # Sample concrete parameters from distributions
        scene, _ = scenario.generate()
        
        print(f"Adversary spawn position: {scene.objects[1].position}")
        print(f"Obstacle spawn position:  {scene.objects[2].position}")

        # Execute dynamic simulation
        simulation = simulator.simulate(scene, maxSteps=120)
        
        if simulation:
            print(f"Run {i + 1} completed: {len(simulation.trajectory)} steps recorded.")
        else:
            print(f"Run {i + 1} terminated early.")

    simulator.destroy()

if __name__ == "__main__":
    run_local_scenario("scenarios/unprotected_left_turn.scenic")
    