import carla
import random
import time

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)  # Avoid connection timeout issues
world = client.get_world()

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True  # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# Set up the Traffic Manager (TM) in synchronous mode
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)

# Set a seed for reproducibility
traffic_manager.set_random_device_seed(0)
random.seed(0)

# Get spectator for visualization
spectator = world.get_spectator()

# Get all available spawn points
spawn_points = world.get_map().get_spawn_points()

# Draw spawn point indices for debugging
for i, spawn_point in enumerate(spawn_points):
    world.debug.draw_string(spawn_point.location, str(i), life_time=10)

# Select vehicle models
models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
blueprints = [v for v in world.get_blueprint_library().filter('*vehicle*') if any(model in v.id for model in models)]

# Define routes
route_1_indices = [129, 28, 124, 33, 97, 119, 58, 154, 147]
route_2_indices = [21, 76, 38, 34, 90, 3]
route_1 = [spawn_points[i].location for i in route_1_indices]
route_2 = [spawn_points[i].location for i in route_2_indices]

# Highlight routes for visualization
for i in route_1_indices:
    world.debug.draw_string(spawn_points[i].location, str(i), life_time=60, color=carla.Color(255, 0, 0))
for i in route_2_indices:
    world.debug.draw_string(spawn_points[i].location, str(i), life_time=60, color=carla.Color(0, 0, 255))

# Initialize vehicle spawning parameters
spawn_delay = 20
counter = spawn_delay
max_vehicles = 200
alt = False  # To alternate between spawn points
vehicles = []  # List to store spawned vehicles

# Main simulation loop
try:
    while True:
        world.tick()

        # Check number of vehicles in the world
        n_vehicles = len(world.get_actors().filter('*vehicle*'))

        # Spawn vehicles if conditions are met
        if counter == spawn_delay and n_vehicles < max_vehicles:
            vehicle_bp = random.choice(blueprints)
            spawn_point = spawn_points[32] if alt else spawn_points[149]
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

            if vehicle:  # If vehicle is successfully spawned
                vehicle.set_autopilot(True)
                traffic_manager.update_vehicle_lights(vehicle, True)
                traffic_manager.random_left_lanechange_percentage(vehicle, 0)
                traffic_manager.random_right_lanechange_percentage(vehicle, 0)
                traffic_manager.auto_lane_change(vehicle, False)

                # Assign route
                traffic_manager.set_path(vehicle, route_1 if alt else route_2)
                vehicles.append(vehicle)
                alt = not alt  # Toggle between routes

            counter = 0  # Reset counter
        else:
            counter += 1

        # Sleep to match the simulation tick rate
        time.sleep(settings.fixed_delta_seconds)

finally:
    print("Cleaning up vehicles...")
    for vehicle in vehicles:
        vehicle.destroy()
    world.apply_settings(carla.WorldSettings(synchronous_mode=False))  # Reset to asynchronous mode
    print("Done!")

