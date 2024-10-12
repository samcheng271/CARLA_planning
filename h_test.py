import carla
import random
import math
import numpy as np
from queue import PriorityQueue

class CarlaPathPlanner:
    def __init__(self, waypoint, start_waypoint, end_waypoint, vehicle, world, carla_map, resolution=2.0):
        self.settings = 'CollisionChecking'
        self.resolution = resolution
        self.waypoint = waypoint
        self.obstacle_threshold = 3.0
        self.b = {}
        self.OPEN = PriorityQueue()

        self.tag = {}
        self.V = set()
        self.parameters = ()
        self.ind = 0
        self.Path = []
        self.done = False
        self.Obstaclemap = {}
        self.world = world
        self.map = carla_map
        self.vehicle = vehicle
        self.state = self.waypoint
        self.location = self.vehicle.get_location()
        self.state_space = self.map.get_waypoint(self.location, project_to_road=True)
        self.waypoints = self.map.generate_waypoints(self.resolution)
        print(f"Number of waypoints generated: {len(self.waypoints)}")
        self.h = {}
        self.next_waypoint = None
        self.x0 = start_waypoint
        print(f'Start waypoint: {self.x0.transform.location}')
        self.xt = end_waypoint
        print(f'End waypoint: {self.xt.transform.location}')

    def store_h(self, state):
        # Euclidean distance between 3D points
        if state is None:
            return float('inf')

        if state.id not in self.h:
            x = self.xt.transform.location.x - state.transform.location.x
            y = self.xt.transform.location.y - state.transform.location.y
            z = self.xt.transform.location.z - state.transform.location.z

            print(f"State: Location({self.state.transform.location.x}, {self.state.transform.location.y}, {self.state.transform.location.z})")
            print(f"Self.xt: Location({self.xt.transform.location.x}, {self.xt.transform.location.y}, {self.xt.transform.location.z})")
            print(f"x: {float(self.xt.transform.location.x) - float(state.transform.location.x)}")
            print(f"y: {self.xt.transform.location.y - state.transform.location.y}")
            print(f"random: {math.sqrt(x**2 + y**2)}")

            #norm = math.sqrt(x**2 + y**2 + z**2) + np.finfo(float).eps  # To avoid division by zero
            self.h[state.id] = norm

        return self.h[state.id]

if __name__ == '__main__':
    # Connect to the simulator
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # Get the world and map
    world = client.get_world()
    carla_map = world.get_map()

    # Get a firetruck blueprint and spawn it at a random location
    blueprint_library = world.get_blueprint_library()
    firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
    spawn_points = carla_map.get_spawn_points()

    # Choose random starting and destination points
    point_a = random.choice(spawn_points)
    point_b = random.choice(spawn_points)
    
    # Ensure point A and point B are different
    while point_b.location == point_a.location:
        point_b = random.choice(spawn_points)

    firetruck = world.spawn_actor(firetruck_bp, point_a)

    # Get the start and end waypoints
    start_waypoint = carla_map.get_waypoint(point_a.location)
    end_waypoint = carla_map.get_waypoint(point_b.location)

    print(f"Start waypoint: {start_waypoint.transform.location}")
    print(f"End waypoint: {end_waypoint.transform.location}")
    
    # Initialize the CarlaPathPlanner
    waypoint = start_waypoint  # Can be any waypoint; here we use the starting waypoint
    planner = CarlaPathPlanner(waypoint, start_waypoint, end_waypoint, firetruck, world, carla_map)

    # Optionally store the heuristic for the start waypoint
    planner.store_h(start_waypoint)


