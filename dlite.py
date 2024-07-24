import carla
import random
import time
from queue import PriorityQueue

class DStarLite:
    def __init__(self, world, start_waypoint, end_waypoint):
        self.world = world
        self.start = start_waypoint
        self.goal = end_waypoint
        self.U = PriorityQueue()
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.epsilon = 1

    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)

    def calculate_key(self, s):
        if s not in self.g:
            self.g[s] = float('inf')
        if s not in self.rhs:
            self.rhs[s] = float('inf')
        return [
            min(self.g[s], self.rhs[s]) + self.epsilon * self.heuristic(s, self.start) + self.km,
            min(self.g[s], self.rhs[s])
        ]

    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        for s in self.world.get_map().generate_waypoints(2.0):
            self.rhs[s] = float('inf')
            self.g[s] = float('inf')
        self.rhs[self.goal] = 0
        self.U.put((self.calculate_key(self.goal), self.goal))











    def main():
        client = carla.Client('localhost', 4000)
        client.set_timeout(10.0)

        world = client.get_world()
        carla_map = world.get_map()

        spawn_points = carla_map.get_spawn_points()
        start_transform = random.choice(spawn_points)
        end_transform = random.choice(spawn_points)

        start_waypoint = carla_map.get_waypoint(start_transform.location)
        end_waypoint = carla_map.get_waypoint(end_transform.location)

        # Get the route
        # route = DStarLite_route(world, start_waypoint, end_waypoint)

        # Draws the route the vehicle will follow (red)
        # for waypoint in route:
        #     world.debug.draw_string(waypoint.transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
        
        # for waypoint in route:
        #     vehicle.set_transform(waypoint.transform)
        #     time.sleep(0.05)

