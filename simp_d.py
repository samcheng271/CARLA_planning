import math
import heapq
import random
import numpy as np
import carla
import logging

logging.basicConfig(
    filename='dstar.log',
    level=logging.INFO,
    format='%(levelname)s - %(funcName)s - %(message)s - %(lineno)d'
)

class DStar:
    def __init__(self, start_wp, goal_wp, vehicle, world, resolution=2.0):
        self.start = start_wp
        self.goal = goal_wp
        self.vehicle = vehicle
        self.world = world
        self.resolution = resolution
        self.h = {}
        self.parent = {}
        self.tag = {}
        self.OPEN = []
        self.xt = self.goal

    def cost(self, wp1, wp2):
        return wp1.transform.location.distance(wp2.transform.location)

    def get_neighbors(self, waypoint):
        neighbors = []
        next_wps = waypoint.next(self.resolution)
        if next_wps:
            neighbors.extend(next_wps)
        left_wp = waypoint.get_left_lane()
        if left_wp and left_wp.lane_type == carla.LaneType.Driving:
            neighbors.append(left_wp)
        right_wp = waypoint.get_right_lane()
        if right_wp and right_wp.lane_type == carla.LaneType.Driving:
            neighbors.append(right_wp)
        return neighbors

    def get_kmin(self):
        if self.OPEN: 
            return self.OPEN[0][0] 
        else: 
            float('inf')

    def process_state(self):
        if not self.OPEN:
            return float('inf')
        k_old, _, current_state = heapq.heappop(self.OPEN)
        current_h = self.h.get(current_state.id, float('inf'))

        if k_old < current_h:
            for neighbor in self.get_neighbors(current_state):
                new_cost = self.cost(current_state, neighbor)
                if self.h.get(neighbor.id, float('inf')) + new_cost < current_h:
                    current_h = self.h[neighbor.id] + new_cost
                    self.h[current_state.id] = current_h
                    self.parent[current_state.id] = neighbor
            heapq.heappush(self.OPEN, (current_h, current_state.id, current_state))
        else:
            self.tag[current_state.id] = 'Closed'
            for neighbor in self.get_neighbors(current_state):  
                new_cost = self.cost(current_state, neighbor) 
                new_cost = current_h + new_cost
                if new_cost < self.h.get(neighbor.id, float('inf')):
                    self.h[neighbor.id] = new_cost
                    self.parent[neighbor.id] = current_state
                    heapq.heappush(self.OPEN, (new_cost, neighbor.id, neighbor))
                    self.tag[neighbor.id] = 'Open'
        return self.get_kmin()


    def run(self):
        self.h = {}
        self.parent = {}
        self.tag = {}
        self.OPEN = []
        self.xt = self.goal

        self.h[self.goal.id] = 0
        self.tag[self.goal.id] = 'Open'
        heapq.heappush(self.OPEN, (0, self.goal.id, self.goal))

        while self.OPEN and (self.h.get(self.start.id, float('inf')) > self.get_kmin() or 
                            self.tag.get(self.start.id) != 'Closed'):
            self.process_state()

        if self.h.get(self.start.id, float('inf')) == float('inf'):
            print("No path found")
            return None
        path = self.reconstruct_path()
        if path:
            print("Path found")
            self.visualize_path(path)
        else:
            print("No path found")
        return path

    def reconstruct_path(self):
        if self.start.id not in self.parent:
            print("No path found from start")
            return None
        path = [self.start]
        current = self.start
        while current.id != self.goal.id:
            if current.id not in self.parent:
                print("Path reconstruction failed")
                return None
            current = self.parent[current.id]
            path.append(current)
        return path

    def visualize_path(self, path):
        if not path:
            return
        for i in range(len(path) - 1):
            wp1 = path[i].transform.location
            wp2 = path[i + 1].transform.location
            self.world.debug.draw_string(
                wp1, "1",
                draw_shadow=False,
                color=carla.Color(0, 255, 0),
                life_time=15.0
            )
            self.world.debug.draw_string(
                wp2, "2",
                draw_shadow=False,
                color=carla.Color(0, 255, 0),
                life_time=15.0
            )
        start_loc = path[0].transform.location
        goal_loc  = path[-1].transform.location
        self.world.debug.draw_string(
            start_loc, "START",
            draw_shadow=False, color=carla.Color(0, 255, 0),
            life_time=15.0
        )
        self.world.debug.draw_string(
            goal_loc, "GOAL",
            draw_shadow=False, color=carla.Color(0, 0, 255),
            life_time=15.0
        )

    def move_vehicle(self, path):
        if not path:
            return
        for wp in path:
            self.vehicle.set_transform(wp.transform)
            self.world.wait_for_tick()

if __name__ == '__main__':
    try:
        client = carla.Client('localhost', 4000)
        client.set_timeout(10.0)
        world = client.get_world()
        carla_map = world.get_map()
        spawn_points = carla_map.get_spawn_points()
        point_a = random.choice(spawn_points)
        point_b = random.choice(spawn_points)
        while point_b.location.distance(point_a.location) < 10.0:
            point_b = random.choice(spawn_points)
        blueprint_library = world.get_blueprint_library()
        firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        firetruck = world.spawn_actor(firetruck_bp, point_a)
        start_wp = carla_map.get_waypoint(point_a.location, project_to_road=True)
        goal_wp = carla_map.get_waypoint(point_b.location, project_to_road=True)
        world.debug.draw_string(
            start_wp.transform.location, "START",
            draw_shadow=False, color=carla.Color(255, 0, 0),
            life_time=30.0, persistent_lines=True
        )
        world.debug.draw_string(
            goal_wp.transform.location, "GOAL",
            draw_shadow=False, color=carla.Color(255, 0, 0),
            life_time=30.0, persistent_lines=True
        )
        dstar = DStar(start_wp, goal_wp, firetruck, world, resolution=2.0)
        path = dstar.run()
        if path:
            print("Moving vehicle along the planned path")
            dstar.move_vehicle(path)
        firetruck.destroy()
    except Exception as e:
        print("Error during D* planning or execution:", e)