
import math
import heapq
import random
import numpy as np
import carla


class DStar:
    def __init__(self, start_wp, goal_wp, vehicle, world, resolution=2.0):
        self.start = start_wp
        self.goal = goal_wp
        self.vehicle = vehicle
        self.world = world
        self.resolution = resolution
        self.h = {}
        self.parent = {}
        self.OPEN = []
       

    def cost(self, wp1, wp2):
       
        loc1 = wp1.transform.location
        loc2 = wp2.transform.location
        return loc1.distance(loc2)

    
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

    def run(self):
        self.h.clear()
        self.parent.clear()
        self.OPEN.clear()

        self.h[self.start.id] = 0
        heapq.heappush(self.OPEN, (0, self.start.id, self.start))

        while self.OPEN:
            current_cost, _, current = heapq.heappop(self.OPEN)
            if current.id == self.goal.id:
                break
            #get x,y coor and print heuristics around the backtrace 
            for neighbor in self.get_neighbors(current):
                new_cost = self.h[current.id] + self.cost(current, neighbor)
                if new_cost < self.h.get(neighbor.id, float('inf')):
                    self.h[neighbor.id] = new_cost
                    self.parent[neighbor.id] = current
                    heapq.heappush(self.OPEN, (new_cost, neighbor.id, neighbor))

        path = self.reconstruct_path()
        if path:
            print("Path found!")
            self.visualize_path(path)
        else:
            print("No path found!")
        return path

    def reconstruct_path(self):
        if self.goal.id not in self.h:
            print("No path found.")
            return None

        path = []
        current = self.goal
        while current.id != self.start.id:
            path.append(current)
            if current.id not in self.parent:
                print(f"Path reconstruction failed at waypoint {current.id}.")
                return None
            current = self.parent[current.id]
        path.append(self.start)
        path.reverse()
        return path

    def visualize_path(self, path):
        if not path:
            return

        for i in range(len(path) - 1):
            wp1 = path[i].transform.location
            wp2 = path[i + 1].transform.location
            self.world.debug.draw_line(
                wp1, wp2,
                thickness=0.2,
                color=carla.Color(255, 0, 0), 
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

        # Clean up.
        firetruck.destroy()

    except Exception as e:
        print("Error during D* planning or execution:", e)
