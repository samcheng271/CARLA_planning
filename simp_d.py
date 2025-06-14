import math
import heapq
import random
import numpy as np
import carla
import logging

"""
logging.basicConfig(
    filename='dstar.log',
    level=logging.INFO,
    format='%(levelname)s - %(funcName)s - %(message)s - %(lineno)d'
)
"""

class DStar:
    def __init__(self, start_wp, goal_wp, vehicle, world, resolution: float = 1.0):
        self.start          = start_wp
        self.goal           = goal_wp
        self.vehicle        = vehicle
        self.world          = world
        self.resolution     = resolution
        self.map            = world.get_map()
        #self.all_waypoints  = self.map.generate_waypoints(self.resolution)
        self.all_obst_wps   = {}
        self.new_obst_wps   = {}
        self.edge_cost      = {}
        self.h              = {}
        self.parent         = {}
        self.tag            = {}
        self.OPEN           = []
        self.xt             = self.goal
        self.s_current      = self.start

    @staticmethod
    def round_tuple(val: float, ndigits: int = 3):
        return round(val, ndigits)

    def waypoint_rep(self, waypoint):
        location = waypoint.transform.location
        return (
            self.round_tuple(location.x),
            self.round_tuple(location.y),
            self.round_tuple(location.z),
            waypoint.lane_id
        )

    def cost(self, wp1, wp2):
        key = frozenset((id(wp1), id(wp2)))
        if key in self.edge_cost:
            return self.edge_cost[key]
        key_cost = wp1.transform.location.distance(wp2.transform.location)
        print(f"  Waypoint 1: Location({wp1.transform.location.x}, {wp1.transform.location.y}, {wp1.transform.location.z})")
        print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        print(f"cost: {key_cost}") 
        return key_cost

    def get_neighbors(self, waypoint):
        neighbors = []
        prev_wps = waypoint.previous(self.resolution)
        if prev_wps:
            neighbors.extend(prev_wps)
        if waypoint.lane_change & carla.LaneChange.Left:
            left_wp = waypoint.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                neighbors.append(left_wp)
        if waypoint.lane_change & carla.LaneChange.Right:
            right_wp = waypoint.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                neighbors.append(right_wp)
        if waypoint.transform.location.distance(self.start.transform.location) < 1.5:
            neighbors.append(self.start)
        return neighbors

    def get_kmin(self):
        return self.OPEN[0][0] if self.OPEN else float("inf")

    def process_state(self):
        if not self.OPEN:
            return float("inf")
        kOld, _, currState = heapq.heappop(self.OPEN)
        self.s_current = currState

        currKey   = self.waypoint_rep(currState)
        current_h = self.h.get(currKey, float("inf"))

        if kOld < current_h:
            for neighbor in self.get_neighbors(currState):
                newKey    = self.waypoint_rep(neighbor)
                newCost   = self.cost(currState, neighbor)
                heuristic = self.h.get(newKey, float("inf"))
                if heuristic + newCost < current_h:
                    self.h[currKey]       = heuristic + newCost
                    self.parent[currKey]  = neighbor
            heapq.heappush(self.OPEN, (self.h[currKey], currKey, currState))

        elif kOld == current_h:
            self.tag[currKey] = "Closed"
            for neighbor in self.get_neighbors(currState):
                newKey  = self.waypoint_rep(neighbor)
                newCost = current_h + self.cost(currState, neighbor)
                if newCost < self.h.get(newKey, float("inf")):
                    self.h[newKey]        = newCost
                    self.parent[newKey]   = currState
                    heapq.heappush(self.OPEN, (newCost, newKey, neighbor))
                    self.tag[newKey]      = "Open"

        else:
            for neighbor in self.get_neighbors(currState):
                newKey    = self.waypoint_rep(neighbor)
                newCost   = self.cost(currState, neighbor)
                heuristic = self.h.get(newKey, float("inf"))
                if current_h + newCost < heuristic:
                    self.h[newKey]        = current_h + newCost
                    self.parent[newKey]   = currState
                    heapq.heappush(self.OPEN, (self.h[newKey], newKey, neighbor))
                    self.tag[newKey]      = "Open"

        return self.get_kmin()

    def modify_cost(self, wp1, wp2, inputCost):
        edgeKey = frozenset((id(wp1), id(wp2)))
        self.edge_cost[edgeKey] = inputCost
        for wp in (wp1, wp2):
            key = self.waypoint_rep(wp)
            heuristic = self.h.get(key, float("inf"))
            heapq.heappush(self.OPEN, (heuristic, key, wp))
            self.tag[key] = "Open"

        startKey = self.waypoint_rep(self.start)
        while (self.OPEN and
               (self.h.get(startKey, float("inf")) > self.get_kmin() or
                self.tag.get(startKey) != "Closed")):
            self.process_state()

    '''
    def add_obst_loc_to_obst_wp(self, location):
        wp = self.map.get_waypoint(location)
        shortest_dist = float("inf")
        for z in self.all_waypoints:
            dist = wp.transform.location.distance(z.transform.location)
            if dist < shortest_dist:
                gen_wp        = z
                shortest_dist = dist
        self.all_obst_wps[gen_wp.id] = gen_wp
        self.new_obst_wps[gen_wp.id] = gen_wp

    def obs_signal(self, location):
        self.add_obst_loc_to_obst_wp(location)

    '''

    '''
    def add_obstacles(self, location):
            return 
    '''
    def run(self):
        self.h.clear()
        self.parent.clear()
        self.tag.clear()
        self.OPEN.clear()

        goal_key  = self.waypoint_rep(self.goal)
        start_key = self.waypoint_rep(self.start)

        self.h[goal_key]   = 0.0
        self.tag[goal_key] = "Open"
        heapq.heappush(self.OPEN, (0.0, goal_key, self.goal))

        '''
        counter = 0
        k = 20
        m = 5
        '''
        while (self.OPEN and
            (self.h.get(start_key, float("inf")) > self.get_kmin() or
                self.tag.get(start_key) != "Closed")):
            self.process_state()
            '''
            counter += 1

            if counter % k == 0:
                obs_wp = self.s_current
                for _ in range(m):
                    prev_list = obs_wp.previous(self.resolution)
                    if not prev_list:
                        break
                    obs_wp = prev_list[0]

                self.new_obst_wps[obs_wp.id] = obs_wp
                for nb in self.get_neighbors(obs_wp):
                    self.modify_cost(obs_wp, nb, float("inf"))

                loc = obs_wp.transform.location
                self.world.debug.draw_string(
                    loc, "X", draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0),
                    life_time=10.0
                )
            '''
        if self.h.get(start_key, float("inf")) == float("inf"):
            print("No path found")
            return None
        self.new_obst_wps = {}
        path = self.reconstruct_path()
        if path:
            self.visualize_path(path)
        else:
            print("No path found")
        return path
    
    '''
    def manual_obstacle(self, obs_wp):
        if obs_wp.id in self.all_obst_wps:
            return
        self.all_obst_wps[obs_wp.id] = obs_wp
        for nb in self.get_neighbors(obs_wp):
            self.modify_cost(obs_wp, nb, float("inf"))
    '''

    def reconstruct_path(self):
        start_key = self.waypoint_rep(self.start)
        goal_key  = self.waypoint_rep(self.goal)
        if start_key not in self.parent:
            print("No path found from start")
            return None

        path = [self.start]
        current = self.start
        while self.waypoint_rep(current) != goal_key:
            c_key = self.waypoint_rep(current)
            if c_key not in self.parent:
                print("Path reconstruction failed")
                return None
            next_wp = self.parent[c_key]
            path.append(next_wp)
            current = next_wp
        return path

    def visualize_path(self, path):
        if not path:
            return
        for i in range(len(path) - 1):
            p1 = path[i].transform.location
            p2 = path[i + 1].transform.location
            self.world.debug.draw_string(p1, "1", draw_shadow=False,
                                         color=carla.Color(0, 255, 0),
                                         life_time=15.0)
            self.world.debug.draw_string(p2, "2", draw_shadow=False,
                                         color=carla.Color(0, 255, 0),
                                         life_time=15.0)
        self.world.debug.draw_string(path[0].transform.location, "START",
                                     draw_shadow=False,
                                     color=carla.Color(0, 255, 0),
                                     life_time=15.0)
        self.world.debug.draw_string(path[-1].transform.location, "GOAL",
                                     draw_shadow=False,
                                     color=carla.Color(0, 0, 255),
                                     life_time=15.0)

    def move_vehicle(self, path):
        if not path:
            return
        for wp in path:
            self.vehicle.set_transform(wp.transform)
            self.world.wait_for_tick()


if __name__ == "__main__":
    try:
        client        = carla.Client("localhost", 4000)
        client.set_timeout(10.0)
        world         = client.get_world()
        carla_map     = world.get_map()

        spawn_pts     = carla_map.get_spawn_points()
        point_a, point_b = random.sample(spawn_pts, 2)

        bp_lib        = world.get_blueprint_library()
        firetruck_bp  = bp_lib.filter("vehicle.carlamotors.firetruck")[0]
        firetruck     = world.spawn_actor(firetruck_bp, point_a)

        start_wp      = carla_map.get_waypoint(point_a.location, project_to_road=True)
        goal_wp       = carla_map.get_waypoint(point_b.location, project_to_road=True)

        world.debug.draw_string(start_wp.transform.location, "START",
                                draw_shadow=False, color=carla.Color(255, 0, 0),
                                life_time=30.0, persistent_lines=True)
        world.debug.draw_string(goal_wp.transform.location, "GOAL",
                                draw_shadow=False, color=carla.Color(255, 0, 0),
                                life_time=30.0, persistent_lines=True)

        planner = DStar(start_wp, goal_wp, firetruck, world, resolution=1.0)
        print("1")
        path = planner.run()
        print(f"path: {path}")

        idx = 5
        if idx < len(path):
            obstacle_wp = path[idx]
            #handle manual obstacle--
            planner.manual_obstacle(obstacle_wp)
            path = planner.reconstruct_path()
            if path:
                planner.visualize_path(path)
            else:
                print("No path exists after blocking that waypoint.")

        if path:
            u, v = path[3], path[4]
            planner.modify_cost(u, v, float("inf"))
            print("Replanning")
            path = planner.reconstruct_path()
            planner.visualize_path(path)

        if path:
            print("Moving vehicle along the planned path")
            planner.move_vehicle(path)

        firetruck.destroy()

    except Exception as exc:
        print("Error during D* planning or execution:", exc)

