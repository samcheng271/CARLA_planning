
import math
import heapq
import random
#from runpy import run_path
import numpy as np
import carla
import logging
from collections import deque

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
        self.all_obst_wps   = {}
        self.new_obst_wps   = {}
        self.edge_cost      = {}
        self.h              = {}
        self.parent         = {}
        self.tag            = {}
        self.OPEN           = []
        self.all_waypoints = {}
        self.blocked_nodes = set()   
        self.xt             = self.goal
        self.s_current      = self.start
        #self.obstacle = 0
        self.add_obs = {}
        self.all_obs = {}

    def waypoint_location(self, waypoint):
        location = waypoint.transform.location
        return ((location.x), (location.y), (location.z), waypoint.lane_id)

    def cost(self, wp1, wp2):
        key = (self.waypoint_location(wp1), self.waypoint_location(wp2))
        if key in self.edge_cost:
            return self.edge_cost[key]
        key_cost = wp1.transform.location.distance(wp2.transform.location)
        #print(f"  Waypoint 1: Location({wp1.transform.location.x}, {wp1.transform.location.y}, {wp1.transform.location.z})")
        #print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        #print(f"cost: {key_cost}") 
        return key_cost

    def get_neighbors(self, waypoint):
        neighbors = []
        prev_wps = waypoint.previous(self.resolution)
        if prev_wps:
            neighbors.extend(prev_wps)
        '''
        next_wps = waypoint.next(self.resolution)
        if next_wps:
            neighbors.extend(next_wps)
        '''
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
    
    '''
    def delete(self):
        currKey = self.waypoint_location(self.s_current)
        self.tag[currKey] = "Closed"
    '''

    def process_state(self):
        if not self.OPEN:
            return float("inf")
        kOld, _, currState = heapq.heappop(self.OPEN)
        self.s_current = currState

        currKey   = self.waypoint_location(currState)
        current_h = self.h.get(currKey, float("inf"))

        if kOld < current_h:
            for neighbor in self.get_neighbors(currState):
                newKey    = self.waypoint_location(neighbor)
                newCost   = self.cost(currState, neighbor)     
                heuristic = self.h.get(newKey, float("inf"))      
                if (self.tag.get(newKey) != "New" and heuristic <= kOld
                    and current_h > heuristic + newCost):
                    self.parent[currKey] = neighbor
                    self.h[currKey]      = heuristic + newCost
            heapq.heappush(self.OPEN, (self.h[currKey], repr(currState), currState))
        elif kOld == current_h:
            self.tag[currKey] = "Closed"
            for neighbor in self.get_neighbors(currState):
                newKey  = self.waypoint_location(neighbor)
                newCost = current_h + self.cost(currState, neighbor)
                if (self.tag.get(newKey) == "New"
                    or (self.parent.get(newKey) == currState
                        and self.h.get(newKey, float("inf")) != newCost)
                    or (self.parent.get(newKey) != currState
                        and self.h.get(newKey, float("inf")) > newCost)):
                    self.parent[newKey] = currState
                    self.h[newKey] = newCost
                    heapq.heappush(self.OPEN, (newCost, repr(neighbor), neighbor))
                    self.tag[newKey] = "Open"
        else:
            for neighbor in self.get_neighbors(currState):
                newKey  = self.waypoint_location(neighbor)
                newCost = self.cost(currState, neighbor)
                if (self.tag.get(newKey) == "New" or 
                   (self.parent.get(newKey) == currState and self.h.get(newKey, float("inf")) != current_h + newCost)):
                    self.parent[newKey] = currState
                    self.h[newKey] = current_h + newCost
                    heapq.heappush(self.OPEN, (self.h[newKey], repr(neighbor), neighbor))
                    self.tag[newKey] = "Open"
                elif (self.parent.get(newKey) != currState
                    and self.h.get(newKey, float("inf"))
                        > current_h + newCost
                    and self.tag.get(newKey) == "Closed"):
                    heapq.heappush(self.OPEN, (self.h.get(newKey), repr(neighbor), neighbor))
                    self.tag[newKey] = "Open"
        return self.get_kmin()

    def modify_cost(self, wp1, wp2, cost):
        edgeKey = (self.waypoint_location(wp1), self.waypoint_location(wp2))
        self.edge_cost[edgeKey] = cost
        for wp in (wp1, wp2):
            key = self.waypoint_location(wp)
            if self.tag.get(key) == "Closed":
                k = self.h.get(key, float("inf"))
                heapq.heappush(self.OPEN, (k, id(wp), wp))
                self.tag[key] = "Open"
        return self.get_kmin()
    
    def run(self):
        self.h.clear()
        self.parent.clear()
        self.tag.clear()
        self.OPEN.clear()
        self.s_current = self.goal

        goal_key  = self.waypoint_location(self.goal)
        start_key = self.waypoint_location(self.start)

        self.h[goal_key]   = 0.0
        self.tag[goal_key] = "Open"
        heapq.heappush(self.OPEN, (0.0, goal_key, self.goal))
        val = 0.0
        while self.tag.get(start_key) != "Closed" and val != float("inf"):
            val = self.process_state()
            #self.handle_obstacles()
        if val == float("inf"):
            print("No path found")
            return None
        path = self.reconstruct_path()
        if path:
            self.visualize_path(path)
            return path
        else:
            print("No path found")
            return None

    '''
    def detect_obstacles(self, state_space):
        child_waypoints = self.get_neighbors(state_space)
        for child in child_waypoints:
            child_location = child.transform.location
            for actor in self.world.get_actors():
                if actor.id != self.vehicle.id:
                    actor_location = actor.get_transform().location
                    distance = actor_location.distance(child_location)
                    if distance < 5.0:
                        return True, actor
        return False, None 
    '''
    '''
    def handle_obstacles(self):
        true_obstacles, true_actor = self.detect_obstacles(self.s_current)
        if true_obstacles:
            actor_location = true_actor.get_transform().location 
            print(f"actor: {actor_location}")
            get_obstacle = self.map.get_waypoint(actor_location)
            print(f"obs location: {get_obstacle.transform.location}")
            obs_cost = 99999999
            print(f"cost done")
            child_waypoints = self.get_neighbors(self.s_current)
            print("1")
            
            print("2")
            prev_list = get_obstacle.previous(self.resolution)
            if prev_list:
                for prev_wp in prev_list:
                    print("3")
                    self.modify_cost(get_obstacle, prev_wp, obs_cost)
                    self.modify_cost(prev_wp, get_obstacle, obs_cost)
                    print("4")
            next_list = get_obstacle.next(self.resolution)
            if next_list:
                for prev_wp in next_list:
                    print("3")
                    self.modify_cost(get_obstacle, prev_wp, obs_cost)
                    self.modify_cost(prev_wp, get_obstacle, obs_cost)
                    print("4")
        else:
            print("No obstacles detected")
        '''

    def reconstruct_path(self):
        start_key = self.waypoint_location(self.start)
        goal_key  = self.waypoint_location(self.goal)
        if start_key not in self.parent:
            #print("No path found from start")
            return None
        path = [self.start]
        current = self.start
        while self.waypoint_location(current) != goal_key:
            current_key = self.waypoint_location(current)
            if current_key not in self.parent:
                #print("Path reconstruction failed")
                return None
            next_wp = self.parent[current_key]
            path.append(next_wp)
            current = next_wp
        return path

    def visualize_path(self, path):
        if not path:
            return
        for i in range(len(path) - 1):
            p1 = path[i].transform.location
            p2 = path[i + 1].transform.location
            self.world.debug.draw_string(p1, "^", draw_shadow=False,
                                         color=carla.Color(0, 255, 0),
                                         life_time=15.0)
            self.world.debug.draw_string(p2, "^", draw_shadow=False,
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

    def print_path(self, path, tag="path"):
        if not path:
            return
        coords = [
            (round(wp.transform.location.x, 2),
             round(wp.transform.location.y, 2),
             round(wp.transform.location.z, 2))
            for wp in path
        ]
        print(f"{tag} ({len(coords)} waypoints):\n{coords}\n")

if __name__ == "__main__":
    try:
        client        = carla.Client("localhost", 4000)
        client.set_timeout(10.0)
        world         = client.get_world()
        carla_map     = world.get_map()
        blueprint_library = world.get_blueprint_library()
        firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        spawn_points = carla_map.get_spawn_points()

        '''
        point_a = random.choice(spawn_points)
        firetruck = world.spawn_actor(firetruck_bp, point_a)
        # Choose a random destination (point B)
        point_b = random.choice(spawn_points)
        if point_b.location == point_a.location:
            point_b = random.choice(spawn_points)
        '''
        start_ind = 25
        end_ind = 35
        # Choose a random starting location (point A)
        #point_a = random.choice(spawn_points)
        point_a = spawn_points[start_ind]
        firetruck = world.spawn_actor(firetruck_bp, point_a)

        # Choose a random destination (point B)
        point_b  = spawn_points[end_ind]
        #point_b = random.choice(spawn_points)
        while point_b.location == point_a.location:
            point_b = random.choice(spawn_points)
        '''
        start_waypoint = carla_map.get_waypoint(point_a.location)
        end_waypoint = carla_map.get_waypoint(point_b.location)
        
        obs1_ind = 20
        obs_pt1 = spawn_pts[obs1_ind]
        obs1_bp = bp_lib.filter("vehicle.chevrolet.impala")[0]
        chevrolet = world.spawn_actor(obs1_bp, obs_pt1)

        obs2_ind = 25
        obs_pt2 = spawn_pts[obs2_ind]
        obs2_bp = bp_lib.filter("vehicle.dodge.charger_police")[0]
        dodge = world.spawn_actor(obs2_bp, obs_pt2)
        '''
        
        start_wp = carla_map.get_waypoint(point_a.location, project_to_road=True)
        goal_wp  = carla_map.get_waypoint(point_b.location, project_to_road=True)

        world.debug.draw_string(start_wp.transform.location, "START",
                                draw_shadow=False, color=carla.Color(255, 0, 0),
                                life_time=30.0, persistent_lines=True)
        world.debug.draw_string(goal_wp.transform.location, "GOAL",
                                draw_shadow=False, color=carla.Color(255, 0, 0),
                                life_time=30.0, persistent_lines=True)
        planner = DStar(start_wp, goal_wp, firetruck, world, resolution=1.0)
        print("1")
        path = planner.run()
        #print(f"path: {path}")
        #planner.print_path(path, "initial")

        #obs_indices = [5, 10] 
        if path is None:
            print("Initial planning failed—exiting.")
            #sys.exit()
        print("All waypoints in initial path:")
        for idx, wp in enumerate(path):
            loc = wp.transform.location
            print(f"  {idx:03d}: (x={loc.x}, y={loc.y}, z={loc.z})")

        planner.print_path(path)
        print("planner print done")
        planner.edge_cost.clear()
        obs_indices = [200]
        print(f"obs_indices: {obs_indices}")
        obs_cost = 99999999
        for i in obs_indices:
            print("new obstacles vehicles")
            if i < len(path):
                print("len path")
                wp = path[i]
                print("wp")
                neighbor_list = planner.get_neighbors(wp)
                if neighbor_list:
                    for n_wp in neighbor_list:
                        planner.modify_cost(wp, n_wp, obs_cost)
                        planner.modify_cost(n_wp, wp, obs_cost)
        print("before new path")
        new_path = planner.run()
        print(f"new path: {new_path}")
        
        if new_path is None: 
            print("new path failed")
        else:
            print("All waypoints in new path:")
            for idx, wp in enumerate(new_path):
                loc = wp.transform.location
                #print(f"  {idx:03d}: (x={loc.x}, y={loc.y}, z={loc.z})")
            planner.print_path(new_path)
            print("planner new path print done")
            planner.visualize_path(new_path)
            print("new path visualization done")
            path = new_path

        '''
        for i, bp_name in zip(obs_indices, ("vehicle.chevrolet.impala",
                                            "vehicle.dodge.charger_police")):
            wp = path[i]                              # a carla.Waypoint
            bp = bp_lib.filter(bp_name)[0]
            actor = world.spawn_actor(bp, wp.transform)
            # optionally draw a label so you can verify where you spawned it:
            world.debug.draw_string(
                wp.transform.location, 
                f"Obs{i}", 
                draw_shadow=False, 
                color=carla.Color(0,255,0), 
                life_time=30.0, 
                persistent_lines=True
            )
        
        idx = 5
        if idx < len(path):
            #obstacle_wp = path[idx]
            #handle manual obstacle
            #planner.manual_obstacle(obstacle_wp)
            path = planner.reconstruct_path()
            if path:
                planner.visualize_path(path)
            else:
                print("No path exists after blocking that waypoint.")
        '''
        '''
        i = 120 
        if i > 0 and i < len(path):
            k = frozenset((planner.waypoint_location(path[i-1]), planner.waypoint_location(path[i])))
            print("blocked edge cost:", planner.edge_cost.get(k))
        '''

        if path:
            u, v = path[3], path[4]
            planner.modify_cost(u, v, float("inf"))
            #print("Replanning")
            path = planner.reconstruct_path()
            #planner.print_path(path, "replan")
            planner.visualize_path(path)

        if path:
            #print("Moving vehicle along the planned path")
            planner.move_vehicle(path)
        firetruck.destroy()

    except Exception as exc:
        print("Error during D* planning or execution:", exc)
