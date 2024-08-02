import carla
import random
import numpy as np
from collections import defaultdict
from queue import PriorityQueue

class D_star(object):
    def __init__(self, waypoint, resolution=1):
        self.settings = 'CollisionChecking'

        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.waypoints = self.map.generate_waypoints(resolution)
        except Exception as e:
            print(f"Failed to connect to Carla: {e}")
            self.client = None
            self.world = None
            self.map = None
            self.waypoints = []
            return

        self.resolution = resolution
        self.state_space = self.convert_waypoints(self.waypoints)

        # Use actual coordinates from the CARLA map
        start_location = self.waypoints[0].transform.location
        goal_location = self.waypoints[-1].transform.location
        self.x0 = self.get_nearest_state(self.state_space, (start_location.x, start_location.y, start_location.z))
        self.xt = self.get_nearest_state(self.state_space, (goal_location.x, goal_location.y, goal_location.z))

        self.b = defaultdict(lambda: defaultdict(dict))
        self.OPEN = PriorityQueue()
        self.h = {}
        self.tag = {}
        self.V = set()
        self.ind = 0
        self.Path = []
        self.done = False
        self.Obstaclemap = {}
        self.init_vehicle()

    def init_vehicle(self):
        try:
            spawn_points = self.world.get_map().get_spawn_points()
            vehicle_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_points[0])
            if self.vehicle:
                print("Vehicle spawned.")
            else:
                print("Failed to spawn.")
        except Exception as e:
            print(f"Error: {e}")

    def get_nearest_state(self, state_space, waypoint):
        
        wp_location = carla.Location(x=waypoint[0], y=waypoint[1], z=waypoint[2])
        nearest_state = None
        min_distance = 1000

        for state in state_space:
            state_location = carla.Location(x=state[0], y=state[1], z=state[2])
            if(state_location - wp_location).x > 0:
                distance = state_location.distance(wp_location)
        
                if wp_location.distance(state_location) < min_distance and wp_location.distance(state_location) > 5:
                    min_distance = wp_location.distance(state_location)
                    nearest_state = state

        # Check for obstacles near the nearest state
        if nearest_state:
            nearest_location = carla.Location(x=nearest_state[0], y=nearest_state[1], z=nearest_state[2])
            for actor in self.world.get_actors():
                if isinstance(actor, carla.Vehicle) and actor.id != self.vehicle.id:  
                    obs_location = actor.get_location()
                    if obs_location.distance(nearest_location) < self.obstacle_threshold:
                        # If an obstacle is too close, mark this state as invalid
                        self.Obstaclemap[nearest_state] = True
                        return None
                    
        return nearest_state
    
    #converts carla waypoints objects into list of x,y,z coordinate tuples 
    def convert_waypoints(self, waypoints):
        if not waypoints:
            return []
        return [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z) for wp in waypoints]
    

    # Checks if a 'y' state has both heuristic and tag dicts 
    def checkState(self, y):
        if isinstance(y, carla.Waypoint):
            waypoint_id = y.id
        else:
            waypoint_id = y

        if waypoint_id not in self.h:
            self.h[waypoint_id] = 0
        if waypoint_id not in self.tag:
            self.tag[waypoint_id] = 'New'

    #returns the min key value from the Open set
    def get_kmin(self):
        if self.OPEN:
            if self.OPEN.values():
                return min(self.OPEN.values())
        return -1

    #OPen here could be a open set that comes from tag "open"
    #Convert Open into a priorityQUEUE?
    def min_state(self, waypoint):
        location = carla.Location(x=waypoint[0], y=waypoint[1], z=waypoint[2])
        current_wp = self.map.get_waypoint(location, project_to_road=True)
        if self.OPEN:
            minvalue = min(self.OPEN.values())
            for current_wp in self.OPEN.keys():
                if self.OPEN[current_wp] == minvalue:
                    return current_wp, self.OPEN.pop(current_wp) #returns state k with associated key value
        return None, -1

    def insert(self, x, h_new):
        x_id = x if isinstance(x, tuple) else (x.transform.location.x, x.transform.location.y, x.transform.location.z)
        if self.tag.get(x_id, 'New') == 'New':
            kx = h_new
        elif self.tag[x_id] == 'Open':
            kx = min(self.h.get(x_id, float('inf')), h_new)
        elif self.tag[x_id] == 'Closed':
            kx = min(self.h.get(x_id, float('inf')), h_new)
        self.OPEN.put((kx, x_id))  
        self.h[x_id], self.tag[x_id] = h_new, 'Open'
        
    def process_state(self):
        #If x ,kold are initalized to be wps in min_state then 
        # additional conversion here isnt necessary
        x, kold = self.min_state()
        
        self.tag[x] = 'Closed'
        self.V.add(x)
        if x is None:
            return -1
        # check if 1st timer s
        self.checkState(x)
        if kold < self.h[x]:  # raised states
            for y in self.children(self, x):
                # check y
                self.checkState(y)
                a = self.h[y] + self.cost(self, y, x)
                if self.h[y] <= kold and self.h[x] > a:
                    self.b[x], self.h[x] = y, a
        if kold == self.h[x]:  # lower
            for y in self.children(self, x):
                # check y
                self.checkState(y)
                bb = self.h[x] + self.cost(self, x, y)
                if self.tag[y] == 'New' or \
                        (self.b[y] == x and self.h[y] != bb) or \
                        (self.b[y] != x and self.h[y] > bb):
                    self.b[y] = x
                    self.insert(y, bb)
        else:
            for y in self.children(self, x):
                 # check y
                self.checkState(y)
                bb = self.h[x] + self.cost(self, x, y)
                if self.tag[y] == 'New' or \
                        (self.b[y] == x and self.h[y] != bb):
                    self.b[y] = x
                    self.insert(y, bb)
                else:
                    if self.b[y] != x and self.h[y] > bb:
                        self.insert(x, self.h[x])
                    else:
                        if self.b[y] != x and self.h[y] > bb and \
                                self.tag[y] == 'Closed' and self.h[y] == kold:
                            self.insert(y, self.h[y])
        return self.get_kmin()

    def modify_cost(self, x):
        location = carla.Location(x=x[0], y=x[1], z=x[2])
        current_wp = self.map.get_waypoint(location, project_to_road=True)

        xparent = self.b[current_wp]
        if self.tag[current_wp] == 'Closed':
            self.insert(current_wp, self.h[xparent] + self.cost(current_wp, xparent))

    def modify(self, x):
        #waypoint initialization here or no?
        self.modify_cost(x)
        while True:
            kmin = self.process_state()
            if kmin >= self.h[x]:
                break

    #calculates the cost of moving from one state to next state 
    def cost(self, start, goal):
        if goal in self.Obstaclemap:
            return np.inf
        return carla.Location(x=start[0], y=start[1], z=start[2]).distance(
            carla.Location(x=goal[0], y=goal[1], z=goal[2])
        )

    def children(self, state):
        children = []
        state_location = carla.Location(x=state[0], y=state[1], z=state[2])
        current_waypoint = self.map.get_waypoint(state_location, project_to_road=True)

        if current_waypoint is None:
            return children

        for next_wp in current_waypoint.next(2.0):
            next_state = (next_wp.transform.location.x, next_wp.transform.location.y, next_wp.transform.location.z)
            if next_state in self.state_space:
                children.append(next_state)

        if current_waypoint.lane_change & carla.LaneChange.Left:
            left_wp = current_waypoint.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                left_state = (left_wp.transform.location.x, left_wp.transform.location.y, left_wp.transform.location.z)
                if left_state in self.state_space:
                    children.append(left_state)

        if current_waypoint.lane_change & carla.LaneChange.Right:
            right_wp = current_waypoint.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                right_state = (right_wp.transform.location.x, right_wp.transform.location.y, right_wp.transform.location.z)
                if right_state in self.state_space:
                    children.append(right_state)

        return children

    def path(self, goal=None):
        path = []
        location = []
        if not goal:
            x = self.x0
        else:
            x = goal
        start = self.xt
        while x != start:
            path.append([np.array(x), np.array(self.b[x])])
            location.append(carla.Location(x=x[0], y=x[1], z=x[2]))
            x = self.b[x]

        location.append(carla.Location(x=start[0], y=start[1], z=start[2]))
        #Re-check if set_path is being initialized properly, don't think it being 
        # implemented the same way here
        if self.vehicle:
            self.client.get_trafficmanager().set_path(self.vehicle, location)

        return path
    
    def run(self):
        
        self.OPEN.put((self.h(self.x0, self.xt), self.x0))
        self.tag[self.x0] = 'Open'
        while self.tag.get(self.xt, 'New') != "Closed":
            kmin = self.process_state()
            if kmin == -1:
                print("No path found.")
                return

        self.Path = self.path()
        self.done = True
        self.visualize_path(self.Path)

        for _ in range(100):
            self.move_vehicle()
            s = self.xt
            while s != self.x0:
                sparent = self.b.get(s)
                if self.cost(s, sparent) == np.inf:
                    self.modify(s)
                    continue
                self.ind += 1
                s = sparent
            self.Path = self.path()
            self.visualize_path(self.Path)
    
    #controls vehicle to be moved from one place to another
    def move_vehicle(self):
        if not self.vehicle:
            print("Vehicle not initialized.")
            return

        if self.Path:
            next_waypoint = self.Path[0]
            location = carla.Location(x=next_waypoint[0][0], y=next_waypoint[0][1], z=next_waypoint[0][2])
            self.vehicle.set_location(location)
        else:
            print("Path empty.")

    def visualize_path(self, path):
        debug = self.world.debug
        for segment in path:
            start, end = segment
            debug.draw_line(
                carla.Location(x=start[0], y=start[1], z=start[2]),
                carla.Location(x=end[0], y=end[1], z=end[2]),
                thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=5.0
            )

if __name__ == '__main__':
    D = D_star(1)
    D.run()


