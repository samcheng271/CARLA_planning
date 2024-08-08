import heapq
import carla
import numpy as np
import random
from collections import defaultdict
from queue import PriorityQueue

#write a separate file
class D_star(object):
    def __init__(self, waypoint, resolution=0.5):
        self.settings = 'CollisionChecking'
        self.resolution = resolution
        self.waypoint = waypoint
        self.obstacle_threshold = 5.0
        self.b = defaultdict(lambda: defaultdict(dict))
        self.OPEN = []
        self.h = {}
        self.tag = {}
        self.V = set()
        self.parameters = ()
        self.ind = 0
        self.Path = []
        self.done = False
        self.Obstaclemap = {}

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        #this is already initalizing vehicle to a starting locaiton
        vehicle_bp = self.world.get_blueprint_library().filter('vehicle.carlamotors.firetruck')[0]
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.start_spawn = self.spawn_points[0]
        vehicle = self.world.spawn_actor(vehicle_bp, self.start_spawn)

        self.vehicle = vehicle
        # print("Vehicle spawned.", vehicle.get_location())
        self.state = self.waypoint
        self.state_space = self.map.get_waypoint(self.vehicle.get_location())
        self.waypoints = self.map.generate_waypoints(self.resolution)
        

        wp_tuple = self.init_vehicle()
        if wp_tuple:
            start_location, goal_location = wp_tuple
        self.start_location = start_location
        self.goal_location = goal_location
        #self.start_location = self.init_vehicle.parameters[0]
        #self.goal_location = self.init_vehicle.parameters[1]

        #self.x0 = self.get_nearest_state(self.start_location)
        #self.xt = self.get_nearest_state(self.goal_location)
        self.x0 = self.start_location.next(1.0)
        self.xt = self.goal_location.next(1.0)



        #make sure if a initalization function is being called here, that anything that might be used w init variables are
        #initalized also properly.
        # self.init_vehicle()

    def populate_open(self):
        if not self.vehicle:
            return []
        #self.key = self.cost(self.vehicle.get_location(), self.get_nearest_state(self.state_space))
        vehicle_transform = self.vehicle.transform
        vehicle_location = vehicle_transform.location
        self.key = self.cost(self.vehicle_location, self.get_nearest_state(self.waypoint))
        tup = (self.key, self.state_space)
        for wp in self.waypoints:
            heapq.heappush(self.OPEN, tup)
        return self.OPEN

    def init_vehicle(self):
        try:
            if self.vehicle:
                print("Vehicle spawned.")
            else:
                print("Failed to spawn.")
                self.vehicle = None
                return None
        except Exception as e:
            print(f"Failed to connect to Carla: {e}")
            self.vehicle = None
            return None

        goal_vehicle = random.choice(self.spawn_points)
        while goal_vehicle.location == self.start_spawn.location:
            goal_vehicle = random.choice(self.spawn_points)

        start_waypoint = self.map.get_waypoint(self.start_spawn.location, project_to_road=True)
        end_waypoint = self.map.get_waypoint(goal_vehicle.location, project_to_road=True)

        # waypoint_a = self.map.get_waypoint(self.start_spawn, project_to_road=True)
        # waypoint_b = self.map.get_waypoint(goal_vehicle, project_to_road=True)

        # start_waypoint = waypoint_a
        # end_waypoint = waypoint_b

        start_end = (start_waypoint, end_waypoint)
        return start_end


    def get_nearest_state(self, state):

        #wp_location = carla.Location(x=waypoints[0], y=waypoints[1], z=waypoints[2])
        # transform = vehicle.get_transform()
        # vehicle_location = transform.location
        vehicle_location = self.vehicle.get_location()
        nearest_state = None
        min_distance = 1000

        #gets the state_space nearest to vehicle
        for state in self.waypoints:
            # print('statex:: ',state.transform.location.x)
            state_location = carla.Location(x=state.transform.location.x, y=state.transform.location.y, z=state.transform.location.z)
            distance = vehicle_location.distance(state_location)

            if distance < min_distance and distance > 5:
                min_distance = distance
                nearest_state = state


        if nearest_state:
            nearest_location = carla.Location(x=nearest_state.transform.location.x, y=nearest_state.transform.location.y, z=nearest_state.transform.location.z)
            min_dist = float('inf')
            for actor in self.world.get_actors():
                if isinstance(actor, carla.Vehicle) and actor.id != self.vehicle.id:
                    obs_location = actor.get_location()
                    if obs_location.distance(nearest_location) < self.obstacle_threshold:
                        # If an obstacle is too close, mark this state as invalid
                        min_dist = min(min_dist, obs_location.distance(nearest_location))
            if min_dist < self.obstacle_threshold:
                self.Obstaclemap[nearest_state] = True
                return None

        return nearest_state

    #converts carla waypoints objects into list of x,y,z coordinate tuples
    #def convert_waypoints(self, waypoints):
        #if not waypoints:
            #return []
        #return [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z) for wp in waypoints]

    # Checks if a 'y' state has both heuristic and tag dicts
    def checkState(self, y):
        if isinstance(y, carla.Waypoint):
            waypoint_id = y.id
        else:
            waypoint_id = y

        #in unit tests make sure h returns a number and tag returns a string
        if waypoint_id not in self.h:
            self.h[waypoint_id] = 0
        if waypoint_id not in self.tag:
            self.tag[waypoint_id] = 'New'

    def get_kmin(self):
        if self.OPEN:
            self.populate_open()
            minimum = heapq.heappop(self.OPEN)  # Pop and return the tuple with the minimum key value
            return minimum[0]
        
    def min_state(self):
        if self.OPEN:
            self.populate_open()
            min_element = heapq.heappop(self.OPEN)
            return min_element[1], min_element[0] #returns state k with associated key value
        return None, -1

    #check again
    def insert(self, x, h_new=None):
        if isinstance(x, carla.Waypoint):
            waypoint_id = x.id
        else:
            waypoint_id = x

        tag = self.tag.get(x, 'New')

        if h_new is None:
            vehicle_location = self.vehicle.get_location()
            v_waypoint = world.get_map().get_waypoint(vehicle_location, project_to_road=True)
            #change first parameter to a waypoint
            h_new = self.cost(v_waypoint, x)

        if tag == 'New':
            kx = h_new
            #analyze Open case
        elif tag == 'Open':
            kx = min(self.h.get(x, float('inf')), h_new)
        elif tag == 'Closed':
            kx = min(self.h.get(x, float('inf')), h_new)
        else:
            kx = h_new

        heapq.heappush(self.OPEN, (kx, x))
        self.h[x], self.tag[x] = h_new, 'Open'

    #see how process state goes through obstacle avoidance
    def process_state(self):
        x, kold = self.min_state()

        self.tag[x] = 'Closed'
        self.V.add(x)
        if x is None:
            return -1
        # check if 1st timer s
        self.checkState(x)
        if kold < self.h[x]:  # raised states
            for y in self.children(x):
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
                    self.insert(bb, y)
        else:
            for y in self.children(self, x):
                 # check y
                self.checkState(y)
                bb = self.h[x] + self.cost(self, x, y)
                if self.tag[y] == 'New' or \
                        (self.b[y] == x and self.h[y] != bb):
                    self.b[y] = x
                    self.insert(bb, y)
                else:
                    if self.b[y] != x and self.h[y] > bb:
                        self.insert(self.h[x], x)
                    else:
                        if self.b[y] != x and self.h[y] > bb and \
                                self.tag[y] == 'Closed' and self.h[y] == kold:
                            self.insert(self.h[y], y)
        return self.get_kmin()

    def modify_cost(self):
        #x is a state; initialize it to a state->waypoint, more specifically the current wp
        location = self.vehicle.get_location()
        current_wp = self.map.get_waypoint(location, project_to_road=True)
        #b is a backpointer-holder-gets the pred of current state
        xparent = self.b[current_wp]
        if self.tag[current_wp] == 'Closed':
            self.insert(self.h[xparent] + self.cost(current_wp, xparent), current_wp)

    def modify(self, state_space):
        #x is a state; initialize it to a state->waypoint, more specifically the current wp
        location = self.vehicle.get_location()
        current_wp = self.map.get_waypoint(location, project_to_road=True)
        self.modify_cost(current_wp)
        while True:
            kmin = self.process_state()
            if kmin >= self.h[current_wp]:
                break

    def cost(self, start, goal):
        if goal in self.Obstaclemap:
            return np.inf
        #return carla.Location(x=start[0], y=start[1], z=start[2]).distance(carla.Location(x=goal[0], y=goal[1], z=goal[2]))
        return start.transform.location.distance(goal.transform.location)

    def children(self, state):
        children = []
        #get vehicle's current location
        location = self.vehicle.get_location()
        current_waypoint = self.map.get_waypoint(location, project_to_road=True)

        if current_waypoint is None:
            return children

        for next_wp in current_waypoint.next(2.0):
            next_state = (next_wp.transform.location.x, next_wp.transform.location.y, next_wp.transform.location.z)
            if next_state in self.waypoints:
                children.append(next_state)

        if current_waypoint.lane_change & carla.LaneChange.Left:
            left_wp = current_waypoint.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                left_state = (left_wp.transform.location.x, left_wp.transform.location.y, left_wp.transform.location.z)
                if left_state in self.waypoints:
                    children.append(left_state)

        if current_waypoint.lane_change & carla.LaneChange.Right:
            right_wp = current_waypoint.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                right_state = (right_wp.transform.location.x, right_wp.transform.location.y, right_wp.transform.location.z)
                if right_state in self.waypoints:
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

        return path

    def run(self):

        heapq.heappush(self.OPEN, (self.cost(self.x0, self.xt), self.x0))
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
    try:
        D = D_star(1)
        D.run()
    finally:
        D.vehicle.destroy()
