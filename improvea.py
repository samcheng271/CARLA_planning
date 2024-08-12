import heapq
import carla
import numpy as np
import random
from collections import defaultdict
from queue import PriorityQueue

#write a separate file
#consider move-robot
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
        self.location = self.vehicle.get_location()
        self.state_space = self.map.get_waypoint(self.location, project_to_road=True)
        #self.state_space = self.map.get_waypoint(self.vehicle.get_location())
        self.waypoints = self.map.generate_waypoints(self.resolution)
        

        wp_tuple = self.init_vehicle()
        if wp_tuple:
            start_location, goal_location = wp_tuple
        # print(f'start_location = {start_location}, goal_location = {goal_location}')
        self.start_location = start_location
        self.goal_location = goal_location
        #self.start_location = self.init_vehicle.parameters[0]
        #self.goal_location = self.init_vehicle.parameters[1]

        self.x0 = self.map.get_waypoint(self.start_location.transform.location)
        self.xt = self.map.get_waypoint(self.goal_location.transform.location)
        # self.x0 = self.get_nearest_state(self.start_location)
        # self.xt = self.get_nearest_state(self.goal_location)
        # self.x0 = self.start_location.next(1.0)
        # self.xt = self.goal_location.next(1.0)
        # print(f'self.x0 = {self.x0} : self.xt = {self.xt}')



        #make sure if a initalization function is being called here, that anything that might be used w init variables are
        #initalized also properly.
        # self.init_vehicle()

    def populate_open(self):
        if not self.vehicle:
            return []
        #self.key = self.cost(self.vehicle.get_location(), self.get_nearest_state(self.state_space))

        #self.vehicle_location = self.vehicle.get_location()
        #vehicle_waypoint = self.map.get_waypoint(self.vehicle_location, project_to_road=True)
        #self.key = self.cost(vehicle_waypoint, self.get_nearest_state(self.waypoint))
        self.key = self.cost(self.state_space, self.get_nearest_state(self.waypoint))
        tup = (self.state_space, self.key)
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
                self.Obstaclemap[nearest_state.id] = True
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
            return minimum[1]
        
    def min_state(self):
        if self.OPEN:
            self.populate_open()
            min_element = heapq.heappop(self.OPEN)
            return min_element[0], min_element[1] #returns state k with associated key value
        return None, -1

    def get_wpid(self, waypoint_id):
        for waypoint in self.waypoints:
            if waypoint.id == waypoint_id:
                return waypoint
        return None 
    
    #check again
    def insert(self, state, h_new):
        #x is a state
        #if isinstance(state, carla.Waypoint):
        waypoint_id = state.id
        #else:
            #waypoint_id = state

        if h_new is None:
            #here x is an id initalization but cost is being performed between a waypoint and a waypoint id 
            state_location = world.get_map().get_wpid(waypoint_id).transform.location
            #state.get_location()
            #state_waypoint = world.get_map().get_waypoint(state_location, project_to_road=True)

            #vehicle_location = self.vehicle.get_location()
            #v_waypoint = world.get_map().get_waypoint(vehicle_location, project_to_road=True)
            #change first parameter to a waypoint
            h_new = self.cost(self.state_space, state_location)
            #tag state here is always getting initalized to "new"
            #tag = self.tag.get(x, 'New')
            
        if waypoint_id in self.tag:
            tag = self.tag[waypoint_id]
        else: 
            tag = 'New'

        if tag == 'New':
            kx = h_new
            #analyze Open case
        if tag == 'Open':
            kx = min(self.h.get(state, float('inf')), h_new)
        if tag == 'Closed':
            kx = min(self.h.get(state, float('inf')), h_new)
        
        heapq.heappush(self.OPEN, (kx, state))
        self.h[waypoint_id], self.tag[waypoint_id] = h_new, 'Open'

    #see how process state goes through obstacle avoidance
    def process_state(self):
        if not self.OPEN:
            print("Open is empty")
            return -1
        #x is a state, kold is key
        #x, kold = self.min_state()
        kold, x = self.min_state()
        if x is None:
            print("No valid state")
            return -1 

        # print(f'x: {x}, kold: {kold}')
        self.tag[x.id] = 'Closed'
        self.V.add(x)
        
        if x.id == self.xt.id:
            print("goal reached")
            return -1
        
        self.checkState(x)
        # print(f'x: {x}, kold: {kold}')
        # print(f'h: {self.h}')
        # print(f'b: {self.b}')
        if kold < self.h[x.id]:  # raised states
            for y in self.children(x):
                # check y
                self.checkState(y)
                a = self.h[y] + self.cost(self, y, x)
                if self.h[y] <= kold and self.h[x] > a:
                    self.b[x], self.h[x] = y, a
        if kold == self.h[x.id]:  # lower
            for y in self.children(x):
                # check y
                self.checkState(y)
                bb = self.h[x] + self.cost(x, y)
                if self.tag[y] == 'New' or \
                        (self.b[y] == x and self.h[y] != bb) or \
                        (self.b[y] != x and self.h[y] > bb):
                    self.b[y] = x
                    self.insert(y, bb)
        else:
            for y in self.children(x):
                 # check y
                self.checkState(y)
                bb = self.h[x] + self.cost(x, y)
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
        print("No min")
        return self.get_kmin()

    def modify_cost(self):
        #x is a state; initialize it to a state->waypoint, more specifically the current wp
        #location = self.vehicle.get_location()
        #current_wp = self.map.get_waypoint(location, project_to_road=True)
        #b is a backpointer-holder-gets the pred of current state
        xparent = self.b[self.state_space]
        if self.tag[self.state_space] == 'Closed':
            self.insert(self.state_space, self.h[xparent] + self.cost(self.state_space, xparent))

    def modify(self, state_space):
        #x is a state; initialize it to a state->waypoint, more specifically the current wp
        #location = self.vehicle.get_location()
        #current_wp = self.map.get_waypoint(location, project_to_road=True)
        self.modify_cost(state_space)
        while True:
            kmin = self.process_state()
            if kmin >= self.h[state_space]:
                break

    def cost(self, start, goal):
        # print(f'start: {start}, goal: {goal}')
        # g_tuple = tuple(goal)
        if goal.id in self.Obstaclemap:
            return np.inf
        # s = start[0]
        # g= goal[0]
        # print(f'start: {start}, goal: {goal[0]}')
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
        print(f'self.tag: {self.tag}')
        print(f'self.x0: {self.x0}')
        while self.tag.get(self.xt.id, 'New') != "Closed":
            print(f"Goal state tag: {self.tag.get(self.xt.id, 'New')}") 
            kmin = self.process_state()
            print(f'kmin: {kmin}')
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

            if s == self.x0:
                print("Path was reconstructed")
                break

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
