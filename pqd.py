import heapq
import carla
import numpy as np
import random
from collections import defaultdict
from queue import PriorityQueue

#consider move-robot-how does move-robot compare to move_vehicle 
class D_star(object):
    #def __init__(self, waypoint, start_location, goal_location, vehicle, world, map, resolution=2.0):
    def __init__(self, waypoint, start_waypoint, end_waypoint, vehicle, world, map, resolution=2.0):
        self.settings = 'CollisionChecking'
        self.resolution = resolution
        self.waypoint = waypoint
        self.obstacle_threshold = 3.0
        #self.b = defaultdict(lambda: defaultdict(dict))
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
        self.map = map
        self.vehicle = vehicle
        self.state = self.waypoint
        self.location = self.vehicle.get_location()
        self.state_space = self.map.get_waypoint(self.location, project_to_road=True)
        #print(f"print: {self.state_space}")
        self.waypoints = self.map.generate_waypoints(self.resolution)
        print(f"Number of waypoints generated: {len(self.waypoints)}")
        # print(f"waypoint list: {self.waypoints}")
        self.h = {}
        #self.start_location = start_location
        #self.goal_location = goal_location
        #self.x0 = self.map.get_waypoint(self.start_location.transform.location)
        self.x0 = start_waypoint
        print(f'x0: {self.x0}')
        #self.xt = self.map.get_waypoint(self.goal_location.transform.location)
        self.xt = end_waypoint
        print(f'xt: {self.xt}')

    """
    def populate_open(self):
        #takes wp in front of vehicle not left or right
        next_waypoint = self.state_space.next(self.resolution)
        if next_waypoint: 
            next_wp = next_waypoint[0]
            self.key = self.cost(self.state_space, next_wp)
            print(f'key: {self.key}')
            #existing_state = None
            for existing_state in next_waypoint:
                for item in self.OPEN.queue:
                    if item[0] == self.key and item[1] == self.state_space:
                        existing_state = item

            if existing_state is None:
                tup = (self.key, self.state_space)
                self.OPEN.put(tup)
                print(f'OPEN tuple insert: {tup}')
            
        return self.OPEN
    """
    def get_nearby_waypoints(self, radius=50.0):
        vehicle_location = self.vehicle.get_location()
        nearby_waypoints = []

        for wp in self.waypoints:
            wp_location = wp.transform.location
            if wp_location.distance(vehicle_location) <= radius:
                nearby_waypoints.append(wp)

        return nearby_waypoints

    def populate_open(self):
        nearby_waypoints = self.get_nearby_waypoints(radius=50.0)
    
        for wp in nearby_waypoints:
            nearest_state = self.get_nearest_state(wp)
            if nearest_state:
                key = self.cost(wp, nearest_state)
                print(f'key: {key}')
                tup = (key, wp)
                self.OPEN.put(tup)
                print(f'OPEN tuple insert: {tup}')
        return self.OPEN

    
    def get_nearest_state(self, state):

        nearest_state = None
        min_distance = float('inf')

        nearby_waypoints = self.get_nearby_waypoints(radius=50.0)

        for state in nearby_waypoints:
            state_location = carla.Location(
                x=(state.transform.location.x),
                y=(state.transform.location.y),
                z=(state.transform.location.z)
            )

            direction_vector = state_location - self.location
            distance = direction_vector.length()

        
            if distance < min_distance and distance > 5:
                min_distance = distance
                nearest_state = state

                if min_distance <= self.resolution:
                    break
        """
        if nearest_state:
            nearest_location = carla.Location(
                x=(nearest_state.transform.location.x),
                y=(nearest_state.transform.location.y),
                z=(nearest_state.transform.location.z)
            )
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
        """
        return nearest_state

    
    # Checks if a 'y' state has both heuristic and tag dicts
    def checkState(self, y):
        if isinstance(y, carla.Waypoint):
            waypoint_id = y.id
            #print(f'checkState, waypoint_id: {y.id}')
        else:
            waypoint_id = y
        #in unit tests make sure h returns a number and tag returns a string
        if waypoint_id not in self.h:
            self.h[waypoint_id] = 0
            #print(f'checkState, wp_id not in h: {self.h.get(waypoint_id)}')
        if waypoint_id not in self.tag:
            self.tag[waypoint_id] = 'New'

    def get_kmin(self):
        if self.OPEN:
            self.populate_open()
            minimum = self.OPEN.get() # Pop and return the tuple with the minimum key value
            print(f'get_kmin: minimum: {minimum[0]}')
            return minimum[0]
        
    def min_state(self):
        if self.OPEN:
            self.populate_open()
            minimum = self.OPEN.get()
            print(f'get_kmin, state: key: {minimum[0]}, state: {minimum[1]}')
            return minimum[0], minimum[1] #returns state k with associated key value
        return None, -1
    
    def insert(self, h_new, state):
        waypoint_id = state.id
        heuristic = self.store_h(state)

        if waypoint_id not in self.tag:
            self.tag[waypoint_id] = 'New'
    
        new_tag = self.tag[waypoint_id]
        kmin = self.get_kmin()
    
        if new_tag == 'New':
            kx = h_new
        elif new_tag == 'Open':
            kx = min(kmin, h_new)
        elif new_tag == 'Closed':
            #what is getting stored in h?
            #properly define h and values that go into it
            kx = min(heuristic, h_new)
            #self.tag[waypoint_id] = 'Open'
            self.V.add(state)

        self.OPEN.put((kx, state))
        self.h[waypoint_id] = h_new
        self.tag[waypoint_id] = 'Open'

        #print(f'Inserted state {state} with key {kx}')

    def store_h(self, state):
        if state:
            #here, create a iteration that stores heuristic values of states 
            default = state.transform.location.distance(self.xt.transform.location)
            self.h[state.id] = default
            return default
            
        elif state.id in self.h:
            #self.x[x.id] = float('inf')
            return self.h.get(state.id, float('inf'))
        else:
            return float('inf')

    #see how process state goes through obstacle avoidance
    def process_state(self):
        #x is a state, kold is key
        kold, x = self.min_state()
        print(f'process state: kold: {kold}, statex: {x}')

        if x is None: 
            return -1
        
        if x.id == self.xt.id:
            print("goal reached")
            #return -1
        
        self.store_h(x)
        self.checkState(x)
        #print(f'Checked state: {x}')
        # print(f'x: {x}, kold: {kold}')
        #print(f'h: {self.h}') #len of self.h = 0
        #print(f'b: {self.b}')
        if x not in self.V:
            for y in self.children(x):
                self.checkState(y)  
                #print(f'Processing child y: {y}')
                #print(f'x.id: {x.id}, self.h[x.id]: {self.h[x.id]}')
                #print(f'Cost(x, y): {self.cost(x, y)}')
                """
                if y.id not in self.h:
                    self.h[y.id] = float('inf')
                """
                self.store_h(y)
                if y.id not in self.tag:
                    self.tag[y.id] = 'New'
            
                h_new = self.h[x.id] + self.cost(x, y)
                #print(f'h_new: {h_new}')
                if h_new < self.h[y.id]:
                    #reexamine this part: 1. why is self.h[y.id] getting 
                    #a new heuristic value when one alr exists 
                    #2. why is self.b[y.id] getting x? bc x is the parent of y in this case it should be an actual value not an id(corrected, recheck) 
                    self.h[y.id] = h_new
                    self.b[y.id] = x
                    self.insert(h_new, y)

            if kold < self.h[x.id]:  # raised states
                print(f'kold < h[x.id]: {kold} < {self.h[x.id]}')
                for y in self.children(x):
                    print(f'child: {y}')
                    self.checkState(y)
                    heuristic_a = self.h[y.id] + self.cost(y, x)
                    print(f'heuristic_a: {heuristic_a}')
                    if self.h[y.id] <= kold and self.h[x.id] > heuristic_a:
                        self.b[x.id] = y
                        self.h[x.id] = heuristic_a

            if kold == self.h[x.id]:  # lower states 
                print(f'kold == h[x.id]: {kold} == {self.h[x.id]}')
                #why is x getting closed here
                self.tag[x.id] = 'Closed'
                self.V.add(x)
                for y in self.children(x):
                    # check y
                    print(f'child: {y}')
                    self.checkState(y)
                    bb = self.h[x.id] + self.cost(x, y)
                    print(f'h[x.id]: {self.h[x.id]}')
                    print(f'print bb: {bb}')
                    if self.tag[y.id] == 'New' or \
                            (self.b[y.id] == x and self.h[y.id] != bb) or \
                            (self.b[y.id] != x and self.h[y.id] > bb):
                        print(f'Insert y: {y} with bb: {bb}')
                        self.b[y.id] = x
                        self.insert(bb, y)
            else:
                print(f'kold != h[x.id]: {kold} != {self.h[x.id]}')
                for y in self.children(x):
                    # check y
                    print(f'child: {y}')
                    self.checkState(y)
                    bb = self.h[x.id] + self.cost(x, y)
                    print(f'bb: {bb}')
                    if self.tag[y.id] == 'New' or \
                            (self.b[y.id] == x and self.h[y.id] != bb):
                        print(f'insert y: {y} with bb: {bb}')
                        self.b[y.id] = x
                        self.insert(bb, y)
                    elif self.b[y.id] != x and self.h[y.id] > bb:
                        print(f'insert x: {x} with h[x]: {self.h[x.id]}')
                        self.insert(self.h[x.id], x)
                    elif self.b[y.id] != x and self.h[y.id] > bb and \
                        self.tag[y.id] == 'Closed' and self.h[y.id] == kold:
                        print(f'Insert y: {y} with h[y]: {self.h[y.id]}')
                        self.insert(self.h[y.id], y)
            print("No min")

            if self.tag.get(x.id, 'New') == 'Closed' and x == self.xt and x in self.V:
                print(f'reached destination')
        return self.get_kmin()

    def modify_cost(self):
        ss_id = self.state_space.id
        #make sure b is storing states f and xparent is a state
        xparent = self.b[ss_id]
        print(f'modify_cost: xparent: {xparent}')
        if self.tag[ss_id] == 'Closed':
            print(f'state_space {self.state_space} is Closed')
            cost_value = self.h[xparent] + self.cost(self.state_space, xparent)
            print(f'Inserting with cost: {cost_value}')
            self.insert(cost_value, self.state_space)


    def modify(self, state_space):
        print(f'modify: state_space: {state_space}')
        self.modify_cost()
        #this while loop will run forever
        prev_k = float('inf')
        kmin = self.get_kmin()
        #this could be causing the inf loop cause kmin is never updates thus always greater than h
        while kmin is not None and kmin < self.h[state_space.id]: #kmin-start to goal state_space heuristic 
            prev_k = kmin
            kmin = self.process_state()
            print(f'process_state returned kmin: {kmin}')
        if kmin is None or kmin >= self.h[state_space.id]:
            print(f'kmin {kmin} >= h[state_space] {self.h[state_space]}')
            return -1 

    def cost(self, start, goal):
        distance = start.transform.location.distance(goal.transform.location)
        print(f'Distance: {distance}')
        return distance

    def children(self, state):
        children = []
        #get vehicle's current location
        location = self.vehicle.get_location()
        current_waypoint = self.map.get_waypoint(location, project_to_road=True)

        if current_waypoint is None:
            return children

        next_waypoint = current_waypoint.next(2.0)
        if next_waypoint:
            next_wp = next_waypoint[0]
            if next_wp.transform.location not in [c.transform.location for c in children]:
                children.append(next_wp)

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

    def path(self):
        goal = self.xt
        path = []
        #location = []
        
        if not goal:
            trace_state = self.x0
            print(f'No goal provided, using x0: {trace_state}')
        else:
            trace_state = goal
            print(f'Goal provided: {self.goal_location}, start: {self.x0}')
        #start = self.xt

        while trace_state != self.x0:
            #print(f'Appending to path: x: {trace_state}, b[x]: {self.b[trace_state.id]}')
            #path.append([np.array(trace_state), np.array(self.b[trace_state.id])])
            #location.append(carla.Location(x=trace_state.transform.location.x, y=trace_state.transform.location.y, z=trace_state.transform.location.z))

        #location.append(carla.Location(x=self.goal_location.transform.location.x, y=self.goal_location.transform.location.y, z=self.goal_location.transform.location.z))
            x = self.b[trace_state.id]
            trace_location = np.array([trace_state.transform.location.x, trace_state.transform.location.y, trace_state.transform.location.z])
            parent_location = np.array([x.transform.location.x, x.transform.location.y, x.transform.location.z])
            path.append([trace_location, parent_location])

            trace_state = x
            print(f"path append: {x}")
        
        return path

    def run(self):

        self.OPEN.put((self.cost(self.x0, self.xt), self.x0))
        self.OPEN.put((0, self.xt))
        self.tag[self.x0] = 'New'
        print(f'self.tag: {self.tag}')
        print(f'self.x0: {self.x0}')

        while self.tag.get(self.state_space.id, 'New') != "Closed":
            print(f"Goal state tag: {self.tag.get(self.xt.id, 'New')}") 
            self.process_state()
            """
            kmin = self.process_state() 

            print(f'kmin: {kmin}')
            if kmin == -1:
                print("No path found.")
                return
            """
            if self.tag[self.state_space.id] == 'Closed':
                break
            self.ind += 1
        self.Path = self.path()
        print(f'Path found: {self.Path}')
        self.done = True
        self.visualize_path(self.Path)

        for _ in range(100):
            self.move_vehicle()
            s = self.x0
            while s != self.xt:
                if s == self.x0:
                    sparent = self.b[self.x0.id]
                    print(f'Checking state: {s}, parent: {sparent}')
                else:
                    sparent = self.b[s.id]
                if self.cost(s, sparent) == np.inf:
                    print(f'Cost is infinite, modifying state: {s}')
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
            print(f'Next waypoint: {next_waypoint}')
            location = carla.Location(next_waypoint.transform.location.x, next_waypoint.transform.location.y, next_waypoint.transform.location.z)
            print(f'Setting vehicle location to: {location}')
            self.vehicle.set_location(location)
        else:
            print("Path empty.")
    
    
    def visualize_path(self, path):
        debug = self.world.debug
        for segment in path:
            start, end = segment
            debug.draw_line(
                carla.Location(x=self.x0.transform.location.x, y=self.x0.transform.location.y, z=self.x0.transform.location.z),
                carla.Location(x=self.xt.transform.location.x, y=self.xt.transform.location.y, z=self.xt.transform.location.z),
                thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=15.0
            )

if __name__ == '__main__':
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    # world = client.load_world('Town05') # Use this to switch towns
    # Get the world and map
    world = client.get_world()
    carla_map = world.get_map()

    # Spawn a firetruck at a random location (point A)
    blueprint_library = world.get_blueprint_library()
    firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
    spawn_points = carla_map.get_spawn_points()


    # Choose a random starting location (point A)
    point_a = random.choice(spawn_points)
    firetruck = world.spawn_actor(firetruck_bp, point_a)
    # Choose a random destination (point B)
    point_b = random.choice(spawn_points)
    if point_b.location == point_a.location:
        point_b = random.choice(spawn_points)

    start_waypoint = carla_map.get_waypoint(point_a.location)
    end_waypoint = carla_map.get_waypoint(point_b.location)

    try:
        D = D_star(1, start_waypoint, end_waypoint, firetruck, world, carla_map)
        D.run()

    finally:
        firetruck.destroy() 
        print("Firetruck destroyed")