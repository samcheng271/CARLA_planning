import heapq
import carla
import numpy as np
import random
from collections import defaultdict
# from queue import PriorityQueue
from queue import PriorityQueue
#from PriorityQueue import PriorityQueue, Priority # Make sure you have PriorityQueue.py 
# and that the built in queue module is commented out

#consider move-robot-how does move-robot compare to move_vehicle 
class D_star(object):
    def __init__(self, waypoint, resolution=2.0):
        self.settings = 'CollisionChecking'
        self.resolution = resolution
        self.waypoint = waypoint
        self.obstacle_threshold = 3.0
        self.b = defaultdict(lambda: defaultdict(dict))
        self.OPEN = PriorityQueue()
        
        self.tag = {}
        self.V = set()
        self.parameters = ()
        self.ind = 0
        self.Path = []
        self.done = False
        self.Obstaclemap = {}
        #heapq.heapify(self.OPEN)

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
        self.state = self.waypoint
        self.location = self.vehicle.get_location()
        self.state_space = self.map.get_waypoint(self.location, project_to_road=True)
        #print(f"print: {self.state_space}")
        #self.state_space = self.map.get_waypoint(self.vehicle.get_location())
        self.waypoints = self.map.generate_waypoints(self.resolution)
        print(f"Number of waypoints generated: {len(self.waypoints)}")
        # print(f"waypoint list: {self.waypoints}")
        self.h = {}

        wp_tuple = self.init_vehicle()
        if wp_tuple:
            start_location, goal_location = wp_tuple
        print(f'start_location = {start_location}, goal_location = {goal_location}')
        self.start_location = start_location
        self.goal_location = goal_location
        #self.start_location = self.init_vehicle.parameters[0]
        #self.goal_location = self.init_vehicle.parameters[1]

        self.x0 = self.map.get_waypoint(self.start_location.transform.location)
        print(f'x0: {self.x0}')
        self.xt = self.map.get_waypoint(self.goal_location.transform.location)
        print(f'xt: {self.xt}')

    def init_vehicle(self):
        try:
            if self.vehicle:
                #self.vehicle is initalized 
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

        start_end = (start_waypoint, end_waypoint)
        return start_end


    def get_nearest_state(self, state):

        vehicle_location = self.vehicle.get_location()
        positive_vehicle = self.vehicle.get_transform().get_forward_vector()
        print(f'positive_vehicle: {positive_vehicle}')
        nearest_state = None
        min_distance = float('inf')

        
        for state in self.waypoints:
            state_location = carla.Location(x=(state.transform.location.x), y=(state.transform.location.y), z=(state.transform.location.z))
            """
            print(f'statex: {state.transform.location.x}')
            print(f'statey: {state.transform.location.y}')
            print(f'statez: {state.transform.location.z}')
            """
            direction_vector = state_location - vehicle_location
            distance = direction_vector.length()
            
            #issue: min distance is sometimes less than distance which is causing the condition to fail 
            if positive_vehicle.dot(direction_vector) > 0 and distance < min_distance and distance > 5:
                min_distance = distance
                nearest_state = state

                if min_distance <= self.resolution:
                    break

        if nearest_state:
            nearest_location = carla.Location(x=(nearest_state.transform.location.x), y=(nearest_state.transform.location.y), z=(nearest_state.transform.location.z))
            print(f'nearest_location: {nearest_location}')
            min_dist = float('inf')
            # print(f'get actors:{self.world.get_actors()}')
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
    
    def populate_open(self):
        if not self.vehicle:
            return []
        next_waypoint = self.state_space.next(self.resolution)
        if next_waypoint: 
            next_wp = next_waypoint[0]
            next_wp_loc = carla.Location(x=next_wp.transform.location.x, y=next_wp.transform.location.y, z=next_wp.transform.location.z)
            #self.key = self.cost(self.state_space, self.get_nearest_state(self.waypoint))
            self.key = self.cost(self.state_space, next_wp_loc)
            #print(f'key: {self.key}')
            tup = (self.key, self.state_space)
    
            self.OPEN.put(tup)
            print(f'OPEN tuple insert: {tup}')
            
        return self.OPEN

    # Checks if a 'y' state has both heuristic and tag dicts
    def checkState(self, y):
        if isinstance(y, carla.Waypoint):
            waypoint_id = y.id
            print(f'checkState, waypoint_id: {y.id}')
        else:
            waypoint_id = y
            print(f'checkstate, else: waypoint_id: {y}')

        #in unit tests make sure h returns a number and tag returns a string
        if waypoint_id not in self.h:
            self.h[waypoint_id] = 0
        if waypoint_id not in self.tag:
            self.tag[waypoint_id] = 'New'

    def get_kmin(self):
        if self.OPEN:
            self.populate_open()
            minimum = self.OPEN.get() # Pop and return the tuple with the minimum key value
            #print(f'get_kmin: minimum: {minimum[0]}')
            return minimum[0]
        
    def min_state(self):
        if self.OPEN:
            self.populate_open()
            minimum = self.OPEN.get()
            #print(f'get_kmin, state: key: {minimum[0]}, state: {minimum[1]}')
            return minimum[0], minimum[1] #returns state k with associated key value
        
        return None, -1

    def get_wpid(self, waypoint_id):
        for waypoint in self.waypoints:
            if waypoint.id == waypoint_id:
                return waypoint
        return None 
    
    #check again
    def insert(self, h_new, state):
        #x is a state
        #if isinstance(state, carla.Waypoint):
        waypoint_id = state.id
        #else:
            #waypoint_id = state

        if h_new is None:
            state_location = self.world.get_map().get_wpid(waypoint_id).transform.location
            h_new = self.cost(self.state_space, state_location)
            
        if waypoint_id in self.tag:
            tag = self.tag[waypoint_id]
        else: 
            tag = 'New'

        if tag == 'Closed':
            self.V.add(state)
            return -1

        #check this if loop
        if tag == 'New' or h_new < self.h.get(waypoint_id, float('inf')):
            kx = min(self.h.get(waypoint_id, float('inf')), h_new)
            self.OPEN.put((kx, state))
            self.h[waypoint_id] = h_new
            self.tag[waypoint_id] = 'Open' 
            print(f'Inserted state {state} with key {kx}')

    #see how process state goes through obstacle avoidance
    def process_state(self):
        #x is a state, kold is key
        kold, x = self.min_state()
        print(f'process state: kold: {kold}, statex: {x}')

        if x is None: 
            return -1
        
        if self.h[x.id] == float('inf'):
            return -1 
        
        if x.id == self.xt.id:
            print("goal reached")
            return -1
        
        self.checkState(x)
        print(f'Checked state: {x}')
        # print(f'x: {x}, kold: {kold}')
        print(f'h: {self.h}')
        print(f'b: {self.b}')
        if x.id not in self.V:
            for y in self.children(x):
                self.checkState(y)  
                print(f'Processing child y: {y}')
                print(f'x.id: {x.id}, self.h[x.id]: {self.h[x.id]}')
                print(f'Cost(x, y): {self.cost(x, y)}')
                if y.id not in self.h:
                    self.h[y.id] = float('inf')
                if y.id not in self.tag:
                    self.tag[y.id] = 'New'
            
                h_new = self.h[x.id] + self.cost(x, y)
                print(f'h_new: {h_new}')
                if h_new < self.h[y.id]:
                    self.h[y.id] = h_new
                    self.b[y.id] = x.id
                    self.insert(h_new, y)

            if kold < self.h[x.id]:  # raised states
                print(f'kold < h[x.id]: {kold} < {self.h[x.id]}')
                for y in self.children(x):
                    print(f'child: {y}')
                    self.checkState(y)
                    a = self.h[y.id] + self.cost(y, x)
                    print(f'print a: {a}')
                    if self.h[y.id] <= kold and self.h[x.id] > self.h[y.id] + self.cost(y, x):
                        self.b[x.id] = y.id
                        self.h[x.id] = self.h[y.id] + self.cost(y, x)

            if kold == self.h[x.id]:  # lower states 
                print(f'kold == h[x.id]: {kold} == {self.h[x.id]}')
                self.tag[x.id] = 'Closed'
                self.V.add(x)
                for y in self.children(x):
                    # check y
                    print(f'child: {y}')
                    self.checkState(y)
                    bb = self.h[x.id] + self.cost(x, y)
                    print(f'print bb: {bb}')
                    if self.tag[y.id] == 'New' or \
                            (self.b[y.id] == x.id and self.h[y.id] != bb) or \
                            (self.b[y.id] != x.id and self.h[y.id] > bb):
                        print(f'Insert y: {y} with bb: {bb}')
                        self.b[y.id] = x.id
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
                        self.b[y.id] = x.id
                        self.insert(bb, y)
                    elif self.b[y.id] != x.id and self.h[y.id] > bb:
                        print(f'insert x: {x} with h[x]: {self.h[x.id]}')
                        self.insert(self.h[x.id], x)
                    elif self.b[y.id] != x.id and self.h[y.id] > bb and \
                        self.tag[y.id] == 'Closed' and self.h[y.id] == kold:
                        print(f'Insert y: {y} with h[y]: {self.h[y.id]}')
                        self.insert(self.h[y.id], y)
            print("No min")
        else: 
            return -1
        return self.get_kmin()

    def modify_cost(self):
        xparent = self.b[self.state_space]
        print(f'modify_cost: xparent: {xparent}')
        if self.tag[self.state_space] == 'Closed':
            print(f'state_space {self.state_space} is Closed')
            cost_value = self.h[xparent] + self.cost(self.state_space, xparent)
            print(f'Inserting with cost: {cost_value}')
            self.insert(cost_value, self.state_space)

    def modify(self, state_space):
        print(f'modify: state_space: {state_space}')
        self.modify_cost()
        #this while loop will run forever 
        kmin = self.get_kmin()
        while kmin is not None and kmin < self.h[state_space]:
            kmin = self.process_state()
            print(f'process_state returned kmin: {kmin}')
        if kmin is None or kmin >= self.h[state_space]:
            print(f'kmin {kmin} >= h[state_space] {self.h[state_space]}')
            return -1 

    def cost(self, start, goal):
        """
        print(f'Calculating cost from {start} to {goal}')
        if goal.id in self.Obstaclemap:
            print(f'Goal {goal.id} is in Obstaclemap')
            return np.inf
        """

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
        print(f'path: goal: {goal}')
        path = []
        location = []
        start = self.xt
        if not goal:
            x = self.x0
            print(f'No goal provided, using x0: {x}')
        else:
            x = goal
            print(f'Goal provided: {goal}, start: {start}')

        while x != start:
            if x not in self.b: 
                break
            print(f'Appending to path: x: {x}, b[x]: {self.b[x]}')
            path.append([np.array(x), np.array(self.b[x])])
            location.append(carla.Location(x=x[0], y=x[1], z=x[2]))
            x = self.b[x]

        print(f'Appending final location: start: {start}')
        location.append(carla.Location(x=start[0], y=start[1], z=start[2]))

        return path

    def run(self):

        #heapq.heappush(self.OPEN, (self.cost(self.x0, self.xt), self.x0))
        #double check
        self.OPEN.put((self.cost(self.x0, self.xt), self.x0))

        print(f'self.tag: {self.tag}')
        print(f'self.x0: {self.x0}')

        #this while loop could potentially be the issue, xt.id is never turning into closed
        #what does it need to make it closed--for it to be in visited?
        while self.tag.get(self.xt.id, 'New') != "Closed":
            print(f"Goal state tag: {self.tag.get(self.xt.id, 'New')}") 
            kmin = self.process_state()
            print(f'kmin: {kmin}')
            if kmin == -1:
                print("No path found.")
                return

            #for state in self.V: 
            #state is added to V here but nothing else happens
            #need to iterate through V set as the algs runs until so the 
            #condition all of the paths explored are added to this V and eventually x0 == xt
        self.Path = self.path()
        print(f'Path found: {self.Path}')
        self.done = True
        self.visualize_path(self.Path)

        for _ in range(100):
            self.move_vehicle()
            s = self.xt
            while s != self.x0:
                sparent = self.b.get(s)
                print(f'Checking state: {s}, parent: {sparent}')
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
            location = carla.Location(x=next_waypoint[0][0], y=next_waypoint[0][1], z=next_waypoint[0][2])
            print(f'Setting vehicle location to: {location}')
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