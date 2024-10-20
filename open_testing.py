import carla
import numpy as np
import random
import math
from queue import PriorityQueue

#consider move-robot-how does move-robot compare to move_vehicle 
class D_star(object):
    #def __init__(self, waypoint, start_location, goal_location, vehicle, world, map, resolution=2.0):
    def __init__(self, waypoint, start_waypoint, end_waypoint, vehicle, world, map, resolution=2.0):
        self.settings = 'CollisionChecking'
        self.resolution = resolution
        self.waypoint = waypoint
        self.obstacle_threshold = 3.0
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
        #self.state_space = start_waypoint
        self.waypoints = self.map.generate_waypoints(self.resolution)
        print(f"Number of waypoints generated: {len(self.waypoints)}")
        self.h = {}
        self.next_waypoint = None
        self.x0 = start_waypoint
        print(f'start_waypoint: {self.x0.transform.location}')
        #self.state_space = self.x0 
        self.xt = end_waypoint
        print(f'end_waypoint: {self.xt.transform.location}')
    
    
    #this function is good 
    def populate_open(self, state):
        
        print("\n=== Starting populate_open() function ===")
        print(f"Initial state: {state}")
        print(f"Current next_waypoint status: {self.next_waypoint is not None}")
        
        if self.next_waypoint:
            print(f"Using stored next_waypoint: {self.next_waypoint.transform.location}")
            child_waypoints = [self.next_waypoint]
            self.next_waypoint = None
            print("Reset next_waypoint to None")
        else:
            print("No stored next_waypoint, generating child waypoints")
            child_waypoints = self.children(state)
            
        print(f"\nTotal child waypoints found: {len(child_waypoints)}")
        print(f"Child waypoint locations: {[wp.transform.location for wp in child_waypoints]}")

        print("\n=== Processing each child waypoint ===")
        for i, child in enumerate(child_waypoints):
            print(f"\nProcessing child {i + 1}/{len(child_waypoints)}")
            print(f"Child location: {child.transform.location}")
            
            if child in self.V:
                print(f"Skipping already processed waypoint: {child.transform.location}")
                continue

            print(f"Computing cost for child waypoint")
            cost = self.cost(state, child)
            h_value = self.store_h(child)
            key = cost + h_value
            print(f"Cost: {cost}, H-value: {h_value}, Total key: {key}")
            
            print("Checking for existing similar keys in OPEN queue")
            existing_items = [item for item in self.OPEN.queue if abs(item[0] - key) < 1e-6]
            print(f"Found {len(existing_items)} similar keys")

            if not existing_items:
                print(f"No similar keys found, adding to OPEN queue with key: {key}")
                self.OPEN.put((key, child))
                print(f"Added to OPEN: Waypoint at {child.transform.location} with key {key}")
                print(f"Current state: {state.transform.location}")
                self.V.add(child)
                print(f"Added waypoint to visited set V")
            else:
                print(f"\nSimilar key {key} already exists")
                print("Generating next waypoints for current child")
                next_waypoints = self.children(child)

                if next_waypoints:
                    self.next_waypoint = next_waypoints[0]
                    print(f"Stored new next_waypoint: {self.next_waypoint.transform.location}")
                else:
                    print("No next waypoints available")

        print(f"\n=== Populate_open() Summary ===")
        print(f"Final OPEN queue size: {self.OPEN.qsize()}")
        print(f"Visited set V size: {len(self.V)}")
        print("=== End populate_open() ===\n")
        
        return self.OPEN
    
    '''
    def populate_open(self):
        #must hold states that need to be evaluated throughout 
        #this k calc makes sense because when youre doing key calc both are important but process state alr takes into account this
        key = self.cost(self.x0, self.state_space) + self.store_h(self.state_space)
        self.OPEN.put((key, self.state_space))
        print(f"OPEN: {key}")
        self.V.add(self.state_space)
        return self.OPEN

    '''
    def delete(self, state):
        while not self.OPEN.empty():
            self.OPEN.get()
        self.tag[state.id] = "Closed"
    
    def cost(self, wp1, wp2):
        distance = wp1.transform.location.distance(wp2.transform.location)
        print(f"Calculating distance between:")
        print(f"  Waypoint 1: Location({wp1.transform.location.x}, {wp1.transform.location.y}, {wp1.transform.location.z})")
        print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        print(f"  Distance: {distance}")
        print(f"x0: {self.x0}")
        print(f"xt: {self.xt}")
        return distance
    
    def store_h(self, state):
        
        #Euclidean distance between 3D points

        #:param location_1, location_2: 3D points
        if state is None:
            return float('inf')

        if state.id not in self.h:
            
            x = self.xt.transform.location.x - state.transform.location.x
            y = self.xt.transform.location.y - state.transform.location.y
            z = self.xt.transform.location.z - state.transform.location.z
            #test_distance = carla.Location(x=1.0, y=2.0, z=0.0)
            #x = carla.distance(x)
            norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
            print(f"State: Location({state.transform.location.x}, {state.transform.location.y}, {state.transform.location.z})")
            print(f"Self.xt: Location({self.xt.transform.location.x}, {self.xt.transform.location.y}, {self.xt.transform.location.z})")
            #print(f"random: {math.sqrt(x**2 + y**2)}")
            #print(f"test_dist: {norm}")
            """
            heuristic = 0
            self.h[state.id] = heuristic
            print(f"store_h, heuristic: {heuristic}")
            """

            self.h[state.id] = norm 
        return self.h[state.id]
    
    def get_kmin(self):
        if self.OPEN:
            self.populate_open(self.state_space)
            print(f"OPEN: {self.OPEN.empty()}")
            minimum = self.OPEN.get() # Return the tuple with the minimum key value
            print(f'get_kmin: minimum: {minimum[0]}')
            return minimum[0]
        
    def min_state(self):
        if self.OPEN:
            print("min_state")
            self.populate_open(self.state_space)
            minimum = self.OPEN.get()
            print(f'get_kmin, state: key: {minimum[0]}, state: {minimum[1]}')
            return minimum[0], minimum[1] #returns state k with associated key value
        return None, -1
    
    #make sure tags are updated correctly 
    def insert(self, h_new, state):
        if state.id not in self.tag:
            self.tag[state.id] = 'New'
        print(f"h_new: {h_new}")
        #self.checkState(state)
    
        state_tag = self.tag[state.id]
        print(f'state_tag: {state_tag}')
        kmin = self.get_kmin()
    
        if state_tag == 'New':
            kx = h_new
            print(f"kx: {kx}")
        elif state_tag == 'Open':
            kx = min(kmin, h_new)
            print(f"kx: {kx}")
            #may need to repopulate here
            self.delete()
        elif state_tag == 'Closed':
            kx = min(self.h[state.id], h_new)
            print(f"kx: {kx}")

        self.tag[state.id] = 'Open'
        self.OPEN.put((kx, state)) 
        print(f'Inserted state {state} with key {kx}')

    
    def store_h(self, state):
        if state is None:
            return float('inf')

        if state.id not in self.h:
            
            heuristic = state.transform.location.distance(self.xt.transform.location)
            print(f"State: Location({self.state.transform.location.x}, {self.state.transform.location.y}, {self.state.transform.location.z})")
            print(f"Self.xt: Location({self.xt.transform.location.x}, {self.xt.transform.location.y}, {self.xt.transform.location.z})")
            print(f"store_h, heuristic: {heuristic}")
            """
            heuristic = 0
            self.h[state.id] = heuristic
            print(f"store_h, heuristic: {heuristic}")
            """
            self.h[state.id] = heuristic
        return self.h[state.id]
    

    #see how process state goes through obstacle avoidance
    def process_state(self):
        kold, x = self.min_state()
        print(f'process state: kold: {kold}, statex: {x}')

        if x is None: 
            return -1
        
        if x.id == self.xt.id:
            print("goal reached")
            #return -1
        
        self.store_h(x)
        #self.checkState(x)
        #print(f'Checked state: {x}')
        print(f'x: {x}, kold: {kold}')
        #print(f'h: {self.h}') #len of self.h = 0
        #print(f'b: {self.b}')
        if x.id not in self.V: 
            for y in self.children(x):
                #self.checkState(y)  
                print(f'Processing child y: {y}')
                print(f'x.id: {x.id}, self.h[x.id]: {self.h[x.id]}')
                print(f'Cost(x, y): {self.cost(x, y)}')

                self.store_h(y)
                if y.id not in self.tag:
                    self.tag[y.id] = 'New'
            
                print("2")
                h_new = self.h[x.id] + self.cost(x, y)
                print(f"process state: cost(x, y) = {self.cost(x, y)}")
                print(f'h_new: {h_new}')
                if h_new < self.h[y.id]: 
                    self.h[y.id] = h_new
                    self.b[y.id] = x
                    self.insert(h_new, y)

            if kold < self.h[x.id]:  # raised states
                print(f'kold < h[x.id]: {kold} < {self.h[x.id]}')
                for y in self.children(x):
                    #print(f'child: {y}')
                    #self.checkState(y)
                    print("3")
                    heuristic_a = self.h[y.id] + self.cost(y, x)
                    print(f'heuristic_a: {heuristic_a}')
                    print(f"process state: cost(y, x) = {self.cost(y, x)}")
                    if self.h[y.id] <= kold and self.h[x.id] > heuristic_a:
                        self.b[x.id] = y
                        self.h[x.id] = heuristic_a

            if kold == self.h[x.id]:  # lower states 
                print(f'kold == h[x.id]: {kold} == {self.h[x.id]}')
                self.tag[x.id] = 'Closed'
                self.V.add(x)
                for y in self.children(x):
                    # check y
                    #print(f'child: {y}')
                    #self.checkState(y)
                    print("4")
                    bb = self.h[x.id] + self.cost(x, y)
                    #print(f"process state: cost(x, y) = {self.cost(x, y)}")
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
                    #self.checkState(y)
                    print("5")
                    bb = self.h[x.id] + self.cost(x, y)
                    print(f'bb: {bb}')
                    if self.tag[y.id] == 'New' or \
                            (self.b[y.id] == x and self.h[y.id] != bb):
                        self.b[y.id] = x
                        self.insert(bb, y)
                    elif self.b[y.id] != x and self.h[y.id] > bb:
                        #print(f'insert x: {x} with h[x]: {self.h[x.id]}')
                        self.insert(self.h[x.id], x)
                    elif self.b[y.id] != x and self.h[y.id] > bb and \
                        self.tag[y.id] == 'Closed' and self.h[y.id] == kold:
                        #print(f'Insert y: {y} with h[y]: {self.h[y.id]}')
                        self.insert(self.h[y.id], y)
            print("No min")
            #made change here
        return self.min_state()
    
    def modify_cost(self, state):
        x_parent = self.b[state.id]
        print(f'modify_cost: xparent: {x_parent}')
        if self.tag[state.id] == 'Closed':
            print(f'state_space {state} is Closed')
            print("6")
            cost_value = self.h[x_parent.id] + self.cost(state, x_parent)
            print(f"cost(state_space, xparent): {self.cost(state, x_parent)}")
            self.insert(cost_value, state)


    #repair_replan, check
    def modify(self, state):
        #print(f'modify: state_space: {state_space}')
        self.modify_cost(state)
        kmin = self.get_kmin()
        #this could be causing the inf loop because if kmin is never updates it is always greater than h
        while kmin is not None and kmin < self.h[state.id]:          
            kmin, y = self.process_state()
            print(f'process_state returned kmin: {kmin}')
            if kmin is None or kmin >= self.h[state.id] or kmin == -1:
                return -1 
        
        self.Path(y)

    def children(self, state):
        print("\n=== Starting children() function ===")
        print(f"Initial state: {state}")
        
        children = []
        if state is None:
            print("State is None, returning empty children list")
            return children

        next_waypoint = state.next(2.0)
        print(f"Next waypoints: {[wp.transform.location for wp in next_waypoint]}")
        
        if next_waypoint:
            print(f"Adding {len(next_waypoint)} next waypoints to children")
            children.extend(next_waypoint)

        print(f"\nChecking lane changes from current state at {state.transform.location}")
        
        if state.lane_change & carla.LaneChange.Left:
            print("Left lane change is possible")
            left_wp = state.get_left_lane()
            print(f"Left waypoint obtained: {left_wp}")
            
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                print(f"Adding left waypoint: {left_wp.transform.location}")
                children.append(left_wp)
                print(f"Left waypoint: {left_wp.transform.location if left_wp else 'None'}")

        if state.lane_change & carla.LaneChange.Right:
            print("Right lane change is possible")
            right_wp = state.get_right_lane()
            print(f"Right waypoint obtained: {right_wp}")
            
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                print(f"Adding right waypoint: {right_wp.transform.location}")
                children.append(right_wp)
                print(f"Right waypoint: {right_wp.transform.location if right_wp else 'None'}")

        print(f"\nStarting waypoint optimization loop for {len(children)} children")
        for i in range(len(children)):
            print(f"\nProcessing child {i}")
            initial_dist = children[i].transform.location.distance(state.transform.location)
            print(f"Initial distance from state: {initial_dist}")
            
            if children[i].id is not None:
                print(f"Current child ID: {children[i].id}")
                min_dist = float('inf')
                best_waypoint = None
                
                for z in children:
                    curr_dist = children[i].transform.location.distance(z.transform.location)
                    print(f"Distance to other waypoint: {curr_dist}")
                    
                    if curr_dist < min_dist:
                        min_dist = curr_dist
                        best_waypoint = z
                        print(f"New minimum distance found: {min_dist}")
                
                if best_waypoint is not None:
                    print(f"Updating waypoint {i} with closer waypoint at distance {min_dist}")
                    children[i] = best_waypoint

        print(f"\n=== Returning {len(children)} children ===")
        print(f"Final children locations: {[c.transform.location for c in children]}\n")
        return children
    
    def detect_obstacles(self, state_space):
        child_waypoints = self.children(state_space)
        for child in child_waypoints:
            child_location = child.transform.location
            for actor in world.get_actors():
                if actor.id != self.vehicle.id: 
                    actor_location = actor.get_location()
                    if actor_location.distance(child_location) < 5.0:
                        return True         
        return False
    
    def handle_obstacles(self):
        """
        Detect obstacles and handle states accordingly.
        If obstacles are detected, recalculates children and updates the state space.
        """
        if self.detect_obstacles(self.state_space):
            print("Obstacle detected.")
        
            child_waypoints = self.children(self.state_space)
            for child in child_waypoints:
                self.delete(child)
                self.modify(child)  # modify for deleted child
            
            if not self.OPEN.empty():
                #self.populate_open()
                kold, x = self.min_state()
                self.state_space = x
                next_waypoints = self.children(self.state_space)
                
                for waypoint in next_waypoints:
                    self.modify_cost(waypoint)  # Modify_cost for new waypoints
                
                print(f"Next wps calculated: {[wp.transform.location for wp in next_waypoints]}")
            
        else:
            print("No obstacles detected")

    #backpointer list 
    def path(self, state):
        self.Path = [state]
        search_state = state
    
        while search_state != self.xt and search_state is not None:
            if search_state.id not in self.b:
                return []  # No path exists
            next_state = self.b[search_state.id]
            self.Path.append(next_state)
            search_state = next_state 
        return self.Path


    #controls vehicle to be moved from one place to another
    
    def move_vehicle(self):
        if not self.vehicle:
            #print("Vehicle not initialized.")
            return

        if self.Path:
            next_waypoint = self.Path[0]
            location = carla.Location(next_waypoint.transform.location.x, next_waypoint.transform.location.y, next_waypoint.transform.location.z)
            self.vehicle.set_location(location)
        else:
            print("Path empty.")


    def visualize_path(self, path):
        if not path:
            return
        debug = self.world.debug
        for i in range(len(path)-1):
            start = path[i]
            end = path[i+1]
            debug.draw_line(
                start.transform.location,
                end.transform.location,
                thickness=0.1,
                color=carla.Color(r=255, g=0, b=0),
                life_time=15.0
            )

    
    def run(self):
        self.state_space = self.x0
        self.tag[self.x0.id] = 'New'
        self.populate_open(self.x0)

        while self.state_space != self.xt:
            if self.OPEN.empty():
                print("No path found.")
                return

            _, current_state = self.min_state()
            self.state_space = current_state

            if self.state_space == self.xt:
                break

            self.process_state()

            if self.detect_obstacles(self.state_space):
                self.modify(self.state_space)
                self.delete(self.state_space)
                self.populate_open(self.state_space)

            self.tag[self.state_space.id] = 'Closed'

            current_path = self.path(self.state_space)
            self.visualize_path(current_path)

        self.Path = self.path(self.state_space)
        print("Path found!")
        self.visualize_path(self.Path)

        for waypoint in self.Path:
           self.move_vehicle(waypoint)

        #exit program

    def move_vehicle(self):
        if not self.vehicle:
            #print("Vehicle not initialized.")
            return

        if self.Path:
            next_waypoint = self.Path[0]
            location = carla.Location(next_waypoint.transform.location.x, next_waypoint.transform.location.y, next_waypoint.transform.location.z)
            self.vehicle.set_location(location)
        else:
            print("Path empty.")
    
    '''
    def visualize_path(self, path):
        debug = self.world.debug
        for segment in path:
            self.x0, self.xt = segment
            debug.draw_line(
                carla.Location(x=self.x0.transform.location.x, y=self.x0.transform.location.y, z=self.x0.transform.location.z),
                carla.Location(x=self.xt.transform.location.x, y=self.xt.transform.location.y, z=self.xt.transform.location.z),
                thickness=0.1, color=carla.Color(r=255, g=0, b=0), life_time=15.0
            )
    '''
    
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
    print(f"Start waypoint: {start_waypoint}")

    end_waypoint = carla_map.get_waypoint(point_b.location)
    print(f"End waypoint: {end_waypoint}")

    d_star = D_star(waypoint=start_waypoint, 
                start_waypoint=start_waypoint, 
                end_waypoint=end_waypoint, 
                vehicle=firetruck, 
                world=world, 
                map=carla_map)

        
    
    if d_star.state_space in d_star.V:
        d_star.V.remove(d_star.state_space)
    
    d_star.run()

    if d_star.state_space.transform.location == (d_star.xt.transform.location):
        print("Goal reached!")
        

        print(f"Current state: {d_star.state_space.transform.location}")
        print(f"Open queue size: {d_star.OPEN.qsize()}")
        print(f"Closed set size: {len(d_star.V)}")
    
    firetruck.destroy()