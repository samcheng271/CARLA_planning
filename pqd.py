import heapq
import carla
import numpy as np
import random
from collections import defaultdict
from queue import PriorityQueue
 
#functions that could be causing issues: 1. populate_open, insert, modify/modify_cost, run
#^functions are logically correct but waypoint handling maybe causing errors 
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
        self.waypoints = self.map.generate_waypoints(self.resolution)
        print(f"Number of waypoints generated: {len(self.waypoints)}")
        self.h = {}
        self.x0 = start_waypoint
        print(f'x0: {self.x0}')
        self.xt = end_waypoint
        print(f'xt: {self.xt}')

    #waypoint comparison error in priority queue, same waypoints are used again->same 
    # keys are used which causes waypoints to be compared instead in the tuple->not 
    # allowed in priority queue calculations thus error occurs
    
    #this function is good 
    def populate_open(self):
        
        #if self.next_waypoint:
            #child_waypoints = [self.next_waypoint]
            #self.next_waypoint = None
        #else:
            #child_waypoints = self.children(self.state_space)
        
        
        child_waypoints = self.children(self.state_space)
        print(f"Total child waypoints found: {len(child_waypoints)}")
        #call funciton here that takes into consideration obstalce avoidance
    
        for child in child_waypoints:
            if child in self.V:
                print(f"Alr processed waypoint: {child.transform.location}")
                continue

            print(f"Processing child waypoint: {child.transform.location}")

            key = self.cost(self.state_space, child)
            #when items that are correclty getting accessed we don't need to check for existing items
            #temp fix 
            existing_items = [item for item in self.OPEN.queue if abs(item[0] - key) < 1e-6]

            if not existing_items:
                self.OPEN.put((key, child))
                print(f"OPEN: {child} key: {key}")
                self.V.add(child)
                #print(f"V: {self.V}")
            else:
                print(f"Key {key} exists")
                next_waypoints = self.children(child)

                if next_waypoints:
                    self.next_waypoint = next_waypoints[0]
                    print(f"Next waypoint: {self.next_waypoint.transform.location}")
                else:
                    print("No next waypoint")

        #OPEN should always have max 5 points
        print(f"OPEN queue size: {self.OPEN.qsize()}")
    
        return self.OPEN
    
    
   # Checks if a 'y' state has both heuristic and tag dicts
    def checkState(self, y):
        if y.id not in self.h:
            #heuristic = self.store_h(y)
            #self.h[y.id] = heuristic 
            self.h[y.id] = 0
            print(f'Heuristic for state {y}: {self.h[y.id]}')
        else:
            print(f'Heuristic for state {y}: {self.h[y.id]}')

        if y.id not in self.tag:
            self.tag[y.id] = 'New'
            print(f'Tag for state {y}: {self.tag[y.id]}')
    
    def get_kmin(self):
        if self.OPEN:
            #self.populate_open()
            print(f"OPEN: {self.OPEN.empty()}")
            minimum = self.OPEN.get() # Return the tuple with the minimum key value
            print(f'get_kmin: minimum: {minimum[0]}')
            return minimum[0]
        
    def min_state(self):
        if self.OPEN:
            print("min_state")
            #self.populate_open()
            minimum = self.OPEN.get()
            print(f'get_kmin, state: key: {minimum[0]}, state: {minimum[1]}')
            return minimum[0], minimum[1] #returns state k with associated key value
        return None, -1
    
    #make sure tags are updated correctly 
    def insert(self, h_new, state):
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
        elif state_tag == 'Closed':
            kx = min(self.h[state.id], h_new)
            print(f"kx: {kx}")

        self.h[state.id] = h_new
        self.tag[state.id] = 'Open'
        self.OPEN.put((kx, state)) 
        print(f'Inserted state {state} with key {kx}')
    """
    def store_h(self, state):
        if state is None:
            return float('inf')

        if state.id not in self.h:
            
            #heuristic = state.transform.location.distance(self.xt.transform.location)
            #print(f"State: Location({self.state.transform.location.x}, {self.state.transform.location.y}, {self.state.transform.location.z})")
            #print(f"Self.xt: Location({self.xt.transform.location.x}, {self.xt.transform.location.y}, {self.xt.transform.location.z})")
            #print(f"store_h, heuristic: {heuristic}")
            
            heuristic = 0
            self.h[state.id] = heuristic
            print(f"store_h, heuristic: {heuristic}")
        return self.h[state.id]
    """
    #see how process state goes through obstacle avoidance
    def process_state(self):
        kold, x = self.min_state()
        print(f'process state: kold: {kold}, statex: {x}')

        if x is None: 
            return -1
        
        if x.id == self.xt.id:
            print("goal reached")
            #return -1
        
        self.delete(x)

        #self.store_h(x)
        #self.checkState(x)
        #print(f'Checked state: {x}')
        print(f'x: {x}, kold: {kold}')
        print(f'h: {self.h}') #len of self.h = 0
        print(f'b: {self.b}')
        if x not in self.V:
            """
            for y in self.children(x):
                self.checkState(y)  
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
            """
            if kold < self.h[x.id]:  # raised states
                print(f'kold < h[x.id]: {kold} < {self.h[x.id]}')
                for y in self.children(x):
                    #print(f'child: {y}')
                    self.checkState(y)
                    print("3")
                    heuristic_a = self.h[y.id] + self.cost(y, x)
                    print(f'heuristic_a: {heuristic_a}')
                    print(f"process state: cost(y, x) = {self.cost(y, x)}")
                    if self.h[y.id] <= kold and self.h[x.id] > heuristic_a:
                        self.b[x.id] = y
                        self.h[x.id] = heuristic_a

            if kold == self.h[x.id]:  # lower states 
                print(f'kold == h[x.id]: {kold} == {self.h[x.id]}')
                #self.tag[x.id] = 'Closed'
                #self.V.add(x)
                for y in self.children(x):
                    # check y
                    #print(f'child: {y}')
                    self.checkState(y)
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
                    self.checkState(y)
                    print("5")
                    bb = self.h[x.id] + self.cost(x, y)
                    print(f'bb: {bb}')
                    if self.tag[y.id] == 'New' or \
                            (self.b[y.id] == x and self.h[y.id] != bb):
                        self.b[y.id] = x
                        self.insert(bb, y)
                    else: 
                        if(self.b[y.id] != x and self.h[y.id] > bb):
                            #print(f'insert x: {x} with h[x]: {self.h[x.id]}')
                            self.insert(self.h[x.id], x)
                        else:
                            if(self.b[y.id] != x and self.h[y.id] > bb and \
                            self.tag[y.id] == 'Closed' and self.h[y.id] == kold):
                            #print(f'Insert y: {y} with h[y]: {self.h[y.id]}')
                                self.insert(self.h[y.id], y)
            print("No min")
        return self.get_kmin()

    #modify_cost/modify are both logically correct acc to D*, but parent/pred 
    # calculations need to be rechecked 
    def modify_cost(self, state):
        ss_id = state.id
        
        xparent = self.b[ss_id]
        print(f'modify_cost: xparent: {xparent}')
        if self.tag[ss_id] == 'Closed':
            print(f'state_space {state} is Closed')
            print("6")
            cost_value = self.h[xparent.id] + self.cost(state, xparent)
            print(f"cost(state_space, xparent): {self.cost(state, xparent)}")
            self.insert(cost_value, state)

    #repair_replan, check
    def modify(self, state):
        #print(f'modify: state_space: {state_space}')
        self.modify_cost(state)
        kmin = self.get_kmin()
        while kmin is not None and kmin < self.h[state.id]:          
            kmin = self.process_state()
            print(f'process_state returned kmin: {kmin}')
            if kmin is None or kmin >= self.h[state.id] or kmin == -1:
                return -1 
        self.Path()
    

    def cost(self, wp1, wp2):
        distance = wp1.transform.location.distance(wp2.transform.location)
        print(f"Calculating distance between:")
        print(f"  Waypoint 1: Location({wp1.transform.location.x}, {wp1.transform.location.y}, {wp1.transform.location.z})")
        print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        print(f"  Distance: {distance}")
        return distance
    
    def delete(self, state):
        self.OPEN.remove(state)
        self.tag[state.id] = "CLOSED"

    def children(self, state):
        children = []
        if state is None:
            return children

        next_waypoint = state.next(2.0)
        print(f"Next waypoints: {[wp.transform.location for wp in next_waypoint]}")
        if next_waypoint:
            """
            for wp in next_waypoint:
                if wp.transform.location not in [c.transform.location for c in children]:
            """
            children.extend(next_waypoint)

        if state.lane_change & carla.LaneChange.Left:
            left_wp = state.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                #if left_wp.transform.location not in [c.transform.location for c in children]:
                    children.append(left_wp)
                    print(f"Left waypoint: {left_wp.transform.location if left_wp else 'None'}")
            """
                diag_left_wp = left_wp.next(2.0)
                if diag_left_wp:
                    for wp in diag_left_wp:
                        if wp.transform.location not in [c.transform.location for c in children]:
                            children.append(wp)
            """

        if state.lane_change & carla.LaneChange.Right:
            right_wp = state.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                #if right_wp.transform.location not in [c.transform.location for c in children]:
                    children.append(right_wp)
                    print(f"Right waypoint: {right_wp.transform.location if right_wp else 'None'}")
            """
                diag_right_wp = right_wp.next(2.0)
                if diag_right_wp:
                    for wp in diag_right_wp:
                        if wp.transform.location not in [c.transform.location for c in children]:
                            children.append(wp)
            """

        return children  

    def detect_obstacles(self, state_space):
        child_waypoints = self.children(state_space)
        for child in child_waypoints:
            child_location = child.transform.location
            for actor in world.get_actors():
                if actor.id != self.vehicle.id: 
                    actor_location = actor.get_location()
                    if actor_location.distance(child_location) < 5.0:
                        obstacle_found = True
                        break

        return obstacle_found
    
    def handle_obstacles(self):
        """
        Detect obstacles and handle states accordingly.
        If obstacles are detected, recalculates children and updates the state space.
        """
        if self.detect_obstacles():
            print("Obstacle detected.")
        
            child_waypoints = self.children(self.state_space)
            for child in child_waypoints:
                self.delete(child)
                self.modify(child)  # odify for deleted child
            
            if not self.OPEN.empty():
                #self.populate_open()
                kold, x = self.min_state()
                self.state_space = x
                next_waypoints = self.children(self.state_space)
                
                for waypoint in next_waypoints:
                    self.modify_cost(waypoint)  # Modify_cost for new waypoints
                
                print(f"Next waypoints recalculated: {[wp.transform.location for wp in next_waypoints]}")
            
        else:
            print("No obstacles detected")

    #backpointer list 
    def path(self):
        #goal = self.xt
        self.Path = [self.state_space]
        
        """
        if not goal:
            trace_state = self.x0
            print(f'No goal provided, using x0: {trace_state}')
        else:
            trace_state = goal
            print(f'Goal provided: {self.goal_location}, start: {self.x0}')
        #start = self.xt
        """
        while self.state_space != self.x0 and self.state_space != None:
            x = self.b[self.state_space.id]
            #trace_location = np.array([trace_state.transform.location.x, trace_state.transform.location.y, trace_state.transform.location.z])
            #parent_location = np.array([x.transform.location.x, x.transform.location.y, x.transform.location.z])
            self.Path.append(x)
            #trace_state = x
            print(f"path append: {x}")
        if x != self.xt:
            self.Path = []
        
    
    
    def run(self):
        #cost_value = self.cost(self.x0, self.xt)
        #self.OPEN.put((cost_value, self.x0))
        #self.OPEN.put((0, self.xt))
        #self.tag[self.x0] = 'New'
        self.h[self.xt.id] = 0
        self.insert(self.h[self.xt.id], self.xt)



        while self.tag.get(self.state_space.id, 'New') != "Closed":
            self.process_state()
            """
            if not self.OPEN.empty():
                _, new_state_space = self.OPEN.get()
                self.state_space = new_state_space
                print(f"Updated state_space to: {self.state_space.transform.location}")
            """
            if self.tag[self.x0.id] == 'Closed':
                break
            self.ind += 1
        self.Path = self.path()
        print(f'Path found: {self.Path}')
        self.done = True
        self.visualize_path(self.Path)

        while self.state_space != self.xt:
            self.handle_obstacles()
            self.modify(self.state_space)
            self.path()
            self.visualize_path(self.Path)
            self.move_vehicle()

            if self.path == []:
                print("No path is found")
                return
            

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
        debug = self.world.debug
        for segment in path:
            self.x0, self.xt = segment
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
        D = D_star(start_waypoint, start_waypoint, end_waypoint, firetruck, world, carla_map)
        D.run()

    finally:
        firetruck.destroy() 
        print("Firetruck destroyed")