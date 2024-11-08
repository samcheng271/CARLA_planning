import carla
import numpy as np
import random
import math
from queue import PriorityQueue

#consider move-robot-how does move-robot compare to move_vehicle 
#add all the elements of populate open in to a list and based on the list, iterate and return the value needed in process state
#use that to calculate cost not the actual cost calclaution
#refer to original code 
class D_star(object):
    #def __init__(self, waypoint, start_location, goal_location, vehicle, world, map, resolution=2.0):
    def __init__(self, waypoint, start_waypoint, end_waypoint, vehicle, world, map, resolution=2.0):
        self.settings = 'CollisionChecking'
        self.resolution = resolution
        self.waypoint = waypoint
        self.obstacle_threshold = 3.0
        self.b = {}
        self.OPEN = []
        
        self.tag = {}
        self.V = set()
        self.parameters = ()
        self.ind = 0
        self.Path = []
        #self.hold_wps = []
        self.done = False
        self.Obstaclemap = {}

        #1. Should world map and stuff be in init
        self.world = world
        self.map = world.get_map()
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
        print(f"PO state: {state}")
        '''
        if self.next_waypoint:
            child_waypoints = [self.next_waypoint]
            self.next_waypoint = None
        else:
            print("No stored next_waypoint")
            child_waypoints = self.children(state)
        '''
        child_waypoints = self.children(state)
        print(f"Child waypoint locations: {[wp.transform.location for wp in child_waypoints]}")

        for child in child_waypoints:
            print(f"Child location: {child.transform.location}")
            
            if child in self.V:
                print(f"Skipping already processed waypoint: {child.transform.location}")
                continue

            cost = self.cost(state, child)
            h_value = self.store_h(child)
            key = cost + h_value
            print(f"Cost: {cost}, h-value: {h_value}, Total key: {key}")

            self.b[child.id] = state
            print(f"self.b[child.id]: {self.b[child.id]}")
            existing_items = next((key for key, wp in self.OPEN if wp == child), None)
            if not existing_items:
                print(f"No similar keys found, adding to OPEN queue with key: {key}")
                self.OPEN.append((key, child))
                self.OPEN.sort(key=lambda x: x[0])
                #self.hold_wps.append((key, child))
                print(f"Added to OPEN: Waypoint at {child.transform.location} with key {key}")
                self.V.add(child)
            else:
                print("Generating next waypoints for current child")
                next_waypoints = self.children(child)

                if next_waypoints:
                    self.next_waypoint = next_waypoints[0]
                    print(f"Stored new next_waypoint: {self.next_waypoint.transform.location}")
                else:
                    print("No next waypoints available")

        print(f"Final OPEN queue size: {'empty' if self.OPEN == [] else 'not empty'}")
        print(f"Visited set V size: {len(self.V)}")
       
        return self.OPEN

    
    def delete(self, state):
        while len(self.OPEN) > 0:
            self.OPEN.pop(0)
        self.tag[state.id] = "Closed"
    

    def cost(self, wp1, wp2):
        distance = wp1.transform.location.distance(wp2.transform.location)
        print(f"Calculating distance between:")
        print(f"  Waypoint 1: Location({wp1.transform.location.x}, {wp1.transform.location.y}, {wp1.transform.location.z})")
        print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        print(f"  Distance: {distance}")
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
        if len(self.OPEN) > 0 and self.OPEN[0]:
            first_element = self.OPEN[0][0]
            return first_element
        else:
            print("The list or tuple is empty")
        
    def min_state(self):
        if len(self.OPEN) > 0 and self.OPEN[0]:
            #self.populate_open(self.state_space)
            minimum = self.OPEN[0]
            print(f'get_kmin, state: key: {minimum}')
            return minimum
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
            if h_new == kx:
                self.delete()
        elif state_tag == 'Closed':
            kx = min(self.h[state.id], h_new)
            print(f"kx: {kx}")

        self.tag[state.id] = 'Open'
        self.OPEN.append((kx, state)) 
        print(f'Inserted state {state} with key {kx}')

    #here, the cost is getting accessed multiple times because of the cost call in mulitple areas 
    #create logic so that it only calculates the access for multiple types 
    #see how process state goes through obstacle avoidance
    def process_state(self):
        kold, x = self.min_state()
        print(f'process state: kold: {kold}, statex: {x}')

        if x is None: 
            return -1
        
        if x.id == self.xt.id:
            print("goal reached")
            #return -1

        #recheck this
        """
        if x in self.V:
            print(f"Skipping already processed waypoint: {x.transform.location}")
        """
            
        self.store_h(x)
        #self.checkState(x)
        #print(f'Checked state: {x}')
        print(f'x: {x}, kold: {kold}')
        #print(f'h: {self.h}') #len of self.h = 0
        #print(f'b: {self.b}')
        if x.id not in self.V: 
            '''
            for y in self.children(x):
                #self.checkState(y)  
                print(f'Processing child y: {y}')
                print(f'x.id: {x.id}, self.h[x.id]: {self.h[x.id]}')
                print(f'Cost(x, y): {self.cost(x, y)}')

                self.store_h(y)
                if y.id not in self.tag:
                    self.tag[y.id] = 'New'
            
                print("2")
                #if y in second element of the list, then I want to use the exisitng cost in the list otherwise, i'll just use the given equation below
                existing_cost = next((key for key, wp in self.OPEN if wp == y), None)
            
                if existing_cost is not None:
                    print(f"Using existing cost for {y}: {existing_cost}")
                    cost = existing_cost
                else:
                    cost = self.cost(x, y)
            
                h_new = self.h[x.id] + cost
                print(f"process state: cost(x, y) = {cost}")
                print(f'h_new: {h_new}')
                if h_new < self.h[y.id]: 
                    self.h[y.id] = h_new
                    self.b[y.id] = x
                    print(f"self.b[y.id]: {self.b[y.id]}")
                    self.insert(h_new, y)
                '''
            if kold < self.h[x.id]:  # raised states
                print(f'kold < h[x.id]: {kold} < {self.h[x.id]}')
                for y in self.children(x):
                    #print(f'child: {y}')
                    #self.checkState(y)
                    print("3")
                    #get the cost from open queue based on selecting the cost with asscoiated y and using that to evaluate instead of self.cost. 
                    existing_cost = next((key for key, wp in self.OPEN if wp == y), None)
            
                    if existing_cost is not None:
                        print(f"Using existing cost for {y}: {existing_cost}")
                        cost = existing_cost
                    else:
                        cost = self.cost(x, y)
            
                    heuristic_a = self.h[x.id] + cost
                    #heuristic_a = self.h[y.id] + self.cost(y, x)
                    print(f'heuristic_a: {heuristic_a}')
                    print(f"process state: cost(y, x) = {self.cost(y, x)}")
                    if self.h[y.id] <= kold and self.h[x.id] > heuristic_a:
                        self.b[x.id] = y
                        print(f"rs, y.id: {y.id}")
                        print(f"self.b[x.id]: {self.b[x.id]}")
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
                    existing_cost = next((key for key, wp in self.OPEN if wp == y), None)
            
                    if existing_cost is not None:
                        print(f"Using existing cost for {y}: {existing_cost}")
                        cost = existing_cost
                    else:
                        cost = self.cost(x, y)
            
                    bb = self.h[x.id] + cost
                    #print(f"process state: cost(x, y) = {self.cost(x, y)}")
                    print(f'h[x.id]: {self.h[x.id]}')
                    print(f'print bb: {bb}')
                    if self.tag[y.id] == 'New' or \
                            (self.b[y.id] == x and self.h[y.id] != bb) or \
                            (self.b[y.id] != x and self.h[y.id] > bb):
                        print(f'Insert y: {y} with bb: {bb}')
                        self.b[y.id] = x
                        print(f"ls, x.id: {x.id}")
                        print(f"self.b[y.id]: {self.b[y.id]}")
                        self.insert(bb, y)
            else:
                print(f'kold != h[x.id]: {kold} != {self.h[x.id]}')
                for y in self.children(x):
                    print("5")
                    existing_cost = next((key for key, wp in self.OPEN if wp == y), None)
                    
                    if existing_cost is not None:
                        print(f"Using existing cost for {y}: {existing_cost}")
                        cost = existing_cost
                    else:
                        cost = self.cost(x, y)
                    
                    bb = self.h[x.id] + cost
                    print(f'bb: {bb}')
                    
                    if y.id not in self.tag:
                        self.tag[y.id] = 'New'
                        
                    if self.tag[y.id] == 'New' or \
                        (self.b[y.id] == x and self.h[y.id] != bb):
                        self.b[y.id] = x
                        print(f"else, self.b[y.id]: {self.b[y.id]}")
                        print(f"else, x: {x}")
                        self.insert(bb, y)

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
        print(f"Initial state: {state}")
        
        children = []
        if state is None:
            print("State is None, returning empty children list")
            return children

        next_waypoint = state.next(2.0)
        print(f"Next waypoints: {[(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z) for wp in next_waypoint]}")
        
        if next_waypoint:
            print(f"Adding {len(next_waypoint)} next waypoints to children")
            children.extend(next_waypoint)

        if state.lane_change & carla.LaneChange.Left:
            left_wp = state.get_left_lane()
            print(f"Left_wp: {left_wp}")
            
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                print(f"Adding left waypoint: {left_wp.transform.location}")
                children.append(left_wp)
                print(f"Left waypoint: {(left_wp.transform.location.x, left_wp.transform.location.y, left_wp.transform.location.z) if left_wp else 'None'}")

        if state.lane_change & carla.LaneChange.Right:
            right_wp = state.get_right_lane()
            print(f"Right wp: {right_wp}")
            
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                print(f"Adding right waypoint: {right_wp.transform.location}")
                children.append(right_wp)

        real_points = []
        for child in children:
            projected = carla_map.get_waypoint(child.transform.location, project_to_road=True)
            if projected:
                real_points.append(projected)
        print(f"Projected waypoints: {[(p.transform.location.x, p.transform.location.y, p.transform.location.z) for p in real_points]}")

        for i in range(len(children)):
            if children[i].id is not None:
                initial_dist = float('inf') 
                best_waypoint = children[i]  
                print(f"Evaluating child waypoint {i} at location: ({children[i].transform.location.x}, {children[i].transform.location.y}, {children[i].transform.location.z})")
                
                for projected in real_points:
                    curr_dist = children[i].transform.location.distance(projected.transform.location)
                    print(f"Distance from child to projected waypoint: {curr_dist}")
                    if curr_dist < initial_dist:
                        initial_dist = curr_dist
                        best_waypoint = projected
                        print(f"Updated best waypoint for child {i} to: ({best_waypoint.transform.location.x}, {best_waypoint.transform.location.y}, {best_waypoint.transform.location.z})")

                children[i] = best_waypoint

        print(f"Final children locations: [{[(c.transform.location.x, c.transform.location.y, c.transform.location.z) for c in children]}]")

        # Include draw_string here for all waypoints
        for i in children: 
            print(f"Drawing waypoint at: ({i.transform.location.x}, {i.transform.location.y}, {i.transform.location.z})")
            self.world.debug.draw_string(i.transform.location, '0', draw_shadow=False, 
                                        color=carla.Color(r=220, g=0, b=0), life_time=60.0, 
                                        persistent_lines=True)
        
        return children

    #backpointer list 
    def path(self, state):
        print(f"Starting path search from state: {state}")
        
        Path = [state]
        search_state = state
        
        while search_state != self.xt and search_state is not None:
            #this is going into an infinite loop because the search state and xt are
            #  never same bc of id differences 
            print(f"Current search state: {search_state}")
            
            if search_state.id not in self.b:
                print(f"search_state: {search_state.id} ")
                print(f"print b: {self.b}")
                return [] 
            
            next_state = self.b[search_state.id]
            print(f"Next state found: {next_state}")
            
            Path.append(next_state)
            search_state = next_state

        print(f"Path found: {Path}")

        return Path

    def detect_obstacles(self, state_space):
       
        child_waypoints = self.children(state_space)
        
        for child in child_waypoints:
            child_location = child.transform.location
            
            for actor in world.get_actors():
                if actor.id != self.vehicle.id:
                    actor_location = actor.get_location()
                    distance = actor_location.distance(child_location)

                    if distance < 5.0:
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
                self.modify(child) 
            
            if not self.OPEN:
                kold, x = self.min_state()
                self.state_space = x
                next_waypoints = self.children(self.state_space)
                
                for waypoint in next_waypoints:
                    self.modify_cost(waypoint) 
                
                print(f"Next wps calculated: {[wp.transform.location for wp in next_waypoints]}")
            
        else:
            print("No obstacles detected")

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
        
        for i in range(len(path)-1):
            start = path[i]
            end = path[i+1]
            
            # Draw the connecting line
            self.world.debug.draw_line(
                start.transform.location,
                end.transform.location,
                thickness=0.1,
                color=carla.Color(r=255, g=0, b=0),
                life_time=15.0
            )
            
            # Draw "0" at each location
            self.world.debug.draw_string(
                start.transform.location,
                "0",
                draw_shadow=True,
                color=carla.Color(r=0, g=255, b=0),
                life_time=15.0
            )
            
            # Draw point marker
            self.world.debug.draw_point(
                start.transform.location,
                size=0.1,
                color=carla.Color(r=0, g=255, b=0),
                life_time=15.0
            )
        
        self.world.debug.draw_string(
            path[-1].transform.location,
            "0",
            draw_shadow=True,
            color=carla.Color(r=0, g=0, b=255),
            life_time=15.0
        )
        
        self.world.debug.draw_point(
            path[-1].transform.location,
            size=0.2,
            color=carla.Color(r=0, g=0, b=255),
            life_time=15.0
        )

    def run(self):
        try:
            self.state_space = self.x0
            self.tag[self.x0.id] = 'New'
            
            cost = self.cost(self.x0, self.xt)
            h_value = self.store_h(self.x0)
            if cost == h_value:
                h_value = 0

            key = cost + h_value
            self.OPEN.put((key, self.state_space)) 
   
            self.populate_open(self.x0)

            while self.state_space != self.xt:
                if self.OPEN == []:
                    print("No path found.")
                    return None
                _, current_state = self.min_state()
                self.state_space = current_state
                
                if self.state_space == self.xt:
                    break

                self.process_state()

                if self.detect_obstacles(self.state_space):
                    try:
                        self.modify(self.state_space)
                        self.delete(self.state_space)
                        if self.state_space.id in self.tag:
                            self.populate_open(self.state_space)
                    except Exception as e:
                        print(f"Error handling obstacle: {e}")
                        continue

                self.tag[self.state_space.id] = 'Closed'


                #self.Path = self.path(self.state_space)
                #print("Path found!")
                #self.visualize_path(self.Path)

                # Execute path with safety checks
                '''
                for waypoint in self.Path:
                    if not self.detect_obstacles(waypoint):
                        #self.move_vehicle(waypoint)
                    else:
                        '''
                    #print("Obstacle detected during execution, replanning needed")
                return self.run() 

                #return self.Path

        except Exception as e:
            print(f"Error during path planning: {e}")
            return None
    
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

    world.debug.draw_string(start_waypoint.transform.location, 'START', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    world.debug.draw_string(end_waypoint.transform.location, 'END', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
   
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