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
        self.dictofParents = {}
        self.OPEN = []
        self.tag = {}
        self.V = set()
        self.Path = []
        self.hold_wps = []
        self.Obstaclemap = {}
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
        #self.state_space = self.x0 
        self.xt = end_waypoint
    
    def delete(self, state):
        while len(self.OPEN) > 0:
            self.OPEN.pop(0)
        self.tag[state.id] = "Closed"

    def cost(self, wp1, wp2):
        distance = wp1.transform.location.distance(wp2.transform.location)
        print(f"Calculating distance between:")
        print(f"  Waypoint 1: Location({wp1.transform.location.x}, {wp1.transform.location.y}, {wp1.transform.location.z})")
        print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        print(f"wp1_id: {wp1.id}")
        print(f"wp2_id: {wp2.id}")
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
            norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
            print(f"State: Location({state.transform.location.x}, {state.transform.location.y}, {state.transform.location.z})")
            print(f"Self.xt: Location({self.xt.transform.location.x}, {self.xt.transform.location.y}, {self.xt.transform.location.z})")
            self.h[state.id] = norm 
        return self.h[state.id]

    def get_kmin(self):
        if len(self.OPEN) > 0 and self.OPEN[0]:
            key_val = self.OPEN[0][0]
            print(f"kmin, kval: {key_val}")
            return key_val
        else:
            print("The list or tuple is empty")
        
    def min_state(self):
        if len(self.OPEN) > 0 and self.OPEN[0]:
            minimum = self.OPEN[0][1]
            print(f'min_state: {minimum}')
            return minimum
        return None, -1
    
    def insert(self, h_new, state):
        if state.id not in self.tag:
            self.tag[state.id] = 'New'
        print(f"h_new: {h_new}") 
        state_tag = self.tag[state.id]
        print(f'state_tag: {state_tag}')
        kmin = self.get_kmin()
        if state_tag == 'New':
            kx = h_new
            print(f"kx: {kx}")
        elif state_tag == 'Open':
            kx = min(kmin, h_new)
            print(f"kx: {kx}")
            '''
            if h_new == kx:
                self.delete()
            '''
        elif state_tag == 'Closed':
            kx = min(self.h[state.id], h_new)
            print(f"kx: {kx}")
        self.tag[state.id] = 'Open'
        self.h[state.id]
        self.OPEN.append((kx, state)) 

    """
    def populate_open(self, state):
        print(f"PO state: {state}")
        
        if self.next_waypoint:
            child_waypoints = [self.next_waypoint]
            self.next_waypoint = None
        else:
            print("No stored next_waypoint")
            child_waypoints = self.children(state)
        
        for child in child_waypoints:
            print(f"Child location: {child.transform.location}")
            
            if child in self.V:
                print(f"Skipping already processed waypoint: {child.transform.location}")
                continue

            cost = self.cost(state, child)
            h_value = self.store_h(child)
            key = cost + h_value
            existing_items = next((key for key, wp in self.OPEN if wp == child), None)

            if not existing_items:
                self.OPEN.append((key, child))
                self.OPEN.sort(key=lambda x: x[0])
                print(f"Added to OPEN: Waypoint at {child.transform.location} with key {key}")
                self.V.add(child)
            else:
                next_waypoints = self.children(child)
                if next_waypoints: 
                    self.next_waypoint = next_waypoints[0]
                    print(f"Stored new next_waypoint: {self.next_waypoint.transform.location}")
                else:
                    print("No next waypoints available")

        print(f"OPEN queue size: {'empty' if self.OPEN == [] else 'not empty'}")
        print(f"Visited: {len(self.V)}")
       
        for i in range(len(self.OPEN)):
            location = self.OPEN[i][1].transform.location
            self.world.debug.draw_string(location, f'{i}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

            if i < len(self.OPEN) - 1:
                next_location = self.OPEN[i + 1][1].transform.location
                self.world.debug.draw_line(location, next_location, thickness=0.1, color=carla.Color(r=255, g=255, b=0), life_time=60.0, persistent_lines=True)

        return self.OPEN
     """
    def process_state(self):
        currState = self.min_state()
        kold = self.get_kmin()
        print(f' kold: {kold}, statex: {currState}')
        if currState is None: 
            return -1
        #kold= heuristic + cost
        #self.h[currState.id]= only heuristic, for this reason kold will always be greater than h(currState) and first two conditions are never met 
       #self.store_h(currState)
        print(f"before loop, h dict: {self.h[currState.id]}")
        if currState.id not in self.V: 
            if kold < self.h[currState.id]:  # raised states
                print(f'kold < h[x.id]: {kold} < {self.h[currState.id]}')
                for child in self.children(currState):
                    #elf.store_h(y) 
                    x = self.xt.transform.location.x - child.transform.location.x
                    y = self.xt.transform.location.y - child.transform.location.y
                    z = self.xt.transform.location.z - child.transform.location.z
                    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
                    cost = self.cost(currState, child)
                    heuristic_a = norm + cost 
                    print(f'heuristic_a: {heuristic_a}')
                    #self.dictofParents[child.id] = currState
                    if self.h[child]<=kold and self.h[currState.id] > heuristic_a:
                        self.dictofParents[child.id] = currState
                        print(f"self.dictofParents[x.id]: {self.dictofParents[currState.id]}")
                        self.h[child.id] = heuristic_a
            if kold == self.h[currState.id]:  # lower states 
                print(f'kold == h[x.id]: {kold} == {self.h[currState.id]}')
                #on the last iter, make this tag closed and V closed 
                self.tag[currState.id] = 'Closed'
                self.V.add(currState)
                for child in self.children(currState):
                    x = self.xt.transform.location.x - child.transform.location.x
                    y = self.xt.transform.location.y - child.transform.location.y
                    z = self.xt.transform.location.z - child.transform.location.z
                    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
                    cost = self.cost(currState, child)
                    bb = norm + cost
                    self.dictofParents[child.id] = currState 
                    print(f"currState insert in dop: {currState}")
                    if self.tag[child.id] =='New' or(self.dictofParents[child.id] ==currState and self.h[child.id] != bb) or\
                        (self.dictofParents[child.id] !=currState and self.h[child.id] > bb): 
                        print(f'Insert y: {child} with {bb}')
                        self.dictofParents[child.id] = currState
                        print(f"ls, x.id: {currState.id}")
                        print(f"self.dictofParents[y.id]: {self.dictofParents[child.id]}")
                        self.insert(bb, child)
            else:
                print(f'kold >h[x.id]: {kold} >{self.h[currState.id]}')
                #1. check what values are getting inserted from insert to min state bc that is ht ecurrent state in run that modify cant find 
                #2. States are not correctly inserted into OPEN, it is inserting the current state instead of it's neighbors 
                for child in self.children(currState):
                    x = self.xt.transform.location.x - child.transform.location.x
                    y = self.xt.transform.location.y - child.transform.location.y
                    z = self.xt.transform.location.z - child.transform.location.z
                    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
                    cost = self.cost(currState, child)
                    bb = norm + cost
                    print(f'bb: {bb}')
                    if child.id not in self.tag: 
                        self.tag[child.id] = 'New'
                    print(f"currState insert in dop: {currState}")
                    if self.tag[child.id]== 'New' or (self.dictofParents[currState.id] == child and self.h[child.id] != bb):
                        self.dictofParents[currState.id] = child
                        self.h[child.id] = bb 
                        self.insert(bb, child)
        return self.min_state()
    
    def modify_cost(self, state):
        #state is not in dictofParents but it should be bc dictofParents has 
        #checking for parent of curr min state return from ps 
        print(f"mc_state.id: {state.id}")
        self.dictofParents[state.id] = state
        x_parent = self.dictofParents[state.id]
        print(f'xparent: {x_parent}')
        if self.tag[state.id] == 'Closed':
            cost_value = self.h[x_parent.id] + self.cost(state, x_parent)
            print(f"cost(state_space, xparent): {self.cost(state, x_parent)}")
            self.insert(cost_value, state)

    def modify(self, state):
        #print(f'modify: state_space: {state_space}')
        self.modify_cost(state)
        #while kmin is not None and kmin != self.h[self.goal.id]: 
        kmin = self.get_kmin()
        while kmin is not None and state != self.xt: 
            #two vals with the same vars         
            low_min = self.process_state()
            print(f'process_state returned kmin: {low_min}')
            kmin = self.get_kmin()
            if kmin is None or kmin >= self.h[state.id]:
                return -1 
        self.path(low_min) 

    #repair_replan, check
    def children(self, state):
        children = []
        if state is None:
            return children
        next_waypoint = state.next(2.0)
        if next_waypoint:
            children.extend(next_waypoint)
        if state.lane_change & carla.LaneChange.Left:
            left_wp = state.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                children.append(left_wp)
        if state.lane_change & carla.LaneChange.Right:
            right_wp = state.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                children.append(right_wp)
        """
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
                for projected in real_points:
                    curr_dist = children[i].transform.location.distance(projected.transform.location)
                    if curr_dist < initial_dist:
                        initial_dist = curr_dist
                        best_waypoint = projected
                        print(f"Updated best waypoint: ({best_waypoint.transform.location.x}, {best_waypoint.transform.location.y}, {best_waypoint.transform.location.z})")
                children[i] = best_waypoint
        
        print(f"Child list: [{[(c.transform.location.x, c.transform.location.y, c.transform.location.z) for c in children]}]")
        for i in children: 
            print(f"Drawing waypoint at: ({i.transform.location.x}, {i.transform.location.y}, {i.transform.location.z})")
            self.world.debug.draw_string(i.transform.location, '0', draw_shadow=False, 
                                        color=carla.Color(r=220, g=0, b=0), life_time=60.0, 
                                        persistent_lines=True)
        """
        return children

    #backpointer list 
    def path(self, state):
        print(f"Starting path search from state: {state}")
        if not state:
            return []
        path_list = []
        curr = state
        while True:
            path_list.append(curr)
            if curr.id not in self.dictofParents:
                break
            par = self.dictofParents[curr.id]
            if (not par) or (par.id == curr.id):
                break
            curr = par
            if curr == self.x0:
                path_list.append(curr)
                break
        
        path_list.reverse()

        for i in range(len(path_list) - 1):
            wp1 = path_list[i].transform.location
            wp2 = path_list[i + 1].transform.location
            self.world.debug.draw_line(wp1, wp2, thickness=0.2, 
                                       color=carla.Color(255, 0, 0), life_time=10.0)
        
        self.Path = path_list
        return path_list
        
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
                x = self.min_state()
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
        """
        Visual debugging of the path
        """
        if not path:
            return
        for i in range(len(path) - 1):
            start = path[i].transform.location
            end = path[i + 1].transform.location
            # Draw line
            self.world.debug.draw_line(start, end, thickness=0.1,
                                       color=carla.Color(r=255, g=0, b=0),
                                       life_time=15.0)
            # Mark each location
            self.world.debug.draw_string(start, "0", draw_shadow=True,
                                         color=carla.Color(r=0, g=255, b=0),
                                         life_time=15.0)
        # Mark the final node
        final_wp = path[-1].transform.location
        self.world.debug.draw_string(final_wp, "END?", draw_shadow=True,
                                     color=carla.Color(r=0, g=0, b=255),
                                     life_time=15.0)

    def run(self):
        try:
            self.state_space = self.x0
            self.tag[self.x0.id] = 'New'
            
            cost = self.cost(self.x0, self.xt)
            h_value = self.store_h(self.x0)
            if cost == h_value:
                h_value = 0

            key = cost + h_value
            self.OPEN.append((key, self.state_space)) 
            #this line could be why only the initial state is considered, maybe ms instead of ps but that could still lead to the same issues 
            current_state = self.min_state()
            print(f"current state: {current_state}")
            while current_state != self.xt:
                if self.OPEN == []:
                    print("No path found.")
                    return None
                #current_state = self.min_state()
                #self.state_space = current_state
                if current_state == self.xt:
                    break  
                #self.populate_open(self.state_space)
                #current_state = self.process_state()
                self.modify(current_state)
                for i in range(len(self.OPEN)):
                    location = self.OPEN[i][1].transform.location
                    self.world.debug.draw_string(location, f'new:{i}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

                if i < len(self.OPEN) - 1:
                    next_location = self.OPEN[i + 1][1].transform.location
                    self.world.debug.draw_line(location, next_location, thickness=0.1, color=carla.Color(r=255, g=255, b=0), life_time=60.0, persistent_lines=True)
                #self.path(current_state)
                if self.detect_obstacles(current_state):
                    try:
                        #self.modify(self.state_space)
                        self.delete(current_state)
                        if current_state.id in self.tag:
                            self.modify_cost(current_state)
                    except Exception as e:
                        print(f"Error handling obstacle: {e}")
                        continue
                self.tag[current_state.id] = 'Closed'
                self.Path = self.path(current_state)
                print("Path found!")
                self.visualize_path(self.Path)
                # Execute path with safety checks
                for waypoint in self.Path:
                    if not self.detect_obstacles(waypoint):
                        self.move_vehicle()
                    else:
                        print("Obstacle detected during execution, replanning needed")
                #return self.run() 
            return self.Path

        except Exception as e:
            print(f"Error during path planning: {e}")
            return None

    
if __name__ == '__main__':
    client = carla.Client('localhost', 4000)
    client.set_timeout(10.0)
    # world = client.load_world('Town05') # Use this to switch towns
    # Get the world and map
    world = client.get_world()
    carla_map = world.get_map()

    #check 

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

    d_star.run()
    if d_star.state_space in d_star.V:
        d_star.V.remove(d_star.state_space)

    if d_star.state_space.transform.location == (d_star.xt.transform.location):
        print("Goal reached!")

        print(f"Current state: {d_star.state_space.transform.location}")
        print(f"Open queue size: {d_star.OPEN.qsize()}")
        print(f"Closed set size: {len(d_star.V)}")
    
    firetruck.destroy()