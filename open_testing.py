import carla
import numpy as np
import random
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
        #print(f"print: {self.state_space}")
        self.waypoints = self.map.generate_waypoints(self.resolution)
        print(f"Number of waypoints generated: {len(self.waypoints)}")
        # print(f"waypoint list: {self.waypoints}")
        self.h = {}
        self.next_waypoint = None
        #self.start_location = start_location
        #self.goal_location = goal_location
        #self.x0 = self.map.get_waypoint(self.start_location.transform.location)
        self.x0 = start_waypoint
        print(f'start_waypoint: {self.x0.transform.location}')
        #self.xt = self.map.get_waypoint(self.goal_location.transform.location)
        self.xt = end_waypoint
        print(f'end_waypoint: {self.xt.transform.location}')
    
    def populate_open(self):
        if self.next_waypoint:
            child_waypoints = [self.next_waypoint]
            self.next_waypoint = None
        else:
            child_waypoints = self.children(self.state_space)

        print(f"Total child waypoints found: {len(child_waypoints)}")
        #call funciton here that takes into consideration obstalce avoidance
    
        for child in child_waypoints:
            if child in self.V:
                print(f"Alr processed waypoint: {child.transform.location}")
                continue

            print(f"Processing child waypoint: {child.transform.location}")

            key = self.cost(self.state_space, child)
            heuristic_cost = self.store_h(child)
            #total_cost = key + heuristic_cost

            min_distance = heuristic_cost - key
            print(f"min_distance: {min_distance}")

            #when items that are correclty getting accessed we don't need to check for existing items
            #temp fix 
            existing_items = [item for item in self.OPEN.queue if abs(item[0] - key) < 1e-6]

            if not existing_items:
                self.OPEN.put((min_distance, child))
                print(f"OPEN: {child} key: {key}")
                self.V.add(child)
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

    def delete(self, state):
        self.OPEN.remove(state)
        self.tag[state.id] = "Closed"

    def detect_obstacles(self, state_space):
        """

            create recursion here where when an obstacle is detected, it recalculates children from the new current vehicle space  
            after a move has already been taken 

        """
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
        Detect obstacles and handle states.
        If obstacles are detected, recalculates children and updates the state space.
        """
        if self.detect_obstacles():
            print("Obstacle detected.")
        
            child_waypoints = self.children(self.state_space)
            for child in child_waypoints:
                self.delete(child)

            if not self.OPEN.empty():
                self.populate_open()
                kold, x = self.min_state()
                self.state_space = x
                next_waypoints = self.children(self.state_space)
                print(f"Next waypoints recalculated: {[wp.transform.location for wp in next_waypoints]}")
            
            else:
                print("No obstacles detected")
    
    def cost(self, state, wp2):
        distance = state.transform.location.distance(wp2.transform.location)
        print(f"Calculating distance between:")
        print(f"  Waypoint 1: Location({state.transform.location.x}, {state.transform.location.y}, {state.transform.location.z})")
        print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        print(f"  Distance: {distance}")
        return distance
    
    def store_h(self, state):
        if state is None:
            return float('inf')
    
        if state.id not in self.h:
            heuristic = state.transform.location.distance(self.xt.transform.location)
            print(f"State: Location({self.state.transform.location.x}, {self.state.transform.location.y}, {self.state.transform.location.z})")
            print(f"Self.xt: Location({self.xt.transform.location.x}, {self.xt.transform.location.y}, {self.xt.transform.location.z})")
            print(f"store_h, heuristic: {heuristic}")
            self.h[state.id] = heuristic
    
        return self.h[state.id]
    
    """
    # Checks if a 'y' state has both heuristic and tag dicts
    def checkState(self, y):
        if y.id not in self.h:
            heuristic = self.store_h(y)
            self.h[y.id] = heuristic 
            #self.h[waypoint_id] = 0
            print(f'Heuristic for state {y}: {self.h[y.id]}')
        else:
            print(f'Heuristic for state {y}: {self.h[y.id]}')

        if y.id not in self.tag:
            self.tag[y.id] = 'New'
            print(f'Tag for state {y}: {self.tag[y.id]}')
    """
    def get_kmin(self):
        if self.OPEN:
            self.populate_open()
            #print(f"OPEN: {self.OPEN.empty()}")
            minimum = self.OPEN.get() # Return the tuple with the minimum key value
            print(f'get_kmin: minimum: {minimum[0]}')
            return minimum[0]
        
    def min_state(self):
        if self.OPEN:
            print("min_state")
            self.populate_open()
            minimum = self.OPEN.get()
            print(f'get_kmin, state: key: {minimum[0]}, state: {minimum[1]}')
            return minimum[0], minimum[1] #returns state k with associated key value
        return None, -1
    """
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

        self.V.add(state)
        print(f"V: {self.V}")

        self.OPEN.put((kx, state)) 
        self.tag[state.id] = 'Open'
        print(f'Inserted state {state} with key {kx}')
    """

    
    def children(self, state):
        children = []
        #get vehicle's current location
        #location = self.vehicle.get_location()
        #current_waypoint = self.map.get_waypoint(location, project_to_road=True)

        if state is None:
            return children

        next_waypoint = state.next(2.0)
        print(f"Next waypoints: {[wp.transform.location for wp in next_waypoint]}")
        if next_waypoint:
            """
            next_wp = next_waypoint[0]
            if next_wp.transform.location not in [c.transform.location for c in children]:
                children.append(next_wp)
            """
            children.extend(next_waypoint)

        if state.lane_change & carla.LaneChange.Left:
            left_wp = state.get_left_lane()
            if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                #left_state = (left_wp.transform.location.x, left_wp.transform.location.y, left_wp.transform.location.z)
                #if left_wp in self.waypoints:
                    children.append(left_wp)
                    print(f"Left waypoint: {left_wp.transform.location if left_wp else 'None'}")
                    diag_left_wp = left_wp.next(2.0)
                    if diag_left_wp:
                        children.extend(diag_left_wp)

        if state.lane_change & carla.LaneChange.Right:
            right_wp = state.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                #right_state = (right_wp.transform.location.x, right_wp.transform.location.y, right_wp.transform.location.z)
                #if right_state in self.waypoints:
                    children.append(right_wp)
                    print(f"Right waypoint: {right_wp.transform.location if right_wp else 'None'}")
                    diag_right_wp = right_wp.next(2.0)
                    if diag_right_wp:
                        children.extend(diag_right_wp)

        return children
    


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

    d_star.state_space = start_waypoint

    for waypoint in d_star.waypoints:
        d_star.state_space = waypoint

        min_key, min_state = d_star.min_state()

        if min_state.transform.location.distance(end_waypoint.transform.location) < d_star.resolution:
            print("Goal reached!")
            break

        children = d_star.children(min_state)
        for child in children:
            cost = d_star.cost(min_state, child)
            print(f"Cost from {min_state.transform.location} to {child.transform.location}: {cost}")

            #d_star.insert(cost, child)

        if min_state in d_star.V:
            d_star.V.remove(min_state)

    if min_state.transform.location.distance(end_waypoint.transform.location) < d_star.resolution:
        for waypoint in d_star.waypoints:
            firetruck.set_transform(waypoint.transform)
            print(f"Moving to waypoint: {waypoint.transform.location}")

    firetruck.destroy()