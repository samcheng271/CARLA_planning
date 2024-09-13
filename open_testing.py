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
        self.next_waypoint = None
        #self.start_location = start_location
        #self.goal_location = goal_location
        #self.x0 = self.map.get_waypoint(self.start_location.transform.location)
        self.x0 = start_waypoint
        print(f'x0: {self.x0}')
        #self.xt = self.map.get_waypoint(self.goal_location.transform.location)
        self.xt = end_waypoint
        print(f'xt: {self.xt}')

    def populate_open(self):
        if self.next_waypoint:
            child_waypoints = [self.next_waypoint]
            self.next_waypoint = None
        else:
            child_waypoints = self.children(self.state_space)

        print(f"Total child waypoints found: {len(child_waypoints)}")
    
        for child in child_waypoints:
            if child in self.V:
                print(f"Skipping already processed waypoint: {child.transform.location}")
                continue

            print(f"Processing child waypoint: {child.transform.location}")

            key = self.cost(self.state_space, child)
            print(f"Computed key (cost) for child: {key}")
        
            # Check if this key already exists in the queue
            existing_items = [item for item in self.OPEN.queue if item[0] == key]
            
            if not existing_items:
                self.OPEN.put((key, child))
                print(f"Added to OPEN: {child} with priority {key}")
                self.V.add(child)
            else:
                print(f"Key {key} already exists in the queue. Using next waypoint.")
                next_waypoints = self.children(child)
                if next_waypoints:
                    self.next_waypoint = next_waypoints[0]
                    print(f"Next waypoint set to: {self.next_waypoint.transform.location}")
                else:
                    print("No next waypoint available.")
    
        print(f"OPEN queue size: {self.OPEN.qsize()}")
    
        return self.OPEN
    
    # Checks if a 'y' state has both heuristic and tag dicts
    def checkState(self, y):
        if isinstance(y, carla.Waypoint):
            waypoint_id = y.id
            print(f'checkState called for Waypoint ID: {waypoint_id}')
        else:
            waypoint_id = y
            print(f'checkState called for state: {waypoint_id}')
    
    # Check and initialize the heuristic value
        if waypoint_id not in self.h:
            self.h[waypoint_id] = 0
            print(f'Initialized heuristic for state {waypoint_id} with value {self.h[waypoint_id]}')
    
    # Check and initialize the tag value
        if waypoint_id not in self.tag:
            self.tag[waypoint_id] = 'New'
            print(f'Initialized tag for state {waypoint_id} with value {self.tag[waypoint_id]}')
    
    def cost(self, wp1, wp2):
        distance = wp1.transform.location.distance(wp2.transform.location)
        print(f"Calculating distance between:")
        print(f"  Waypoint 1: Location({wp1.transform.location.x}, {wp1.transform.location.y}, {wp1.transform.location.z})")
        print(f"  Waypoint 2: Location({wp2.transform.location.x}, {wp2.transform.location.y}, {wp2.transform.location.z})")
        print(f"  Distance: {distance}")
        return distance

    def get_kmin(self):
        if self.OPEN:
            self.populate_open()
            #print(f"OPEN: {self.OPEN.empty()}")
            minimum = self.OPEN.get() 
            # Pop and return the tuple with the minimum key value
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

        if state.lane_change & carla.LaneChange.Right:
            right_wp = state.get_right_lane()
            if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                #right_state = (right_wp.transform.location.x, right_wp.transform.location.y, right_wp.transform.location.z)
                #if right_state in self.waypoints:
                    children.append(right_wp)
                    print(f"Right waypoint: {right_wp.transform.location if right_wp else 'None'}")

        return children
    
def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
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

    # Initialize D_star object
    d_star = D_star(waypoint=start_waypoint, 
                start_waypoint=start_waypoint, 
                end_waypoint=end_waypoint, 
                vehicle=firetruck, 
                world=world, 
                map=carla_map)

    # Iterate through all waypoints and run each function
    for waypoint in d_star.waypoints:
        print("\nRunning for Waypoint:", waypoint.transform.location)

        # Update the current state to the waypoint being processed
        d_star.state_space = waypoint
        d_star.checkState(waypoint)

        # Populate OPEN with potential child waypoints
        d_star.populate_open()
        
        # Retrieve the minimum key from the OPEN queue
        kmin = d_star.get_kmin()
        print(f"Minimum key from OPEN: {kmin}")

        
        # Get the minimum state and its corresponding key
        min_key, min_state = d_star.min_state()
        print(f"Minimum key and state: {min_key}, {min_state}")

        d_star.checkState(min_state)
        
        # Calculate the cost between the current state and its children
        children = d_star.children(d_star.state_space)
        for child in children:
            cost = d_star.cost(d_star.state_space, child)
            print(f"Cost from {d_star.state_space.transform.location} to {child.transform.location}: {cost}")

            print("------------------------------------------------------")

if __name__ == '__main__':
    main()
