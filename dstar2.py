import carla
import numpy as np
from collections import defaultdict

from carla import Transform, Location, Rotation

class D_star(object):
    def __init__(self, resolution=1):
        self.Alldirec = self.generate_directions()
        self.settings = 'CollisionChecking'

        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
        except Exception as e:
            print(f"Failed to connect to Carla: {e}")
            return

        self.resolution = resolution

        self.waypoints = self.map.generate_waypoints(self.resolution)
        self.state_space = self.convert_waypoints(self.waypoints)

        self.print_waypoints()
        self.print_state_space()
        self.visualize_waypoints()

        self.x0 = self.get_nearest_state(self.state_space, (0, 0, 0))
        self.xt = self.get_nearest_state(self.state_space, (50, 50, 0))
        self.b = defaultdict(lambda: defaultdict(dict))
        self.OPEN = {}
        self.h = {}
        self.tag = {}
        self.V = set()
        self.ind = 0
        self.Path = []
        self.done = False
        self.Obstaclemap = {}

        self.init_vehicle()

    #generates and returns a dict of direction vectors with their associated Euclidean dist
    def generate_directions(self):
        directions = {}
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    distance = np.sqrt(dx**2 + dy**2 + dz**2)
                    directions[(dx, dy, dz)] = distance
        return directions


    #converts carla waypoints objects into list of x,y,z coordinate tuples 
    def convert_waypoints(self, waypoints):
        return [(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z) for wp in waypoints]

    def print_waypoints(self):
        for wp in self.waypoints:
            print(f"Waypoint at location: x={wp.transform.location.x}, y={wp.transform.location.y}, z={wp.transform.location.z}")

    def print_state_space(self):
        for state in self.state_space:
            print(f"State: {state}")


    #finds the state in state_space closest to a point 
    def get_nearest_state(self, state_space, point):
    #min value of state_space, key arg is used to carry out Euclidean dist between state and point-the point we want to find the nearest state to
    #np.lingalg.norm- calculates the Euclidean norm for the difference vector
        nearest_state = min(state_space, key=lambda state: np.linalg.norm(np.array(state) - np.array(point)))
        return nearest_state

    #checks if a 'y' state has both heuristic and tag dicts 
    def checkState(self, y):
        if y not in self.h:
            self.h[y] = 0
        if y not in self.tag:
            self.tag[y] = 'New'

    #returns the min key value from the Open set
    def get_kmin(self):
        if self.OPEN:
            return min(self.OPEN.values())
        return -1

    #removes state with smalest key value from Open set
    def min_state(self):
        if self.OPEN:
            minvalue = min(self.OPEN.values())
            for k in self.OPEN.keys():
                if self.OPEN[k] == minvalue:
                    return k, self.OPEN.pop(k)
        return None, -1

    def insert(self, x, h_new):
        if self.tag[x] == 'New':
            kx = h_new
        elif self.tag[x] == 'Open':
            kx = min(self.OPEN[x], h_new)
        elif self.tag[x] == 'Closed':
            kx = min(self.h[x], h_new)
        self.OPEN[x] = kx
        self.h[x], self.tag[x] = h_new, 'Open'

    #sees state with min key value from Open set and UPDATES costs and parent pointers for neighboring states based on NEW info
    def process_state(self):
        x, kold = self.min_state()
        self.tag[x] = 'Closed'
        self.V.add(x)
        if x is None:
            return -1
        self.checkState(x)
        if kold < self.h[x]:
            for y in children(self, x):
                self.checkState(y)
                a = self.h[y] + cost(self, y, x)
                if self.h[y] <= kold and self.h[x] > a:
                    self.b[x], self.h[x] = y, a
        if kold == self.h[x]:
            for y in children(self, x):
                self.checkState(y)
                bb = self.h[x] + cost(self, x, y)
                if self.tag[y] == 'New' or \
                        (self.b[y] == x and self.h[y] != bb) or \
                        (self.b[y] != x and self.h[y] > bb):
                    self.b[y] = x
                    self.insert(y, bb)
        else:
            for y in children(self, x):
                self.checkState(y)
                bb = self.h[x] + cost(self, x, y)
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
        return self.get_kmin()

    def modify_cost(self, x):
        xparent = self.b[x]
        if self.tag[x] == 'Closed':
            self.insert(x, self.h[xparent] + self.cost(x, xparent))

    def modify(self, x):
        self.modify_cost(x)
        while True:
            kmin = self.process_state()
            if kmin >= self.h[x]:
                break

    #makes a path from state to goal state by following parent pointers from the goal
    def path(self, goal=None):
        path = []
        if not goal:
            x = self.x0
        else:
            x = goal
        start = self.xt
        while x != start:
            path.append([np.array(x), np.array(self.b[x])])
            x = self.b[x]
        return path

    #retuns list of neighboring states for a state 
    def children(self, state):
        children = []
        for direction, _ in self.Alldirec.items():
            child = tuple(np.array(state) + np.array(direction))
            if child in self.state_space:
                children.append(child)
        return children

    #calculates the cost of moving from one state to next state 
    def cost(self, start, goal):
        if goal in self.Obstaclemap:
            return np.inf
        return np.linalg.norm(np.array(start) - np.array(goal))

    def run(self):
        self.OPEN[self.xt] = 0
        self.tag[self.x0] = 'New'
        while self.tag[self.x0] != "Closed":
            self.process_state()
        self.Path = self.path()
        self.done = True
        self.visualize_path(self.Path)

        # Handle dynamic environment changes in CARLA
        for i in range(5):
            self.move_vehicle()
            s = self.x0
            while s != self.xt:
                sparent = self.b[s]
            # checks if cost is infinite to move from sparent to s and modifies to update cost 
                if self.cost(s, sparent) == np.inf:
                    self.modify(s)
                    continue
                self.ind += 1
                s = sparent
            self.Path = self.path()
            self.visualize_path(self.Path)
        plt.show()

    #spawns vehicle in carla
    def init_vehicle(self):
        try:
            spawn_points = self.world.get_map().get_spawn_points()
            vehicle_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_points[0])
            if self.vehicle:
                print("Vehicle spawned for simulation.")
            else:
                print("Failed to spawn vehicle.")
        except Exception as e:
            print(f"Error initializing vehicle: {e}")

    #controls vehicle to be moved from one place to another
    def move_vehicle(self):
        if not self.vehicle:
            print("Vehicle not initialized.")
            return

        if self.Path:
            next_waypoint = self.Path.pop(0)
            location = carla.Location(x=next_waypoint[0][0], y=next_waypoint[0][1], z=next_waypoint[0][2])
            self.vehicle.set_location(location)

            time.sleep(0.1)
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
    D = D_star(1)
    D.run()
