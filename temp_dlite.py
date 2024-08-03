import carla
import random
import time
# from queue import PriorityQueue
from PriorityQueue import PriorityQueue, Priority
def succ(waypoint):
    neighbors = []
    # Forward neighbor
    forward = waypoint.next(2.0)
    if forward:
        neighbors.extend(forward)
    
    # Legal left lane change
    if waypoint.lane_change & carla.LaneChange.Left:
        left_lane = waypoint.get_left_lane()
        if left_lane and left_lane.lane_type == carla.LaneType.Driving:
            neighbors.append(left_lane)
    
    # Legal right lane change
    if waypoint.lane_change & carla.LaneChange.Right:
        right_lane = waypoint.get_right_lane()
        if right_lane and right_lane.lane_type == carla.LaneType.Driving:
            neighbors.append(right_lane)
    
    return neighbors

class DStarLite:
    def __init__(self, world, start_waypoint, end_waypoint):
        self.world = world
        self.start = start_waypoint
        self.goal = end_waypoint
        self.U = PriorityQueue()
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.epsilon = 1
        print('init')
        

    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    def heuristic_c(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    def contain(self, u):
        return any(item[1] == u for item in self.U.queue)
    
    def wp_key(self, waypoint):
        return (waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z)

    def calculate_key(self, s):
        return [
            min(self.g[s.id], self.rhs[s.id]) + self.epsilon * self.heuristic(s, self.start) + self.km,
            min(self.g[s.id], self.rhs[s.id])
        ]

    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        for s in self.world.get_map().generate_waypoints(2.0):
        # for s in self.world.get_map().generate_waypoints(2.0):
            self.rhs[s.id] = float('inf')
            self.g[s.id] = float('inf')
        self.rhs[self.goal.id] = 0
        # self.U.put((self.calculate_key(self.goal), self.goal))
        self.U.insert(self.goal, [self.heuristic(self.start, self.goal), 0])
        # print(f'self.goal {self.goal}') # wp
        # print(f'self.U {self.U.top_key()}') # (g, rhs)
        # print(f'self.U {self.U.pop()}') # wp
        # print(f'self.U {self.U.top_key()}') # empty
        # print(f'self.g {self.g}') # {..., wp.id: g, ...}

    def update_vertex(self, u):
        if self.g[u.id] != self.rhs[u.id] and self.contain(u):
            self.U.update(u, self.calculate_key(u))
            print('1')
        elif self.g[u.id] != self.rhs[u.id] and not self.contain(u):
            self.U.insert(u, self.calculate_key(u))
            print('2')
        elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
            self.U.remove(u)
            print('3')
    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.s_start) or self.rhs[self.s_start] > self.g[self.s_start]:
            u = self.U.top() # waypoint
            k_old = self.U.top_key() # (g, rhs) which is (heuristic, min(g, rhs))
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif self.g[u.id] > self.rhs[u.id]:
                self.g[u.id] = self.rhs[u.id]
                self.U.remove(u)
                pred = succ(u)
                for s in pred:
                    if s != self.s_goal:
                        self.rhs[s.id] = min(self.rhs[s.id], self.heuristic_c(s, u) + self.g[u.id])
                    self.update_vertex(s)
            else:
                self.g_old = self.g[u.id]
                self.g[u.id] = float('inf')
                pred = succ(u)
                pred.append(u)
                for s in pred:
                    if self.rhs[s.id] == (self.heuristic_c(s, u) + self.g_old):
                        if s != self.s_goal:#?????
                            min_s = float('inf')
                            # succ = self.sensed_map.succ(vertex=s)
                            succ = s
                            for s_ in succ:
                                # temp = self.heuristic_c(s, s_) + self.g[s_.id]
                                # if min_s > temp:
                                #     min_s = temp
                                temp = min( self.heuristic_c(s, s_) + self.g[s_.id], min_s)
                            self.rhs[s.id] = min_s
                    self.update_vertex(u)

    # def rescan(self) -> Vertices:

    #     new_edges_and_old_costs = self.new_edges_and_old_costs
    #     self.new_edges_and_old_costs = None
    #     return new_edges_and_old_costs

    def move_and_replan(self, robot_position):
        path = [robot_position]
        self.s_start = robot_position
        self.s_last = self.s_start
        self.compute_shortest_path()

        while self.s_start != self.s_goal:
            assert (self.rhs[self.s_start] != float('inf')), "There is no known path!"

            # succ = self.sensed_map.succ(self.s_start, avoid_obstacles=False)
            succ = succ(self.s_start)
            min_s = float('inf')
            arg_min = None
            for s_ in succ:
                temp = self.heuristic_c(self.s_start, s_) + self.g[s_]
                if temp < min_s:
                    min_s = temp
                    arg_min = s_

            ### algorithm sometimes gets stuck here for some reason !!! FIX
            self.s_start = arg_min
            path.append(self.s_start)
            # scan graph for changed costs
            changed_edges_with_old_cost = self.rescan()
            #print("len path: {}".format(len(path)))
            # if any edge costs changed
            if changed_edges_with_old_cost:
                self.k_m += heuristic(self.s_last, self.s_start)
                self.s_last = self.s_start

                # for all directed edges (u,v) with changed edge costs
                vertices = changed_edges_with_old_cost.vertices
                for vertex in vertices:
                    v = vertex.pos
                    succ_v = vertex.edges_and_c_old
                    for u, c_old in succ_v.items():
                        c_new = heuristic_c(u, v)
                        if c_old > c_new:
                            if u != self.s_goal:
                                self.rhs[u] = min(self.rhs[u], heuristic_c(u, v) + self.g[v])
                        elif self.rhs[u] == c_old + self.g[v]:
                            if u != self.s_goal:
                                min_s = float('inf')
                                succ_u = self.sensed_map.succ(vertex=u)
                                for s_ in succ_u:
                                    temp = heuristic_c(u, s_) + self.g[s_]
                                    if min_s > temp:
                                        min_s = temp
                                self.rhs[u] = min_s
                            self.update_vertex(u)
            self.compute_shortest_path()
        print("path found!")
        return path, self.g, self.rhs

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

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
while point_b.location == point_a.location:
    point_b = random.choice(spawn_points)

start_waypoint = carla_map.get_waypoint(point_a.location)
end_waypoint = carla_map.get_waypoint(point_b.location)

print("Firetruck starting at", point_a.location)
print(f"Destination: {point_b.location}")
dstar_lite = DStarLite(world, start_waypoint, end_waypoint)
dstar_lite.initialize()


# Clean up
firetruck.destroy()
print('end')