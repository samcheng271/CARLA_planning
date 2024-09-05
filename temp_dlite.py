import carla
import random
import time
# from queue import PriorityQueue
from PriorityQueueDLite import PriorityQueue, Priority
import sys
# print(sys.getrecursionlimit())
sys.setrecursionlimit(50000)
def successors(waypoint):
    neighbors = []
    # Forward neighbor
    forward = waypoint.next(.5)
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
def predescessors(waypoint):
    neighbors = []
    # Forward neighbor
    forward = waypoint.previous(1)
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
    def __init__(self, world, start_waypoint, end_waypoint,all_waypoints):
        self.world = world
        self.map = world.get_map()
        self.start = start_waypoint
        self.goal = end_waypoint
        self.U = PriorityQueue()
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.epsilon = 1
        self.s_last = None
        self.s_current = None
        self.all_waypoints = all_waypoints
        print('init successfully')
        

    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    def heuristic_c(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    def contain(self, u):
        return any(item[1] == u for item in self.U.queue)
    
    def wp_key(self, waypoint):
        return (waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z)

    def calculate_key(self, s):
        # # print(f'g {self.g.popitem()}')
        # print(f's {s}')
        # print(f'START {self.start}')
        # print(f'GOAL {self.goal}')
        # # print(f'g s.id {self.g[self.g.keys()[0]]}')
        # # print(f'self.g',min(self.g.values()))
        # print(f'self.g START',self.g[self.goal.id])
        # print(f'self.g',self.g[s.id])
        # print(f'self.rhs',self.rhs[s.id])
        # x1 = min(self.g[s.id], self.rhs[s.id])
        # x2 = self.epsilon * self.heuristic(s, self.start) + self.km
        # y=min(self.g[s.id], self.rhs[s.id])
        return [
            min(self.g[s.id], self.rhs[s.id]) + self.epsilon * self.heuristic(s, self.start) + self.km,
            min(self.g[s.id], self.rhs[s.id])
        ]


    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        # print(f'self.world.get_map().generate_waypoints(2.0) {self.world.get_map().generate_waypoints(2.0)}')
        # for s in self.world.get_map().generate_waypoints(2.0):
        for s in self.all_waypoints:
            # print(f's {s}')
        # for s in self.world.get_map().generate_waypoints(2.0):
            current_wp = self.map.get_waypoint(s.transform.location)
            self.rhs[current_wp.id] = float('inf')
            self.g[current_wp.id] = float('inf')
        self.g[self.goal.id]=0
        self.g[self.start.id]=float('inf')
        self.rhs[self.goal.id] = 0
        print(f'self.goal.id {self.goal.id}')
        self.rhs[self.start.id] = float('inf')

        # self.U.put((self.calculate_key(self.goal), self.goal))
        self.U.insert(self.goal, [self.heuristic(self.start, self.goal), 0])
        # print(f'self.goal {self.goal}') # wp
        print(f'self.U {self.U.top_key()}') # (g, rhs)
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
    def prev_check(self, u):
        pred = predescessors(u)
        for s in pred:
            self.world.debug.draw_string(s.transform.location, 'U', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=200.0, persistent_lines=True)
            self.prev_check(s)
            if s.waypoint.transform.location.distance(self.goal.transform.location) < 10.0:
                print('REACHED\n')
                break

        return None

    def compute_shortest_path(self):
        while self.U.top_key() < self.calculate_key(self.start) or self.rhs[self.start] > self.g[self.start]:
            u = self.U.top() # waypoint
            k_old = self.U.top_key() # (g, rhs) which is (heuristic, min(g, rhs))
            k_new = self.calculate_key(u)

            if k_old < k_new: # if the waypoint is not up to date in the open set
                print('compute_shortest_path 1')
                
                self.U.update(u, k_new)
            elif self.g[u.id] > self.rhs[u.id]: # if a more optimal path is found
                print('compute_shortest_path 2')
                self.g[u.id] = self.rhs[u.id]
                self.U.remove(u)
                pred = predescessors(u)
                for s in pred:
                    if s != self.s_goal:
                        self.rhs[s.id] = min(self.rhs[s.id], self.heuristic_c(s, u) + self.g[u.id])
                    self.update_vertex(s)
            else:
                print('compute_shortest_path 3')

                self.g_old = self.g[u.id]
                self.g[u.id] = float('inf')
                pred = predescessors(u)
                # pred.append(u)

                # for i in range(5, 50):
                #     self.world.debug.draw_string(self.all_waypoints[i].transform.location, f'{i}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

                z = self.all_waypoints[5]

                # a = self.all_waypoints[6]
                # b = self.all_waypoints[7]
                # c = self.all_waypoints[8]
                # d = self.all_waypoints[9]

                # e = self.all_waypoints[10]
                # f = self.all_waypoints[11]
                # g = self.all_waypoints[12]
                # h = self.all_waypoints[13]
                self.world.debug.draw_string(z.transform.location, 'Z', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(a.transform.location, 'A', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(b.transform.location, 'B', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(c.transform.location, 'C', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(d.transform.location, 'D', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(e.transform.location, 'E', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(f.transform.location, 'F', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(g.transform.location, 'G', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(h.transform.location, 'H', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.world.debug.draw_string(u.transform.location, 'U', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
                # self.prev_check(u)
                for i,j in self.map.get_topology():
                    print(f'i {i} j {j}')
                    self.world.debug.draw_string(i.transform.location, 'W', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

                # for s in pred:
                #     # self.prev_check(u)
                #     if s in self.all_waypoints:
                #         print('GOT IT\n')
                #     self.world.debug.draw_string(s.transform.location, 'W', draw_shadow=False, color=carla.Color(r=110, g=0, b=000), life_time=60.0, persistent_lines=True)
                # for i in self.all_waypoints:
                #     self.world.debug.draw_string(i.transform.location, 'I', draw_shadow=False, color=carla.Color(r=0, g=0, b=220), life_time=60.0, persistent_lines=True)
                    
                print(f'pred:: {pred}')
                for s in pred:
                    # y=self.heuristic_c(s, u)
                    # g=self.g_old
                    # print(f'y {y}')
                    # print(f'g {g}')
                    print(f'pred s:: {s}')
                    print(f'pred start:: {self.start}')
                    print(f'pred goal id:: {self.goal.id}')
                    x = self.map.get_waypoint(self.goal.transform.location, project_to_road=True)
                    print(f'x {x.id}')
                    print(f'pred goal:: {self.goal}')
                    print(f'x {x}')
                    print(f's.id {s.id}')
                    print(f'self.rhs## {self.rhs[s.id]}')
                    self.world.debug.draw_string(pred[0].transform.location, 'X', draw_shadow=False, color=carla.Color(r=220, g=0, b=220), life_time=60.0, persistent_lines=True)
                    if self.rhs[s.id] == (self.heuristic_c(s, u) + self.g_old):
                        if s != self.s_goal:#?????
                            min_s = float('inf')
                            # succ = self.sensed_map.successors(vertex=s)
                            succ = s
                            for s_ in succ:
                                # temp = self.heuristic_c(s, s_) + self.g[s_.id]
                                # if min_s > temp:
                                #     min_s = temp
                                temp = min( self.heuristic_c(s, s_) + self.g[s_.id], min_s)
                            self.rhs[s.id] = min_s
                    self.update_vertex(u)

    def main(self):
        self.s_last = self.start
        self.s_current = self.start
        # self.initialize()
        self.compute_shortest_path()

        while self.s_current != self.goal:
            if self.g[self.s_current.id] == float('inf'):
                print("There is no known path to the goal.")
                return

            # Move to the best successor
            successor = successors(self.s_current)
            if not successor:
                print("No valid successor found.")
                return

            self.s_current = min(
                successor,
                key=lambda s: self.heuristic_c(self.s_current, s) + self.g[s.id]
            )

            print(f"Moving to: {self.s_current.transform.location}")

            # Simulate movement (in a real scenario, you'd move the vehicle here)
            time.sleep(0.5)  # Simulating movement time

            # Scan for changes in edge costs
            changed_edges = self.scan_for_changes()
            
            if changed_edges:
                self.km += self.heuristic(self.s_last, self.s_current)
                self.s_last = self.s_current

                for u, v in changed_edges:
                    c_old = self.heuristic_c(u, v)
                    # Update the edge cost (in a real scenario, you'd get the new cost from the environment)
                    c_new = random.uniform(0.8 * c_old, 1.2 * c_old)  # Simulating cost change
                    
                    if c_old > c_new:
                        if u != self.goal:
                            self.rhs[u.id] = min(self.rhs[u.id], c_new + self.g[v.id])
                    elif self.rhs[u.id] == c_old + self.g[v.id]:
                        if u != self.goal:
                            self.rhs[u.id] = min(
                                self.heuristic_c(u, s) + self.g[s.id]
                                for s in successors(u)
                            )
                    self.update_vertex(u)

                self.compute_shortest_path()

        print("Goal reached!")

    def scan_for_changes(self):
        # In a real scenario, this method would detect actual changes in the environment
        # For this simulation, we'll randomly decide if there are changes
        if random.random() < 0.2:  # 20% chance of detecting changes
            current_successors = successors(self.s_current)
            return [(self.s_current, s) for s in random.sample(current_successors, min(2, len(current_successors)))]
        return []


    
    # # def rescan(self) -> Vertices:

    # #     new_edges_and_old_costs = self.new_edges_and_old_costs
    # #     self.new_edges_and_old_costs = None
    # #     return new_edges_and_old_costs

    # def main(self, robot_position):
    #     path = [robot_position]
    #     self.start = robot_position
    #     self.s_last = self.start
    #     self.compute_shortest_path()

    #     while self.start != self.s_goal:
    #         assert (self.rhs[self.start] != float('inf')), "There is no known path!"

    #         # succ = self.sensed_map.successors(self.start, avoid_obstacles=False)
    #         succ = successors(self.start)
    #         min_s = float('inf')
    #         arg_min = None
    #         for s_ in succ:
    #             temp = self.heuristic_c(self.start, s_) + self.g[s_]
    #             if temp < min_s:
    #                 min_s = temp
    #                 arg_min = s_

    #         ### algorithm sometimes gets stuck here for some reason !!! FIX
    #         self.start = arg_min
    #         path.append(self.start)
    #         # scan graph for changed costs
    #         changed_edges_with_old_cost = self.rescan()
    #         #print("len path: {}".format(len(path)))
    #         # if any edge costs changed
    #         if changed_edges_with_old_cost:
    #             self.k_m += heuristic(self.s_last, self.start)
    #             self.s_last = self.start

    #             # for all directed edges (u,v) with changed edge costs
    #             vertices = changed_edges_with_old_cost.vertices
    #             for vertex in vertices:
    #                 v = vertex.pos
    #                 succ_v = vertex.edges_and_c_old
    #                 for u, c_old in succ_v.items():
    #                     c_new = heuristic_c(u, v)
    #                     if c_old > c_new:
    #                         if u != self.s_goal:
    #                             self.rhs[u] = min(self.rhs[u], heuristic_c(u, v) + self.g[v])
    #                     elif self.rhs[u] == c_old + self.g[v]:
    #                         if u != self.s_goal:
    #                             min_s = float('inf')
    #                             succ_u = self.sensed_map.successors(vertex=u)
    #                             for s_ in succ_u:
    #                                 temp = heuristic_c(u, s_) + self.g[s_]
    #                                 if min_s > temp:
    #                                     min_s = temp
    #                             self.rhs[u] = min_s
    #                         self.update_vertex(u)
    #         self.compute_shortest_path()
    #     print("path found!")
    #     return path, self.g, self.rhs

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Get the world and map
world = client.get_world()
carla_map = world.get_map()
# print(f' top {carla_map.get_topology()}')
# Spawn a firetruck at a random location (point A)
blueprint_library = world.get_blueprint_library()
firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]

spawn_points = carla_map.get_spawn_points()
# print(f'spawn_points {spawn_points[0]}')
# gen_points = carla_map.generate_waypoints(2.0)
# print(f'gen_points {gen_points[0]}')
# all_waypoints = gen_points + spawn_points[0]
# print(f'all_waypoints {all_waypoints[0]}')
# print(f'gen_points {spawn_points[0].transform}\n')
# for i in range(len(spawn_points)-1):
#     spawn_points[i]=spawn_points[i]
# print(f'spawn_points[0]: {spawn_points[0]}\n')
# print(f'spawn_points[0]: {Waypoint(spawn_points[0])}\n')
# for i in range(len(gen_points)-1):
#     gen_points[i]=gen_points[i].transform

# print(f'gen_points[0]: {gen_points[0]}')
# world.debug.draw_string(spawn_points[0].location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)

# Choose a random starting location (point A)
point_a = random.choice(spawn_points)
point_a = spawn_points[0]
print("point_a:", point_a)
firetruck = world.spawn_actor(firetruck_bp, point_a)

# # Get predefined spawn points
# predefined_spawn_points = carla_map.get_spawn_points()

# # Generate additional waypoints
# generated_waypoints = carla_map.generate_waypoints(2.0)

# # Combine predefined spawn points and generated waypoints
# spawn_points = predefined_spawn_points + [waypoint.transform for waypoint in generated_waypoints]
# point_a = random.choice(spawn_points)
# firetruck = world.spawn_actor(firetruck_bp, point_a)
# Choose a random destination (point B)
point_b = random.choice(spawn_points)
while point_b.location == point_a.location:
    point_b = random.choice(spawn_points)

start_waypoint = carla_map.get_waypoint(point_a.location)
end_waypoint = carla_map.get_waypoint(point_b.location)

print("Firetruck starting at", point_a.location)
print(f"Destination: {point_b.location}")

gen_points = carla_map.generate_waypoints(0.5)
real_points = []
for i in gen_points:
    real_points.append(carla_map.get_waypoint(i.transform.location))


print(f'gen_points {gen_points[0]}')
print(f'real_points {real_points[0]}')
all_waypoints = real_points + [start_waypoint] +[end_waypoint]
print(f'all_waypoints {all_waypoints[0]}')
try:

    dstar_lite = DStarLite(world, start_waypoint, end_waypoint,all_waypoints)
    dstar_lite.initialize()
    dstar_lite.main()

finally:
     # Clean up
    firetruck.destroy()
    print('Firetruck destroyed successfully')
print('done')