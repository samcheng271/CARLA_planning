import carla
import random
import time
# from queue import PriorityQueue
from PriorityQueueDLite import PriorityQueue, Priority
import sys
# print(sys.getrecursionlimit())
sys.setrecursionlimit(50000)


class DStarLite:
    def __init__(self, world, start_waypoint, end_waypoint,all_waypoints,wp_pts):
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
        self.wp_pos = wp_pts
        print('init successfully')
        self.new_edges_and_old_costs = None
        
        # why are end_waypoint and start_waypoint in reverse but self.goal and self.start aren't?
        # self.world.debug.draw_string(end_waypoint.transform.location, 'EEEEEEEEEE', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
        # self.world.debug.draw_string(start_waypoint.transform.location, 'SSSSSSSSSS', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)

    def successors(self,waypoint):
        neighbors = []
        # Forward neighbor
        forward = waypoint.next(1)
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
                
        for i in range(len(neighbors)):
            initial_dist = neighbors[i].transform.location.distance(waypoint.transform.location)
            if self.g.get(neighbors[i].id) is None:
                for z in self.all_waypoints:
                    if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:
                        x = z
                # print(f'x {x}')
                neighbors[i] = x
        return neighbors
    def predecessors(self, waypoint):
        waypoint=self.all_waypoints[0]
        neighbors = []
        # Pred(the waypoint) is visible
        # self.world.debug.draw_string(waypoint.transform.location, 'pred', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)

        # Backward neighbor
        Backward = waypoint.previous(1)
        # print(f'Backward {Backward[0]}')
        if Backward:
            
            neighbors.extend(Backward)
        
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

        # print(f'neighbors {neighbors}')
        # print(f'bef neighbors:: {neighbors[0]}')
        # print(f'bef neighbors:: {neighbors[1]}')
        self.world.debug.draw_string(neighbors[0].transform.location, 'weutowr', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)

        self.world.debug.draw_string(neighbors[1].transform.location, 'yweiytwiuy', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)

        # x = self.wp_pos[waypoint.id]
        # print(f'x {x}')
        for i in range(len(neighbors)):
            initial_dist = neighbors[i].transform.location.distance(waypoint.transform.location)
            if self.g.get(neighbors[i].id) is None:
                for z in self.all_waypoints:
                    if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:
                        x = z
                # print(f'x {x}')
                neighbors[i] = x
                # print(f'i {neighbors[i]}')

        # print(f'end neighbors:: {neighbors[0]}')
        # print(f'end neighbors:: {neighbors[1]}')
        # print(f'[0] should be neighbors:: {self.map.get_waypoint(neighbors[0].transform.location, project_to_road=True)}')
        # print(f'[1] should be neighbors:: {self.map.get_waypoint(neighbors[1].transform.location, project_to_road=True)}')
        # print(f'end neighbors:: {neighbors}')
        
        return neighbors
    
    def heuristic(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    def heuristic_c(self, waypoint1, waypoint2):
        return waypoint1.transform.location.distance(waypoint2.transform.location)
    def contain(self, u):
        # print(f'u {self.U.heap}')
        # print(f'u {self.U.vertices_in_heap[0]}')
        return any(item == u for item in self.U.vertices_in_heap)
    
    def wp_key(self, waypoint):
        return (waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z)

    def calculate_key(self, s):
        return Priority(min(self.g[s.id], self.rhs[s.id]) + self.epsilon * self.heuristic(s, self.start) + self.km,
                        min(self.g[s.id], self.rhs[s.id]))


    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        # check = 0
        for s in self.all_waypoints:
            self.rhs[s.id] = float('inf')
            self.g[s.id] = float('inf')
        self.g[self.goal.id]=0
        self.g[self.start.id]=float('inf')
        self.rhs[self.goal.id] = 0
        self.rhs[self.start.id] = float('inf')
        # print(f'self.rhs {self.rhs}')
        # self.U.put((self.calculate_key(self.goal), self.goal))
        # self.U.insert(self.goal, [self.heuristic(self.start, self.goal), 0])
        self.U.insert(self.goal,  Priority(self.heuristic(self.start, self.goal), 0))
        
        print(f'self.goal {self.goal}') # wp
        print(f'self.U {self.U.top_key()}') # Priority(g, rhs)
        print(f'self.U {self.U.heap}') # [priorityNode]
        print(f'self.U {self.U.vertices_in_heap}') # [wp]
        print(f'goal calculate_key {self.calculate_key(self.goal).k1}') 
        pred_list = self.predecessors(self.goal)
        for i in range(len(pred_list)):
            self.world.debug.draw_string(pred_list[i].transform.location, f'init:{i}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
        print(f'pred_list {pred_list}')
        print(f'pred 1 {pred_list[0]}')
        # print(f'self.U {self.U.pop()}') # wp
        # print(f'self.U {self.U.top_key()}') # empty
        # print(f'self.g {self.g}') # {..., wp.id: g, ...}



    def update_vertex(self, u):
        # if self.g[u.id] != self.rhs[u.id] and not self.contain(u):
        #     self.U.insert(u, self.calculate_key(u))
        #     print('2')
        #     print(f'self.U {self.U.top_key()}') # (g, rhs)
        # elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
        #     self.U.remove(u)
        #     print('3')
        #     print(f'self.U {self.U.top_key()}') # (g, rhs)

        # elif self.g[u.id] != self.rhs[u.id] and self.contain(u):
        #     self.U.update(u, self.calculate_key(u))
        #     print('1')
        #     print(f'self.U {self.U.top_key()}') # (g, rhs)

        if self.g[u.id] != self.rhs[u.id] and self.contain(u):
            # a shorter path has been found
            self.U.update(u, self.calculate_key(u))
            print('1')
            print(f'self.U {self.U.top_key()}') # (g, rhs)

        elif self.g[u.id] != self.rhs[u.id] and not self.contain(u):
            # if node hasn't been processed to its optimal cost yet
            self.U.insert(u, self.calculate_key(u))
            print('2')
            print(f'self.U {self.U.top_key()}') # (g, rhs)

        elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
            # have already optimized this node but still in the open set(lookup)
            self.U.remove(u)
            print('3')
            print(f'self.U {self.U.top_key()}') # (g, rhs)

    def compute_shortest_path(self):
        while (self.U.top_key() < self.calculate_key(self.start)) or (self.rhs[self.start.id] > self.g[self.start.id]):
            u = self.U.top() # waypoint
            # self.world.debug.draw_string(u.transform.location, 'U', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=200.0, persistent_lines=True)
            k_old = self.U.top_key() # (g, rhs) which is (heuristic, min(g, rhs))
            k_new = self.calculate_key(u)

            if k_old < k_new: # if the waypoint is not up to date in the open set
                print('compute_shortest_path 1')
                self.U.update(u, k_new)
            elif self.g[u.id] > self.rhs[u.id]: # if a more optimal path is found
                print('compute_shortest_path 2')
                self.g[u.id] = self.rhs[u.id]
                self.U.remove(u)
                for s in self.predecessors(u):
                    if s != self.goal:
                        self.rhs[s.id] = min(self.rhs[s.id], self.heuristic_c(s, u) + self.g[u.id])
                    self.update_vertex(s)
            else:
                print('compute_shortest_path 3')

                self.g_old = self.g[u.id]
                # print('f')
                self.g[u.id] = float('inf')
                # print('g')
                pred = self.predecessors(u)
                # print('h')
                pred.append(u)
                # for i in range(len(self.all_waypoints)-3):
                #     self.world.debug.draw_string(self.all_waypoints[i].transform.location, f'{i}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

                    
                # z = self.all_waypoints[5]
                # self.world.debug.draw_string(z.transform.location, 'Z', draw_shadow=False, color=carla.Color(r=0, g=0, b=220), life_time=30.0, persistent_lines=True)
                # if current waypoint +/- 5 is not nearby then:
                # what if i use self.predecessors, then for each waypoint in pred,
                # i check what waypoints are close by to them in self.all_waypoints
                # also check if they are a legal lane change to avoid getting waypoints on opposite lane
                # use dictionary to store and lookup

                # pz=self.predecessors(z)
                # for i in pz:
                #     self.world.debug.draw_string(i.transform.location, 'P', draw_shadow=False, color=carla.Color(r=0, g=0, b=220), life_time=30.0, persistent_lines=True)

                #     print(f'pz {i}')
                #     # print(f'pz {self.g.get(i.id)}')

                # print(f'p1: {self.all_waypoints[1]}')
                # print(f'p4: {self.all_waypoints[4]}')

                # print(f'pred:: {pred[0]}')
                for s in pred:
                    # print(f's:: {s}')
                    # self.world.debug.draw_string(s.transform.location, 'x', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
                    # print(f'btwn: {self.heuristic_c(self.map.get_waypoint(s.transform.location),self.map.get_waypoint(self.all_waypoints[600].transform.location))}')
                    # print(f'self.heuristic_c(s, u) {self.heuristic_c(s, u)}')
                    # print(f'self.g_old {self.g_old}')              
                    # print(f'self.rhs[s.id] {self.rhs[s.id]}')      
                    if self.rhs[s.id] == (self.heuristic_c(s, u) + self.g_old):
                        if s != self.goal:#?????
                            self.rhs[s.id] = (self.heuristic_c(s, u) + self.g_old)
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
            print(f'self.U {self.U.vertices_in_heap}')
            # print(f'STUFF1:::{self.U.top_key() < self.calculate_key(self.start)}')
            # print(f'STUFF2::: {self.rhs[self.start.id] > self.g[self.start.id]}')
        for i in self.U.vertices_in_heap:
            self.world.debug.draw_string(i.transform.location, f'!{self.rhs[i.id]}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
            
    def rescan(self):

        new_edges_and_old_costs = self.new_edges_and_old_costs
        self.new_edges_and_old_costs = None
        return new_edges_and_old_costs
    def main(self):
        
        self.s_last = self.start
        self.s_current = self.start
        path = [self.s_current]
        # self.initialize()
        self.compute_shortest_path()

        # while self.s_current != self.goal:
        #     # if self.g[self.s_current.id] == float('inf'):
        #     if self.rhs[self.s_current.id] != float('inf'):
        #         print("There is no known path to the goal.")
        #         return

        #     # Move to the best successor
        #     successor = self.successors(self.s_current)
        #     if not successor:
        #         print("No valid successor found.")
        #         return
        #     min_s = float('inf')
        #     arg_min = None
        #     for s_ in successor:
        #         temp = self.heuristic_c(self.s_current, s_) + self.g[s_.id]
        #         if temp < min_s:
        #             min_s = temp
        #             arg_min = s_
        #         # print(f'self.g::{self.g[s_.id]}')
        #         # print(f'self.heur::{self.heuristic_c(self.s_current, s_) }')
        #     self.s_current = arg_min
        #     # self.s_current = min(
        #     #     successor,
        #     #     key=lambda s: self.heuristic_c(self.s_current, s) + self.g[s.id]
        #     # )
        #     # print(f"temp: {self.g}")

        #     self.s_start = arg_min
        #     path.append(self.s_start)
        #     # scan graph for changed costs
        #     changed_edges_with_old_cost = self.rescan()
        #     #print("len path: {}".format(len(path)))
        #     # if any edge costs changed
        #     if changed_edges_with_old_cost:
        #         self.k_m += self.heuristic_c(self.s_last, self.s_start)
        #         self.s_last = self.s_start

        #         # for all directed edges (u,v) with changed edge costs
        #         vertices = changed_edges_with_old_cost.vertices
        #         for vertex in vertices:
        #             v = vertex.pos
        #             succ_v = vertex.edges_and_c_old
        #             for u, c_old in succ_v.items():
        #                 c_new = self.c(u, v)
        #                 if c_old > c_new:
        #                     if u != self.s_goal:
        #                         self.rhs[u] = min(self.rhs[u], self.c(u, v) + self.g[v])
        #                 elif self.rhs[u] == c_old + self.g[v]:
        #                     if u != self.s_goal:
        #                         min_s = float('inf')
        #                         succ_u = self.sensed_map.succ(vertex=u)
        #                         for s_ in succ_u:
        #                             temp = self.c(u, s_) + self.g[s_]
        #                             if min_s > temp:
        #                                 min_s = temp
        #                         self.rhs[u] = min_s
        #                     self.update_vertex(u)
        #     self.compute_shortest_path()
        #     # ===============================================================
        #     # print(f"Moving to: {self.s_current.transform.location}")

        #     # # Simulate movement (in a real scenario, you'd move the vehicle here)
        #     # time.sleep(0.5)  # Simulating movement time

        #     # # Scan for changes in edge costs
        #     # changed_edges = self.scan_for_changes()
            
        #     # if changed_edges:
        #     #     self.km += self.heuristic(self.s_last, self.s_current)
        #     #     self.s_last = self.s_current

        #     #     for u, v in changed_edges:
        #     #         c_old = self.heuristic_c(u, v)
        #     #         # Update the edge cost (in a real scenario, you'd get the new cost from the environment)
        #     #         c_new = random.uniform(0.8 * c_old, 1.2 * c_old)  # Simulating cost change
                    
        #     #         if c_old > c_new:
        #     #             if u != self.goal:
        #     #                 self.rhs[u.id] = min(self.rhs[u.id], c_new + self.g[v.id])
        #     #         elif self.rhs[u.id] == c_old + self.g[v.id]:
        #     #             if u != self.goal:
        #     #                 self.rhs[u.id] = min(
        #     #                     self.heuristic_c(u, s) + self.g[s.id]
        #     #                     for s in self.successors(u)
        #     #                 )
        #     #         self.update_vertex(u)

        #     #     self.compute_shortest_path()
        # # ===============================================================
        # print("path found!")
        # return path, self.g, self.rhs
        # print("Goal reached!")

    # # def rescan(self) -> Vertices:

    # #     new_edges_and_old_costs = self.new_edges_and_old_costs
    # #     self.new_edges_and_old_costs = None
    # #     return new_edges_and_old_costs

    # def main(self, robot_position):
    #     path = [robot_position]
    #     self.start = robot_position
    #     self.s_last = self.start
    #     self.compute_shortest_path()

    #     while self.start != self.goal:
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
    #                         if u != self.goal:
    #                             self.rhs[u] = min(self.rhs[u], heuristic_c(u, v) + self.g[v])
    #                     elif self.rhs[u] == c_old + self.g[v]:
    #                         if u != self.goal:
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
# point_a = random.choice(spawn_points)
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
point_b = spawn_points[10]
# point_b = random.choice(spawn_points)
# while point_b.location == point_a.location:
#     point_b = random.choice(spawn_points)

start_waypoint = carla_map.get_waypoint(point_a.location)
end_waypoint = carla_map.get_waypoint(point_b.location)
world.debug.draw_string(start_waypoint.transform.location, 'START', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
world.debug.draw_string(end_waypoint.transform.location, 'END', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
print("Firetruck starting at", point_a.location)
print(f"Destination: {point_b.location}")

gen_points = carla_map.generate_waypoints(1)
real_points = []
wp_pts = {}
pos = 0

for i in gen_points:
    real_points.append(carla_map.get_waypoint(i.transform.location, project_to_road=True))
    wp_pts[carla_map.get_waypoint(i.transform.location, project_to_road=True).id] = pos
    pos+=1

curr_min = gen_points[0]
for i in gen_points:
    if i.transform.location.distance(start_waypoint.transform.location) < curr_min.transform.location.distance(start_waypoint.transform.location):
        curr_min = i
get_start = curr_min
curr_min = gen_points[0]
for i in gen_points:
    if i.transform.location.distance(end_waypoint.transform.location) < curr_min.transform.location.distance(end_waypoint.transform.location):
        curr_min = i
get_end = curr_min
print(f'gen_points {gen_points[0]}')
print(f'real_points {real_points[0]}')
all_waypoints = real_points # + [get_start] +[get_end]
print(f'all_waypoints {all_waypoints[0]}')
world.debug.draw_string(all_waypoints[0].transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
print(f'get_start {get_start}')
print(f'get_end {get_end}')
world.debug.draw_string(get_start.transform.location, 'S', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
world.debug.draw_string(get_end.transform.location, 'E', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
print('============================================================')
try:

    # dstar_lite = DStarLite(world=world, start_waypoint=get_start, end_waypoint=get_end, all_waypoints=all_waypoints,wp_pts=wp_pts)
    # dstar_lite = DStarLite(world, get_end, get_start, all_waypoints, wp_pts)
    dstar_lite = DStarLite(world, get_start, get_end, all_waypoints, wp_pts)
    dstar_lite.initialize()
    dstar_lite.main()

finally:
     # Clean up
    firetruck.destroy()
    print('Firetruck destroyed successfully')