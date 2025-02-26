import carla
import random
import time
# from queue import PriorityQueue
from PriorityQueueDLite import PriorityQueue, Priority
import sys
import keyboard
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
        self.s_last = None
        self.s_current = None
        self.all_waypoints = all_waypoints
        self.wp_pos = wp_pts
        self.part1 = []
        print('init successfully')
        self.new_edges_and_old_costs = None
        
        # why are end_waypoint and start_waypoint in reverse but self.goal and self.start aren't?
        # self.world.debug.draw_string(end_waypoint.transform.location, 'EEEEEEEEEE', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
        # self.world.debug.draw_string(start_waypoint.transform.location, 'SSSSSSSSSS', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
    def successors(self,waypoint):
        neighbors = []
        # Forward neighbor
        forward = waypoint.next(3)

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
            x = neighbors[i]
            # if self.g.get(neighbors[i].id) is None:
            for z in self.part1+[self.goal]:
                if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:
                    x = z
                    initial_dist=neighbors[i].transform.location.distance(z.transform.location)

            neighbors[i] = x
        
        return neighbors
    def predecessors(self, waypoint):

        neighbors = []
        # Backward neighbor
        Backward = waypoint.previous(3)
        if Backward:
            neighbors.extend(Backward)
        
        # Legal left lane change
        if waypoint.lane_change & carla.LaneChange.Left:
            left_lane = waypoint.get_left_lane()
            if left_lane and left_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(left_lane)
                # self.world.debug.draw_string(left_lane.transform.location, 'L', draw_shadow=False, color=carla.Color(r=220, g=0, b=220), life_time=30.0, persistent_lines=True)
        
        # Legal right lane change
        if waypoint.lane_change & carla.LaneChange.Right:
            right_lane = waypoint.get_right_lane()
            if right_lane and right_lane.lane_type == carla.LaneType.Driving:
                neighbors.append(right_lane)
                # self.world.debug.draw_string(right_lane.transform.location, 'R', draw_shadow=False, color=carla.Color(r=220, g=0, b=220), life_time=30.0, persistent_lines=True)

        x = [0]*len(neighbors)
        for i in range(len(neighbors)):
            # initial_dist = 0.2
            initial_dist = neighbors[i].transform.location.distance(waypoint.transform.location)
            # if self.g.get(neighbors[i].id) is None:
            for z in self.all_waypoints:
                if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:# and (waypoint.road_id == z.road_id or waypoint.lane_id == z.lane_id):
                    x[i] = z
                    initial_dist=neighbors[i].transform.location.distance(z.transform.location)
            # vvv is why i use range and not just for i in neighbors, can't assign i=x
            # neighbors[i] = x

        for i in range(len(neighbors)):
            if x[i] != 0:
                neighbors[i] = x[i]

        # for i in range(len(neighbors)):
        #     initial_dist = neighbors[i].transform.location.distance(waypoint.transform.location)
        #     # if self.g.get(neighbors[i].id) is None:
        #     for z in self.all_waypoints:
        #         if neighbors[i].transform.location.distance(z.transform.location) < initial_dist:# and (waypoint.road_id == z.road_id or waypoint.lane_id == z.lane_id):
        #             x[i] = z
        #             initial_dist=neighbors[i].transform.location.distance(z.transform.location)
        #     # vvv is why i use range and not just for i in neighbors, can't assign i=x
        #     # neighbors[i] = x
        # for i in range(len(neighbors)):
        #     if x[i] != 0:
        #         neighbors[i] = x[i]
        #         # print(f'neighbors {neighbors[i]}')
        self.part1.extend(neighbors)
        # draw all predecessors
        for i in neighbors:
            self.world.debug.draw_string(i.transform.location, f'{self.g[i.id]}', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=30.0, persistent_lines=True)
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
        return Priority(
            min(self.g[s.id], self.rhs[s.id]) + self.heuristic(s, self.start) + self.km,
            min(self.g[s.id], self.rhs[s.id])
            )


    def initialize(self):
        self.U = PriorityQueue()
        self.km = 0
        # check = 0
        for s in self.all_waypoints:
            self.rhs[s.id] = float('inf')
            self.g[s.id] = float('inf')
        self.g[self.goal.id]=float('inf')
        self.rhs[self.goal.id] = 0
        self.g[self.start.id]=float('inf')
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

        self.world.debug.draw_string(self.goal.transform.location, 'goal', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
        pred_list = self.predecessors(self.goal)
        # for i in range(len(pred_list)):
        #     self.world.debug.draw_string(pred_list[i].transform.location, f'init:{i}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)

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
            # print('1')
            # print(f'self.U {self.U.top_key()}') # (g, rhs)

        elif self.g[u.id] != self.rhs[u.id] and not self.contain(u):
            # if node hasn't been processed to its optimal cost yet
            self.U.insert(u, self.calculate_key(u))
            # print('2')
            # print(f'self.U {self.U.top_key()}') # (g, rhs)
            # self.world.debug.draw_string(u.transform.location, 'U', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=200.0, persistent_lines=True)

        elif self.g[u.id] == self.rhs[u.id] and self.contain(u):
            # have already optimized this node but still in the open set(lookup)
            self.U.remove(u)
            # print('3')
            # print(f'self.U {self.U.top_key()}') # (g, rhs)

    def compute_shortest_path(self):
        while (self.U.top_key() < self.calculate_key(self.start)) or (self.rhs[self.start.id] > self.g[self.start.id]):
            u = self.U.top() # waypoint
            # self.world.debug.draw_string(u.transform.location, 'U', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=200.0, persistent_lines=True)
            k_old = self.U.top_key() # (g, rhs) which is (heuristic, min(g, rhs))
            k_new = self.calculate_key(u)

            if k_old < k_new: # if the waypoint is not up to date in the open set
                # print('compute_shortest_path 1')
                self.U.update(u, k_new)
            elif self.g[u.id] > self.rhs[u.id]: # if a more optimal path is found
                # print('compute_shortest_path 2')
                # print(f'self.g {self.g[u.id]}')
                # print(f'self.rhs {self.rhs[u.id]}')
                self.g[u.id] = self.rhs[u.id]
                # print(f'self.g {self.g[u.id]}')

                self.U.remove(u)
                for s in self.predecessors(u):
                    if s != self.goal:
                        
                        self.rhs[s.id] = min(self.rhs[s.id], self.heuristic_c(s, u) + self.g[u.id])
                    self.update_vertex(s)
                # self.world.debug.draw_string(u.transform.location, 'R', draw_shadow=False, color=carla.Color(r=0, g=0, b=220), life_time=60.0, persistent_lines=True)
            else:
                # print('compute_shortest_path 3')
                self.g_old = self.g[u.id]
                self.g[u.id] = float('inf')
                pred = self.predecessors(u)
                pred.append(u)
                for s in pred:   
                    self.world.debug.draw_string(s.transform.location, 'SSSSSSS', draw_shadow=False, color=carla.Color(r=220, g=0, b=00), life_time=30.0, persistent_lines=True)
                    if self.rhs[s.id] == self.heuristic_c(s, u) + self.g_old:
                        print('locally consistent!')
                        if s != self.goal:#?????
                            # self.rhs[s.id] = (self.heuristic_c(s, u) + self.g_old)
                            min_s = float('inf')
                            # succ = self.sensed_map.successors(vertex=s)
                            print('pred:', pred)
                            succ = self.successors(s)
                            for s_ in succ:
                                temp = self.heuristic_c(s, s_) + self.g[s_.id]
                                if min_s > temp:
                                    min_s = temp
                                # temp = min( self.heuristic_c(s, s_) + self.g[s_.id], min_s)
                            self.rhs[s.id] = min_s
                    self.update_vertex(u)
            # print(f'self.U {self.U.vertices_in_heap}')
            # print(f'STUFF1:::{self.U.top_key() < self.calculate_key(self.start)}')
            # print(f'STUFF2::: {self.rhs[self.start.id] > self.g[self.start.id]}')
        # for i in self.U.heap:
        #     print(f'U: k1={i.priority.k1}||k2={i.priority.k2}||v={i.vertex}')


        # for i in self.U.vertices_in_heap:
        #     self.world.debug.draw_string(i.transform.location, f'!{self.rhs[i.id]}', draw_shadow=False, color=carla.Color(r=110, g=0, b=220), life_time=60.0, persistent_lines=True)
        # print(f'len(self.U.vertices_in_heap) {len(self.U.vertices_in_heap)}')
        # print(f'len(self.U) {len(self.U)}')
            
    # def rescan(self):

    #     new_edges_and_old_costs = self.new_edges_and_old_costs
    #     self.new_edges_and_old_costs = None
    #     return new_edges_and_old_costs
    def main(self):
        
        self.s_last = self.start
        self.s_current = self.start
        path = [self.s_current]
        # self.initialize()
        self.compute_shortest_path()
        # for i in self.all_waypoints:
        #     self.world.debug.draw_string(i.transform.location, f'{self.g[i.id]}', draw_shadow=False, color=carla.Color(r=0, g=220, b=220), life_time=30.0, persistent_lines=True)
        # for i in self.U.vertices_in_heap:
        #     self.world.debug.draw_string(i.transform.location, f'{self.rhs[i.id]}', draw_shadow=False, color=carla.Color(r=0, g=220, b=220), life_time=30.0, persistent_lines=True)
        print(f'self.s_current {self.s_current}')
        while True:
            # if self.g[self.s_current.id] == float('inf'):
            if self.rhs[self.start.id] == float('inf'):
                print("There is no known path to the goal.")
                return

            # Move to the best successor
            successor = self.successors(self.s_current)
            if not successor:
                print("No valid successor found.")
                return
            min_s = float('inf')
            arg_min = None
            print('bef move')
            for s_ in successor:
                # print(f's_ {s_}')
                self.world.debug.draw_string(s_.transform.location, f'{self.g[s_.id]}', draw_shadow=False, color=carla.Color(r=220, g=220, b=0), life_time=30.0, persistent_lines=True)
                temp = self.heuristic_c(self.s_current, s_) + self.g[s_.id]
                if temp < min_s:
                    min_s = temp
                    arg_min = s_
            # Moving the car to the best successor
            self.s_current = arg_min
            print('MOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVED')
            print(f'self.s_current {self.s_current}')
            if self.s_current.transform.location.distance(self.goal.transform.location) < 3.5: # see if i can make it work with 2.0
                print('ARRIVED')
                break
            self.s_start = arg_min
            print(f'start: {self.s_start}')
            path.append(self.s_start)

            # scan graph for changed costs
            if(keyboard.is_pressed('space')):
                print('scanning graph...')
            # changed_edges_with_old_cost = self.rescan()
            # # if any edge costs changed
            # if changed_edges_with_old_cost:
            #     self.k_m += self.heuristic_c(self.s_last, self.s_start)
            #     self.s_last = self.s_start

            #     # for all directed edges (u,v) with changed edge costs
            #     vertices = changed_edges_with_old_cost.vertices
            #     for vertex in vertices:
            #         v = vertex.pos
            #         succ_v = vertex.edges_and_c_old
            #         for u, c_old in succ_v.items():
            #             c_new = self.c(u, v)
            #             if c_old > c_new:
            #                 if u != self.s_goal:
            #                     self.rhs[u] = min(self.rhs[u], self.c(u, v) + self.g[v])
            #             elif self.rhs[u] == c_old + self.g[v]:
            #                 if u != self.s_goal:
            #                     min_s = float('inf')
            #                     succ_u = self.sensed_map.succ(vertex=u)
            #                     for s_ in succ_u:
            #                         temp = self.c(u, s_) + self.g[s_]
            #                         if min_s > temp:
            #                             min_s = temp
            #                     self.rhs[u] = min_s
            #                 self.update_vertex(u)
            self.compute_shortest_path()
        print('Done!')

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
# print(f'real_points {real_points[0]}')
for i in gen_points + [get_start] +[get_end]:
    real_points.append(carla_map.get_waypoint(i.transform.location, project_to_road=True))
    wp_pts[carla_map.get_waypoint(i.transform.location, project_to_road=True).id] = pos
    pos+=1
all_waypoints = real_points # + [get_start] +[get_end]
print(f'all_waypoints {all_waypoints[0]}')
# world.debug.draw_string(all_waypoints[0].transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
print(f'get_start {get_start}')
print(f'get_end {get_end}')
world.debug.draw_string(get_start.transform.location, 'S', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
world.debug.draw_string(get_end.transform.location, 'E', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=60.0, persistent_lines=True)
print('============================================================')
try:
    waypoint_tuple_list = carla_map.get_topology()
    for i in waypoint_tuple_list:
        world.debug.draw_string(i[0].transform.location, 'efw', draw_shadow=False, color=carla.Color(r=250, g=0, b=00), life_time=30.0, persistent_lines=True)

    # dstar_lite = DStarLite(world=world, start_waypoint=get_start, end_waypoint=get_end, all_waypoints=all_waypoints,wp_pts=wp_pts)
    # dstar_lite = DStarLite(world, get_end, get_start, all_waypoints, wp_pts)
    # for i in range(len(all_waypoints)-3):
    #     world.debug.draw_string(all_waypoints[i].transform.location, f'{all_waypoints[i].road_id}', draw_shadow=False, color=carla.Color(r=250, g=0, b=00), life_time=30.0, persistent_lines=True)
    dstar_lite = DStarLite(world, get_start, get_end, all_waypoints, wp_pts)
    dstar_lite.initialize()
    dstar_lite.main()

finally:
     # Clean up
    firetruck.destroy()
    print('Firetruck destroyed successfully')