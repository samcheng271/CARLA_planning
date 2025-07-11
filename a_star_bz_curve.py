import carla
import random
import time
import numpy as np
from queue import PriorityQueue
from itertools import tee
import math


def euclidean_heuristic(waypoint, end_waypoint):
    return waypoint.transform.location.distance(end_waypoint.transform.location)

def manhattan_heuristic(waypoint, end_waypoint):
    dx = abs(waypoint.transform.location.x - end_waypoint.transform.location.x)
    dy = abs(waypoint.transform.location.y - end_waypoint.transform.location.y)
    dz = abs(waypoint.transform.location.z - end_waypoint.transform.location.z)
    return dx + dy + dz

def cubic_bz(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (u**3) * p0 \
         + 3 * (u**2) * t * p1 \
         + 3 * u * (t**2) * p2 \
         + (t**3) * p3

def pairwise(iterable):
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

def bz_velocity(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> np.ndarray:
    """First derivative of a cubic Bézier"""
    u = 1.0 - t
    return 3 * ((u**2) * (p1 - p0) + 2 * u * t * (p2 - p1) + (t**2) * (p3 - p2))

def bz_acc(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> np.ndarray:
    """Second derivative of a cubic Bézier"""
    u = 1.0 - t
    return 6 * (u * (p2 - 2*p1 + p0) + t * (p3 - 2*p2 + p1))

'''
def bz_jerk(p0, p1, p2, p3, t):
    u = 1.0 - t
    return 6 * ((p3 - 3*p2 + 3*p1 - p0)) 
'''

def bz_curvature(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> float:
    d1 = bz_velocity(p0, p1, p2, p3, t)
    d2 = bz_acc(p0, p1, p2, p3, t)
    cross = np.cross(d1, d2)
    num   = np.linalg.norm(cross)
    den   = np.linalg.norm(d1)**3
    return float(num / den)
'''
def jaggedness(route):
    xy_list = []
    for wp in route:
        if hasattr(wp, "location"):
            wp_loc = wp.location
        
        else: 
            if hasattr(wp, "transform") and hasattr(wp.transform, "location"):
                wp_loc = wp.transform.location
        
        xy_list.append([wp_loc.x, wp_loc.y])
    if len(xy_list) < 3:
        return 0.0
    xy = np.stack(xy_list, axis=0)
    head_angle = np.unwrap(np.arctan2(np.diff(xy[:,1]), np.diff(xy[:,0])))
    return np.sum(np.abs(np.diff(head_angle)))
'''
def jaggedness(route): 
    wp_xy = []
    for wp in route: 
        if hasattr(wp, "location"):
            wp_loc = wp.location
        else: 
            if hasattr(wp, "transform") and hasattr(wp.transform, "location"):
                wp_loc = wp.transform.location
        wp_xy.append([wp_loc.x, wp_loc.y])
    if(len(wp_xy) < 3):
        return "zero"
    stacked_list = np.stack(wp_xy, axis = 0)
    head_angle = np.unwrap(np.arctan2(np.diff(stacked_list[:,1]), np.diff(stacked_list[:,0])))
    return np.sum(np.abs(np.diff(head_angle)))

def loc_to_vec(loc: carla.Location) -> np.ndarray:
    return np.array([loc.x, loc.y, loc.z], dtype=float)

def vec_to_loc(v: np.ndarray) -> carla.Location:
    return carla.Location(float(v[0]), float(v[1]), float(v[2]))

class AStarNode:
    def __init__(self, waypoint, g_cost, h_cost, parent=None):
        self.waypoint = waypoint
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent

def get_legal_neighbors(waypoint):
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

def smooth_lane_change(route, curve_pts=30):
    smooth_pts = []
    i = 0
    while i < len(route)-1:
        curr = route[i]
        next  = route[i+1]
        if (curr.lane_id != next.lane_id):

            p0_wp = route[i-1]
            p1_wp = curr
            p2_wp = next
            p3_wp = route[i+2]

            p0 = loc_to_vec(p0_wp.transform.location)
            p1 = loc_to_vec(p1_wp.transform.location)
            p2 = loc_to_vec(p2_wp.transform.location)
            p3 = loc_to_vec(p3_wp.transform.location)

            if not smooth_pts:
                smooth_pts.append(p0_wp.transform)
            for t in np.linspace(0.0, 1.0, curve_pts, endpoint=True):
                pt = cubic_bz(p0, p1, p2, p3, t)
                curve = bz_curvature(p0, p1, p2, p3, t)
                print(f"t={t:.3}, curvature={curve:.6f}")
                loc = vec_to_loc(pt)
                rot = p3_wp.transform.rotation 
                smooth_pts.append(carla.Transform(loc, rot))
            i += 2 
        else:
            smooth_pts.append(curr.transform)
            i += 1
    smooth_pts.append(route[-1].transform)
    return smooth_pts

def draw_bz(world, route, samples=30):
    for i, (curr_wp, next_wp) in enumerate(pairwise(route)):
        if (curr_wp.lane_id != next_wp.lane_id):
            p0_loc = route[i-1].transform.location
            p1_loc = curr_wp.transform.location
            p2_loc = next_wp.transform.location
            p3_loc = route[i+2].transform.location

            if i-2 >= 0:
                prev_loc = route[i-2].transform.location
                world.debug.draw_line(
                    prev_loc, p0_loc,
                    thickness=0.05,
                    color=carla.Color(0,0,255),
                    life_time=10.0
                )
            
            p0 = np.array([p0_loc.x, p0_loc.y, p0_loc.z])
            p1 = np.array([p1_loc.x, p1_loc.y, p1_loc.z])
            p2 = np.array([p2_loc.x, p2_loc.y, p2_loc.z])
            p3 = np.array([p3_loc.x, p3_loc.y, p3_loc.z])
            
            prev_pt = cubic_bz(p0, p1, p2, p3, 0.0)
            for t in np.linspace(0.0, 1.0, samples, endpoint=True)[1:]:
                curr_pt = cubic_bz(p0, p1, p2, p3, t)
                loc0 = carla.Location(prev_pt[0], prev_pt[1], prev_pt[2])
                loc1 = carla.Location(curr_pt[0], curr_pt[1], curr_pt[2])
                world.debug.draw_line(
                    loc0, loc1,
                    thickness=0.05,
                    color=carla.Color(0,0,255),
                    life_time=10.0
                )
                prev_pt = curr_pt
            if i+3 < len(route):
                next_loc = route[i+3].transform.location
                world.debug.draw_line(
                    p3_loc, next_loc,
                    thickness=0.05,
                    color=carla.Color(0,0,255),
                    life_time=10.0
                )

def a_star(world, start_waypoint, end_waypoint, heuristic_func=euclidean_heuristic, max_distance=5000):
    start_node = AStarNode(start_waypoint, 0, heuristic_func(start_waypoint, end_waypoint))
    open_set = PriorityQueue()
    open_set.put((start_node.f_cost, id(start_node), start_node))
    came_from = {}
    g_score = {start_waypoint.id: 0}
    f_score = {start_waypoint.id: start_node.f_cost}
    
    while not open_set.empty():
        current_node = open_set.get()[2]
        
        # Early exit if we have reached near the goal
        if current_node.waypoint.transform.location.distance(end_waypoint.transform.location) < 2.0:
            path = []

            while current_node:
                path.append(current_node.waypoint)
                current_node = came_from.get(current_node.waypoint.id)
            return list(reversed(path))
  
        for next_waypoint in get_legal_neighbors(current_node.waypoint):
            # world.debug.draw_string(next_waypoint.transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=220), life_time=60.0, persistent_lines=True)
            # Add a small cost for lane changes
            lane_change = next_waypoint.lane_id != current_node.waypoint.lane_id
            lane_change_cost = 5 if lane_change else 0

            tentative_g_score = g_score[current_node.waypoint.id] + euclidean_heuristic(current_node.waypoint, next_waypoint) + lane_change_cost
            # If the next waypoint is already in the open set, we can skip it
            # Comparing g_score for the reason above tentative_g_score.

            if next_waypoint.id not in g_score or tentative_g_score < g_score[next_waypoint.id]:
                # Draws the possible routes A* checked
                world.debug.draw_string(next_waypoint.transform.location, '^', draw_shadow=False, color=carla.Color(r=0, g=220, b=0), life_time=25.0, persistent_lines=True)
                
                came_from[next_waypoint.id] = current_node
                
                g_score[next_waypoint.id] = tentative_g_score
                f_score[next_waypoint.id] = tentative_g_score + heuristic_func(next_waypoint, end_waypoint)
                new_node = AStarNode(next_waypoint, tentative_g_score, heuristic_func(next_waypoint, end_waypoint), current_node)
                open_set.put((f_score[next_waypoint.id], id(new_node), new_node))
                # In this implementation, the f_score is implicitly used to determine the next waypoint through the PriorityQueue.
                # Therefure, the f_score is used to determine the next waypoint through the PriorityQueue.
    print("A* search failed to find a path")
    return None

def main():
    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 4000)
        client.set_timeout(10.0)
        # world = client.load_world('Town05') # Use this to switch towns
        # Get the world and map
        world = client.get_world()
        carla_map = world.get_map()

        # Spawn a firetruck at a random location (point A)
        blueprint_library = world.get_blueprint_library()
        firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        spawn_points = carla_map.get_spawn_points()


        start_ind = 25
        end_ind = 35
        # Choose a random starting location (point A)
        #point_a = random.choice(spawn_points)
        point_a = spawn_points[start_ind]
        firetruck = world.spawn_actor(firetruck_bp, point_a)

        # Choose a random destination (point B)
        point_b  = spawn_points[end_ind]
        #point_b = random.choice(spawn_points)
        while point_b.location == point_a.location:
            point_b = random.choice(spawn_points)

        start_waypoint = carla_map.get_waypoint(point_a.location)
        end_waypoint = carla_map.get_waypoint(point_b.location)

        print("Firetruck starting at", point_a.location)
        print(f"Destination: {point_b.location}")

        print(f"Point A wp: {start_waypoint}")
        print(f"Point B wp: {end_waypoint}")

        # Run A*
        route = a_star(world, start_waypoint, end_waypoint)
        print("A* alone jaggedness:", jaggedness(route))

        if route is None:
            # To prevent infinite loop
            print("Failed to find a path. Try adjusting the max_distance in the a_star function.")
            firetruck.destroy()
            return
        draw_bz(world, route,samples=30)
        print(f"Route has {len(route)} waypoints")

        # Keeping for debugging purposes
        # start_time = time.time()
        # timeout = 300  # 5 minutes timeout
        # route = carla_map.generate_waypoints(2.0)
        # Draws the route the vehicle will follow (red)
        for waypoint in route:
            world.debug.draw_string(waypoint.transform.location, '^', draw_shadow=False, color=carla.Color(r=220, g=0, b=0), life_time=25.0, persistent_lines=True)

        # Follow the route
        smooth_route = smooth_lane_change(route, curve_pts=12)
        print("A* bz jaggedness:", jaggedness(smooth_route))
        #log_spacing(smooth_route)
        for i, tr in enumerate(smooth_route):
            firetruck.set_transform(tr)
            time.sleep(0.05)
        print("Firetruck has reached its destination or the route has ended!")
    finally:
        # Clean up
        firetruck.destroy()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')