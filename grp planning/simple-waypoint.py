import carla
import sys

sys.path.append('../')
from agents.navigation.global_route_planner import GlobalRoutePlanner
# from agents.navigation.global_route_planner_og import GlobalRoutePlanner # original route planner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = carla.Client("localhost", 4000)
client.set_timeout(10)
world = client.load_world('Town01')
amap = world.get_map()
sampling_resolution = 2
# dao = GlobalRoutePlannerDAO(amap, sampling_resolution)
grp = GlobalRoutePlanner(amap, sampling_resolution)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()
# print(spawn_points)
point_a = carla.Location(spawn_points[50].location)
point_b = carla.Location(spawn_points[100].location)

w1 = grp.trace_route(point_a, point_b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
# w1 = grp.trace_route(point_a, point_b) # original_usage
i = 0
for w in w1:
    print(w[0].transform.location.x, ",",w[0].transform.location.y, w[1])
    if i % 10 == 0:
        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
        color=carla.Color(r=255, g=0, b=0), life_time=120.0,
        persistent_lines=True)
    else:
        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
        color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
        persistent_lines=True)
    i += 1