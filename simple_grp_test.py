

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)
world = client.load_world('Town01')

dao = GlobalRoutePlannerDAO(world.get_map(), 2.0)
grp = GlobalRoutePlanner(dao)
grp.setup()
# Obtain route plan

start_point = carla.Location(x=338.7027893066406,
                             y=226.75003051757812,
                             z=0)

print("    BEGIN ", start_point)
end_point = carla.Location(x=321.98931884765625,
                           y=194.67242431640625,
                           z=0)

interpolated_trace = grp.trace_route(start_point, end_point)

for wp in interpolated_trace:
    print(wp[0].transform.location, wp[0].transform.rotation)

print("    END ", end_point)