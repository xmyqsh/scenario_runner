from __future__ import print_function
import math
import json
import numpy as np
import xml.etree.ElementTree as ET
"""
    Module use to parse all the route configuration parameters
"""



def parse_annotations_file(annotation_filename):
    # Return the annotations of which positions where the scenarios are going to happen.\

    with open(annotation_filename, 'r') as f:
        annotation_dict = json.loads(f.read())

    # Todo, add checks for file structure errors, such as weird names and things

    return annotation_dict


def parse_routes_file(route_filename):
    """
        Returns a list of route elements that is where the challenge is going to happen.

    Args
        route_filename: the path to a set of routes.
    Returns
        List of dicts containing the waypoints, id and town of the routes
    """

    list_route_descriptions = []
    tree = ET.parse(route_filename)
    for route in tree.iter("route"):
        route_town = route.attrib['map']
        route_id = route.attrib['id']
        waypoint_list = []  # the list of waypoints that can be found on this route
        for waypoint in route.iter('waypoint'):
             waypoint_list.append(waypoint)  # Waypoints is basically a list of XML nodes

        list_route_descriptions.append( {
                                    'id': route_id,
                                    'town_name': route_town,
                                    'trajectory': waypoint_list
                                     })

    return list_route_descriptions


def create_location_waypoint(location):
    # Function to correct frans weird names.
    return {

        'x': location['Cords']['x'],
        'y': location['Cords']['y'],
        'z': location['Cords']['z'],
        'yaw': location['Yaw'],
        'pitch': location['Picth']

    }


def remove_redundancy(list_of_vehicles):
    """
       We have a redundant vec of dics. Eliminate it for now.
    """
    vehicle_dict = {}
    for mono_dict in list_of_vehicles:
        vehicle_dict.update(mono_dict)

    return vehicle_dict


def scan_route_for_scenarios(route_description, world_annotations):

    """
    Just returns a plain list of possible scenarios that can happen in this route by matching
    the locations from the scenario into the route description

    For what I understood it does not matter where it actually matches!

    Returns
        A list of scenario definitions with their correspondent parameters
    """

    def match_world_location_to_route(world_location, route_description):

        """
        We match this location to a given route.
            world_location:
            route_description:
        """
        def match_waypoints(w1, wnode):
            dx = w1['x'] - float(wnode.attrib['x'])
            dy = w1['y'] - float(wnode.attrib['y'])
            dz = w1['z'] - float(wnode.attrib['z'])
            dist_position = math.sqrt(dx * dx + dy * dy + dz * dz)
            dyaw = w1['yaw'] - float(wnode.attrib['yaw'])
            dpitch = w1['pitch'] - float(wnode.attrib['pitch'])

            dist_angle = math.sqrt(dyaw * dyaw + dpitch * dpitch)

            return dist_angle < 1 and dist_position < 1  # TODO  check this threshold, I have no idea

        # TODO this function can be optimized to run on Log(N) time
        for route_waypoint in route_description:
            if match_waypoints(world_location, route_waypoint):
                return True

        return False

    possible_scenarios = []

    for town_name, scenarios in world_annotations.items():

        if town_name != route_description['town_name']:
            continue

        for scenario in scenarios:  # For each existent scenario
            scenario_type = scenario["scenario_type"]
            for location in scenario["Available_Localizations"]:
                waypoint = create_location_waypoint(location)   # Function until fran fixes the format

                if match_world_location_to_route(waypoint, route_description['trajectory']):
                    # We match a location for this scenario, create a scenario object so this scenario
                    # can be instantiated later

                    if 'Antagonist_Vehicles' in location:
                        other_vehicles = remove_redundancy(location['Antagonist_Vehicles'])
                    else:
                        other_vehicles = None

                    scenario_description = {
                                           'name': scenario_type,
                                           'Antagonist_Vehicles': other_vehicles,
                                           'trigger_position': waypoint
                                           }
                    possible_scenarios.append(scenario_description)

    return possible_scenarios


################################
    #ROUTE MAKING
#################################
#
# def turn_decision(self, index, route, threshold=math.radians(5)):
#     """
#     This method returns the turn decision (RoadOption) for pair of edges
#     around current index of route list
#     """
#
#     decision = None
#     previous_node = route[index - 1]
#     current_node = route[index]
#     next_node = route[index + 1]
#     next_edge = self._graph.edges[current_node, next_node]
#     if index > 0:
#         current_edge = self._graph.edges[previous_node, current_node]
#         calculate_turn = current_edge['type'] == RoadOption.LANEFOLLOW and \
#                          not current_edge['intersection'] and \
#                          next_edge['type'] == RoadOption.LANEFOLLOW and \
#                          next_edge['intersection']
#         if calculate_turn:
#             cv, nv = current_edge['exit_vector'], next_edge['net_vector']
#             cross_list = []
#             for neighbor in self._graph.successors(current_node):
#                 select_edge = self._graph.edges[current_node, neighbor]
#                 if select_edge['type'] == RoadOption.LANEFOLLOW:
#                     if neighbor != route[index + 1]:
#                         sv = select_edge['net_vector']
#                         cross_list.append(np.cross(cv, sv)[2])
#             next_cross = np.cross(cv, nv)[2]
#             deviation = math.acos(np.dot(cv, nv) / \
#                                   (np.linalg.norm(cv) * np.linalg.norm(nv)))
#             if not cross_list:
#                 cross_list.append(0)
#             if deviation < threshold:
#                 decision = RoadOption.STRAIGHT
#             elif cross_list and next_cross < min(cross_list):
#                 decision = RoadOption.LEFT
#             elif cross_list and next_cross > max(cross_list):
#                 decision = RoadOption.RIGHT
#         else:
#             decision = next_edge['type']
#     else:
#         decision = next_edge['type']
#
#     return decision
#
#
# def abstract_route_plan(route):
#     """
#     The following function generates the route plan based on
#     origin      : carla.Location object of the route's start position
#     destination : carla.Location object of the route's end position
#     return      : list of turn by turn navigation decisions as
#     agents.navigation.local_planner.RoadOption elements
#     Possible values are STRAIGHT, LEFT, RIGHT, LANEFOLLOW, VOID
#     CHANGELANELEFT, CHANGELANERIGHT
#     """
#
#     plan = []
#
#     for i in range(len(route) - 1):
#         road_option = turn_decision(i, route)
#         plan.append(road_option)
#
#     return plan
#
#
# def _find_closest_in_list(current_waypoint, waypoint_list):
#     min_distance = float('inf')
#     closest_index = 0
#     for i, waypoint in enumerate(waypoint_list):
#         distance = waypoint.transform.location.distance(
#             current_waypoint.transform.location)
#         if distance < min_distance:
#             min_distance = distance
#             closest_index = i
#
#     return closest_index
