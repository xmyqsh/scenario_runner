#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Challenge Evaluator Routes

Provisional code to evaluate Autonomous Agents for the CARLA Autonomous Driving challenge
"""
from __future__ import print_function
import argparse
from argparse import RawTextHelpFormatter
import importlib
import sys
import os

import xml.etree.ElementTree as ET

import carla
import srunner.challenge.utils.route_configuration_parser as parser
from srunner.challenge.envs.server_manager import ServerManagerBinary, ServerManagerDocker
#from srunner.scenarios.challenge_basic import *
#from srunner.scenarios.config_parser import *
#from srunner.scenariomanager.scenario_manager import ScenarioManager

from srunner.scenariomanager.carla_data_provider import CarlaActorPool

from srunner.scenarios.control_loss import ControlLoss
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
from srunner.scenarios.object_crash_vehicle import StationaryObjectCrossing, DynamicObjectCrossing
from srunner.scenarios.object_crash_intersection import VehicleTurningRight, VehicleTurningLeft
from srunner.scenarios.opposite_vehicle_taking_priority import OppositeVehicleRunningRedLight
from srunner.scenarios.signalized_junction_left_turn import SignalizedJunctionLeftTurn
from srunner.scenarios.no_signal_junction_crossing import NoSignalJunctionCrossing
from srunner.scenarios.master import Master
# TODO maybe the parsing and the building are actually different.

# The configuration parser

from srunner.scenarios.config_parser import ActorConfiguration, ScenarioConfiguration, RouteConfiguration
from srunner.scenariomanager.traffic_events import TrafficEvent, TrafficEventType



from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO


#############  add the parse trajectory here directly.
##### TODO: remove it a bit after



def parse_trajectory(world, waypoints_trajectory):
    # INstantiate praveens full module
    # Setup the GRP
    hop_resolution = 2.0
    dao = GlobalRoutePlannerDAO(world.get_map(), hop_resolution)
    grp = GlobalRoutePlanner(dao)
    grp.setup()

    # Obtain route plan
    route = []
    for i in range(len(waypoints_trajectory) -1):   # Goes until the one before the last.

        waypoint = waypoints_trajectory[i]
        waypoint_next = waypoints_trajectory[i]

        route = grp.trace_route( carla.Location(x=float(waypoint.attrib['x']),
                                                y=float(waypoint.attrib['y']),
                                                z=float(waypoint.attrib['z'])),
                                 carla.Location(x=float(waypoint_next.attrib['x']),
                                                y=float(waypoint_next.attrib['y']),
                                                z=float(waypoint_next.attrib['z']))
                                 )

        print (route)

    return waypoints_trajectory, waypoints_trajectory




number_class_translation = {


    "Scenario 1": [ControlLoss],
    "Scenario 2": [FollowLeadingVehicle],   # ToDO there is more than one class depending on the scenario configuraiton
    "Scenario 3": [StationaryObjectCrossing, DynamicObjectCrossing],
    "Scenario 4": [VehicleTurningRight, VehicleTurningLeft],
    "Scenario 5": [],
    "Scenario 6": [],
    "Scenario 7": [OppositeVehicleRunningRedLight],
    "Scenario 8": [SignalizedJunctionLeftTurn],
    "Scenario 9": [],
    "Scenario 10": [NoSignalJunctionCrossing]


}


# Import now all the possible scenarios

from srunner.scenarios.challenge_basic import ChallengeBasic


def convert_json_to_actor(actor_dict):
    node = ET.Element('waypoint')
    node.set('x', actor_dict['x'])
    node.set('y', actor_dict['y'])
    node.set('z', actor_dict['z'])

    return ActorConfiguration(node)


class ChallengeEvaluator(object):

    """
    Provisional code to evaluate AutonomousAgent performance
    """

    ego_vehicle = None
    actors = []

    # Tunable parameters
    client_timeout = 15.0   # in seconds
    wait_for_world = 10.0  # in seconds

    # CARLA world and scenario handlers
    world = None
    manager = None

    def __init__(self, args):
        self.output_scenario = []
        self.master_scenario = None
        # first we instantiate the Agent
        #module_name = os.path.basename(args.agent).split('.')[0]
        #module_spec = importlib.util.spec_from_file_location(module_name, args.agent)
        #self.module_agent = importlib.util.module_from_spec(module_spec)
        #module_spec.loader.exec_module(self.module_agent)

        self._sensors_list = []
        self._hop_resolution = 2.0

        # instantiate a CARLA server manager
        if args.use_docker:
            self._carla_server = ServerManagerDocker({'DOCKER_VERSION': args.docker_version})

        else:
            self._carla_server = ServerManagerBinary({'CARLA_SERVER': "{}/CarlaUE4.sh".format(args.carla_root)})



    def cleanup(self, ego=False):
        """
        Remove and destroy all actors
        """

        # We need enumerate here, otherwise the actors are not properly removed
        for i, _ in enumerate(self.actors):
            if self.actors[i] is not None:
                self.actors[i].destroy()
                self.actors[i] = None
        self.actors = []

        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                self._sensors_list[i].destroy()
                self._sensors_list[i] = None
        self._sensors_list = []

        if ego and self.ego_vehicle is not None:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None

    def __del__(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """

        self.cleanup(True)
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world




    def scenario_sampling(self, potential_scenarios_definitions):
        return potential_scenarios_definitions

    # convert to a better json
    def get_actors_instances(self, list_of_antagonist_actors):
        """
        Get the full list of actor instances.
        """

        def get_actors_from_list(list_of_actor_def):
            """
                Receives a list of actor definitions and creates an actual list of ActorConfigurationObjects

            """
            sublist_of_actors = []
            for actor_def in list_of_actor_def:
                sublist_of_actors.append(convert_json_to_actor(parser.create_location_waypoint(actor_def)))

            return sublist_of_actors

        list_of_actors = []
        # Parse vehicles to the left
        if 'Front' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['Front'])

        if 'Left' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['Left'])

        if 'Right' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['Right'])

        return list_of_actors

    def build_master_scenario(self, route):
        # We have to find the target.
        # we also have to convert the route to the expected format


        master_scenario_configuration = ScenarioConfiguration()
        master_scenario_configuration.target = route[-1]
        master_scenario_configuration.route = None

        return Master(self.world, self.ego_vehicle, master_scenario_configuration)


    def build_scenario_instances(self, scenario_definition_vec, town):
        """
            Based on the parsed route and posible scenarios, build all the scenario classes.
        :param scenario_definition_vec: the dictionary defining the scenarios
        :param town:
        :return:
        """
        scenario_instance_vec = []

        for definition in scenario_definition_vec:
            # Get the class possibilities for this scenario number
            possibility_vec = number_class_translation[definition['name']]
            # for now I dont know how to disambiguate this part.
            ScenarioClass = possibility_vec[0]
            # Create the other actors that are going to appear
            list_of_actor_conf_instances = self.get_actors_instances(definition['Antagonist_Vehicles'])
            # Create an actor configuration for the ego-vehicle trigger position
            egoactor_trigger_position = convert_json_to_actor(definition['trigger_position'])

            scenario_configuration = ScenarioConfiguration()
            scenario_configuration.other_actors = list_of_actor_conf_instances
            scenario_configuration.town = town
            scenario_configuration.ego_vehicle = egoactor_trigger_position

            scenario_instance = ScenarioClass(self.world, self.ego_vehicle, scenario_configuration)
            scenario_instance_vec.append(scenario_instance)

        return scenario_definition_vec

    def route_is_running(self):
        """
            The master scenario tests if the route is still running.
        """
        if self.master_scenario is None:
            raise ValueError('You should not run a rout without a master scenario')

        return self.master_scenario.scenario_tree.status == py_trees.common.Status.RUNNING


    def summary_route_performance(self):
        """
          This function is intended to be called from outside and provide
          statistics about the scenario (human-readable, for the CARLA challenge.)
        """
        PENALTY_COLLISION_STATIC = 10
        PENALTY_COLLISION_VEHICLE = 10
        PENALTY_COLLISION_PEDESTRIAN = 30
        PENALTY_TRAFFIC_LIGHT = 10
        PENALTY_WRONG_WAY = 5
        target_reached = False
        failure = False
        result = "SUCCESS"
        final_score = 0.0
        score_penalty = 0.0
        score_route = 0.0
        return_message = ""

        if isinstance(self.master_scenario.scenario.test_criteria, py_trees.composites.Parallel):
            if self.master_scenario.scenario.test_criteria.status == py_trees.common.Status.FAILURE:
                failure = True
                result = "FAILURE"
            if self.master_scenario.scenario.timeout_node.timeout and not failure:
                result = "TIMEOUT"

            list_traffic_events = []
            for node in self.master_scenario.scenario.test_criteria.children:
                if node.list_traffic_events:
                    list_traffic_events.extend(node.list_traffic_events)

            list_collisions = []
            list_red_lights = []
            list_wrong_way = []
            list_route_dev = []
            # analyze all traffic events
            for event in list_traffic_events:
                if event.get_type() == TrafficEventType.COLLISION_STATIC:
                    score_penalty += PENALTY_COLLISION_STATIC
                    msg = event.get_message()
                    if msg:
                        list_collisions.append(event.get_message())

                elif event.get_type() == TrafficEventType.COLLISION_VEHICLE:
                    score_penalty += PENALTY_COLLISION_VEHICLE
                    msg = event.get_message()
                    if msg:
                        list_collisions.append(event.get_message())

                elif event.get_type() == TrafficEventType.COLLISION_PEDESTRIAN:
                    score_penalty += PENALTY_COLLISION_PEDESTRIAN
                    msg = event.get_message()
                    if msg:
                        list_collisions.append(event.get_message())

                elif event.get_type() == TrafficEventType.TRAFFIC_LIGHT_INFRACTION:
                    score_penalty += PENALTY_TRAFFIC_LIGHT
                    msg = event.get_message()
                    if msg:
                        list_red_lights.append(event.get_message())

                elif event.get_type() == TrafficEventType.WRONG_WAY_INFRACTION:
                    score_penalty += PENALTY_WRONG_WAY
                    msg = event.get_message()
                    if msg:
                        list_wrong_way.append(event.get_message())

                elif event.get_type() == TrafficEventType.ROUTE_DEVIATION:
                    msg = event.get_message()
                    if msg:
                        list_route_dev.append(event.get_message())

                elif event.get_type() == TrafficEventType.ROUTE_COMPLETED:
                    score_route = 100.0
                    target_reached = True
                elif event.get_type() == TrafficEventType.ROUTE_COMPLETION:
                    if not target_reached:
                        score_route = event.get_dict()['route_completed']

            final_score = max(score_route - score_penalty, 0)

            return_message += "\n=================================="
            return_message += "\n==[{}] [Score = {:.2f} : (route_score={}, infractions=-{})]".format(result,
                                                                                                     final_score,
                                                                                                     score_route,
                                                                                                     score_penalty)
            if list_collisions:
                return_message += "\n===== Collisions:"
                for item in list_collisions:
                    return_message += "\n========== {}".format(item)

            if list_red_lights:
                return_message += "\n===== Red lights:"
                for item in list_red_lights:
                    return_message += "\n========== {}".format(item)

            if list_wrong_way:
                return_message += "\n===== Wrong way:"
                for item in list_wrong_way:
                    return_message += "\n========== {}".format(item)

            if list_route_dev:
                return_message += "\n===== Route deviation:"
                for item in list_route_dev:
                    return_message += "\n========== {}".format(item)

            return_message += "\n=================================="

        return result, final_score, return_message

    def final_challenge_statistics(self):
        # Grab the statistics from all the routes.
        pass

    def run(self, args):
        """
        Run all routes according to provided commandline args
        """
        """
            The world needs to be running
        """
        self._carla_server.reset(args.host, args.port)
        self._carla_server.wait_until_ready()


        # retrieve worlds annotations
        world_annotations = parser.parse_annotations_file(args.annotations_file)
        # retrieve routes
        route_descriptions_list = parser.parse_routes_file(args.routes_file)

        # find and filter potential scenarios for each of the evaluated routes
        potential_scenarios_list = [parser.scan_route_for_scenarios(route_description, world_annotations)
                                    for route_description in route_descriptions_list]

        # For each of the routes and corresponding possible scenarios to be evaluated.
        for route_description, potential_scenarios in zip(route_descriptions_list, potential_scenarios_list):

            list_of_scenarios_definitions = self.scenario_sampling(potential_scenarios)


            # setup world and client assuming that the CARLA server is up and running
            client = carla.Client(args.host, int(args.port))
            client.set_timeout(self.client_timeout)

            self.world = client.load_world(route_description.town)
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            self.world.apply_settings(settings)
            # Set the actor pool so the scenarios can prepare themselves when needed
            CarlaActorPool.set_world(self.world)

            # prepare route's trajectory
            gps_route, world_coordinates_route = parse_trajectory(route_description['trajectory'])

            # build the master scenario based on the route and the target.
            self.master_scenario = self.build_master_scenario(world_coordinates_route)
            list_scenarios = [self.master_scenario]
            # build the instance based on the parsed definitions.
            list_scenarios += self.build_scenario_instances(list_of_scenarios_definitions)


            client.tick()
            # create agent
            self.agent_instance = getattr(self.module_agent, self.module_agent.__name__)(args.config)
            self.agent_instance.set_global_plan(gps_route)
            # main loop
            while self.route_is_running():
                # update all scenarios
                for scenario in list_scenarios:
                    scenario.scenario.scenario_tree.tick_once()

                # ego vehicle acts
                ego_action = self.agent_instance()
                self.ego_vehicle.apply_control(ego_action)

                # time continues
                client.tick()


            self.agent_instance.destroy()

            # statistics recording
            result, final_score, return_message = self.summary_route_performance()

        # stop CARLA server
        self._carla_server.stop()


            # cleanup

        self.cleanup(ego=True)
        # statistics report
        # TODO:

        # final measurements from the challenge
        self.final_challenge_statistics()


if __name__ == '__main__':

    DESCRIPTION = ("CARLA AD Challenge evaluation: evaluate your Agent in CARLA scenarios\n")

    PARSER = argparse.ArgumentParser(description=DESCRIPTION, formatter_class=RawTextHelpFormatter)
    PARSER.add_argument('--host', default='localhost',
                        help='IP of the host server (default: localhost)')
    PARSER.add_argument('--port', default='2000', help='TCP port to listen to (default: 2000)')
    PARSER.add_argument("--use-docker", type=bool, help="Use docker to run CARLA?", default=False)
    PARSER.add_argument('--docker-version', type=str, help='Docker version to use for CARLA server', default="0.9.3")
    PARSER.add_argument("-a", "--agent", type=str, help="Path to Agent's py file to evaluate")
    PARSER.add_argument("--config", type=str, help="Path to Agent's configuration file", default="")
    PARSER.add_argument('--route-visible', action="store_true", help='Run with a visible route')
    PARSER.add_argument('--debug', action="store_true", help='Run with debug output')
    PARSER.add_argument('--file', action="store_true", help='Write results into a txt file')

    ARGUMENTS = PARSER.parse_args()

    CARLA_ROOT = os.environ.get('CARLA_ROOT')
    ROOT_SCENARIO_RUNNER = os.environ.get('ROOT_SCENARIO_RUNNER')

    if not CARLA_ROOT:
        print("Error. CARLA_ROOT not found. Please run setup_environment.sh first.")
        sys.exit(0)

    if not ROOT_SCENARIO_RUNNER:
        print("Error. ROOT_SCENARIO_RUNNER not found. Please run setup_environment.sh first.")
        sys.exit(0)

    ARGUMENTS.carla_root = CARLA_ROOT

    try:
        challenge_evaluator = ChallengeEvaluator(ARGUMENTS)
        challenge_evaluator.run(ARGUMENTS)
    finally:
        del challenge_evaluator
