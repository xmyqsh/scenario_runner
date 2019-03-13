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

#import carla
import srunner.challenge.utils.route_configuration_parser as parser
from srunner.challenge.envs.server_manager import ServerManagerBinary, ServerManagerDocker
#from srunner.scenarios.challenge_basic import *
#from srunner.scenarios.config_parser import *
#from srunner.scenariomanager.scenario_manager import ScenarioManager

from srunner.scenarios.control_loss import ControlLoss
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
from srunner.scenarios.object_crash_vehicle import StationaryObjectCrossing, DynamicObjectCrossing
from srunner.scenarios.object_crash_intersection import VehicleTurningRight, VehicleTurningLeft
from srunner.scenarios.opposite_vehicle_taking_priority import OppositeVehicleRunningRedLight
from srunner.scenarios.signalized_junction_left_turn import SignalizedJunctionLeftTurn
from srunner.scenarios.no_signal_junction_crossing import NoSignalJunctionCrossing
# TODO maybe the parsing and the building are actually different.

# The configuration parser

from srunner.scenarios.config_parser import ActorConfiguration

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

        # first we instantiate the Agent
        module_name = os.path.basename(args.agent).split('.')[0]
        module_spec = importlib.util.spec_from_file_location(module_name, args.agent)
        self.module_agent = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(self.module_agent)

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


        def get_actors_from_list(list_of_actor_def):
            """
                Receives a list of actor definitions and creates an actual list of ActorConfigurationObjects
            :param list_of_actor_def:
            :return:
            """
            def convert_json_to_actor(actor_dict):
                node = ET.Element('waypoint')
                node.set('x', actor_dict['x'])
                node.set('y', actor_dict['y'])
                node.set('z', actor_dict['z'])

                return ActorConfiguration(node)

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


    def build_scenario_instances(self, scenario_definition_vec, town):
        scenario_instance_vec = []

        for definition in scenario_definition_vec:
            # Get the class possibilities for this scenario number

            posibility_vec = number_class_translation[definition['name']]
            # for now I dont know how to disambiguate this part.
            ScenarioClass = posibility_vec[0]

            # Create the actors...
            list_of_actor_conf_instances = self.get_actors_instances(definition['Antagonist_Vehicles'])

            scenario_instance = ScenarioClass(self.world, self.ego_vehicle, list_of_actor_conf_instances, town)

            scenario_instance_vec.append(scenario_instance)


    def run(self, args):
        """
        Run all routes according to provided commandline args
        """
        # retrieve worlds annotations
        world_annotations = parser.parse_annotations_file(args.annotations_file)
        # retrieve routes
        # Which type of file is expected ????
        list_route_descriptions = parser.parse_routes_file(args.routes_file)

        # For each of the routes to be evaluated.
        for route_description in list_route_descriptions:
            # find and filter potential scenarios
            potential_scenarios_definitions = parser.scan_route_for_scenarios(route_description, world_annotations)
            list_of_scenarios_definitions = self.scenario_sampling(potential_scenarios_definitions)

            # prepare route's trajectory
            gps_route, world_coordinates_route = parser.parse_trajectory(route_description.trajectory)

            # pre-instantiate all scenarios for this route
            list_scenarios = [Idle()]
            # build the instance based on the parsed definitions.
            list_scenarios += self.build_scenario_instances(list_of_scenarios_definitions)

            # setup world and client assuming that the CARLA server is up and running
            client = carla.Client(args.host, int(args.port))
            client.set_timeout(self.client_timeout)

            self.world = client.load_world(route_description.town)
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            self.world.apply_settings(settings)
            client.tick()

            # create agent
            self.agent_instance = getattr(self.module_agent, self.module_agent.__name__)(args.config)
            self.agent_instance.set_global_plan(gps_route)
            # main loop
            while True:
                # update all scenarios
                for scenario in list_scenarios:
                    scenario.scenario.scenario_tree.tick_once()

                # ego vehicle acts
                ego_action = self.agent_instance()
                self.ego_vehicle.apply_control(ego_action)

                # time continues
                client.tick()


            self.cleanup(ego=True)
        self.agent_instance.destroy()

        self.final_summary(args)

        # stop CARLA server
        self._carla_server.stop()

        # statistics recording
            # TODO:

            # cleanup
            # TODO:

        # statistics report
        # TODO:

        # final cleanup
        # TODO:


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
