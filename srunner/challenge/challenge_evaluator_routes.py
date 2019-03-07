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

import carla


class ChallengeRouteEvaluator(object):

    """
    CARLA Autonomous Driving evaluation code to assess an AutonomousAgent performance
    """
    def __init__(self, args):
        # Tunable parameters
        self.client_timeout = 15.0  # in seconds
        self.wait_for_world = 10.0  # in seconds


        # CARLA world and scenario handlers
        self.world = None
        self.ego_vehicle = None
        self._sensors_list = []

        # first we instantiate the Agent
        if args.agent is not None:
            module_name = os.path.basename(args.agent).split('.')[0]
            module_spec = importlib.util.spec_from_file_location(module_name, args.agent)
            self.module_agent = importlib.util.module_from_spec(module_spec)
            module_spec.loader.exec_module(self.module_agent)

        # retrieve worlds annotations
        self.world_annotations = self.parse_annotations_file(args.annotations_file)
        # retrieve routes
        self.list_route_descriptions = self.parse_routes_file(args.routes_file)


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
        Cleanup and delete actors and CARLA world
        """
        self.cleanup(True)
        if self.world is not None:
            del self.world

    def prepare_ego_vehicle(self, agent, trajectory):
        # setup ego vehicle
        pass

        # setup sensors
        pass

        # setup trajectory
        agent.set_global_plan(trajectory)

    def run(self, args):
        """
        Run all routes according to provided commandline args
        """

        for route_description in self.list_route_descriptions:
            for repetition in range(route_description.repetitions):
                # find and filter potential scenarios
                potential_scenarios = self.scan_route_for_scenarios(route_description, self.world_annotations)
                list_scenario_definitions = self.scenario_sampling(potential_scenarios)

                # prepare route's trajectory
                gps_route, world_coordinates_route = self.parse_trajectory(route_description.trajectory)

                # pre-instantiate all scenarios for this route
                self.list_scenarios.append(Idle())
                for scenario_definition in list_scenario_definitions:
                    Scenario = self.get_scenario_class_or_fail()
                    scenario_inst = Scenario(scenario_definition)
                    self.list_scenarios.append(scenario_inst)

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
                self.prepare_ego_vehicle(self.agent_instance, gps_route)

                # main loop
                while True:
                    # update all scenarios
                    for scenario in self.list_scenarios:
                        scenario.scenario_tree.tick_once()

                    # ego vehicle acts
                    ego_action = self.agent_instance()
                    self.ego_vehicle.apply_control(ego_action)

                    # time continues
                    client.tick()

                # statistics recording
                # TODO:

                # cleanup
                # TODO:

        # final statistics report
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
    PARSER.add_argument(
        '--routes', help='Name of the route to be executed. Point to the route_xml_file to be executed.')

    PARSER.add_argument(
        '--scenarios', help='Name of the scenario annotation file to be mixed with the route.')


    ARGUMENTS = PARSER.parse_args()

    CARLA_ROOT = os.environ.get('CARLA_ROOT')
    ROOT_SCENARIO_RUNNER = os.environ.get('ROOT_SCENARIO_RUNNER')

    if not CARLA_ROOT:
        print("Error. CARLA_ROOT not found. Please run setup_environment.sh first.")
        sys.exit(0)

    if not ROOT_SCENARIO_RUNNER:
        print("Error. ROOT_SCENARIO_RUNNER not found. Please run setup_environment.sh first.")
        sys.exit(0)

    if ARGUMENTS.routes is None:
        print("Please specify a path to a route file  '--routes path-to-route'\n\n")
        PARSER.print_help(sys.stdout)
        sys.exit(0)

    if ARGUMENTS.scenarios is None:
        print("Please specify a path to a scenario specification file  '--scenarios path-to-file'\n\n")
        PARSER.print_help(sys.stdout)
        sys.exit(0)

    ARGUMENTS.carla_root = CARLA_ROOT

    try:
        challenge_evaluator = ChallengeEvaluator(ARGUMENTS)
        challenge_evaluator.run(ARGUMENTS)
    finally:
        del challenge_evaluator
