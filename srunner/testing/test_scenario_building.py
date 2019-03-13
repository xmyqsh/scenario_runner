import unittest
import os
from pprint import pprint
#import torch

#from configs import g_conf, set_type_of_process, merge_with_yaml
#from input import RandomSequenceSampler, RandomSampler
#from utils.general import create_log_folder, create_exp_path


import srunner.challenge.utils.route_configuration_parser as parser
from srunner.challenge.challenge_evaluator_routes import ChallengeEvaluator
from srunner.scenarios.idle import Idle

"""
The idea of this test is to check if sampling is able to sample random sequencial images
inside a batch

"""


class TestScenarioBuilder(unittest.TestCase):

    def __init__(self, name='runTest'):
        unittest.TestCase.__init__(self, name)
        self.root_route_file_position = 'srunner/testing/test_files'

    def test_build_scenarios(self):

        challenge = ChallengeEvaluator()

        filename = os.path.join(self.root_route_file_position, 'Town03_scenarios_AntagonistVehicleWorldSpace.json')
        world_annotations = parser.parse_annotations_file(filename)
        # retrieve routes
        # Which type of file is expected ????

        filename = os.path.join(self.root_route_file_position, 'routes_town03_test.xml')
        list_route_descriptions = parser.parse_routes_file(filename)

        # For each of the routes to be evaluated.
        for route_description in list_route_descriptions:
            # find and filter potential scenarios
            potential_scenarios_definitions = parser.scan_route_for_scenarios(route_description, world_annotations)
            list_of_scenarios_definitions = potential_scenarios_definitions

            # prepare route's trajectory
            gps_route, world_coordinates_route = parser.parse_trajectory(route_description.trajectory)

            # pre-instantiate all scenarios for this route
            list_scenarios = [Idle()]
            # build the instance based on the parsed definitions.
            list_scenarios += challenge.build_scenario_instances(list_of_scenarios_definitions)
