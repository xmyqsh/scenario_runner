import unittest
import os
from pprint import pprint
#import torch

#from configs import g_conf, set_type_of_process, merge_with_yaml
#from input import RandomSequenceSampler, RandomSampler
#from utils.general import create_log_folder, create_exp_path


import srunner.challenge.utils.route_configuration_parser as parser

"""
The idea of this test is to check if sampling is able to sample random sequencial images
inside a batch

"""


class TestRouteGenerator(unittest.TestCase):

    def __init__(self, name='runTest'):
        unittest.TestCase.__init__(self, name)
        self.root_route_file_position = 'srunner/testing/test_files'

    def test_scan_route_for_scenarios(self):
        # Read the files going to be used for this test
        filename = os.path.join(self.root_route_file_position, 'Town03_scenarios_AntagonistVehicleWorldSpace.json')
        annotations = parser.parse_annotations_file(filename)
        filename = os.path.join(self.root_route_file_position, 'routes_town06_test.xml')
        routes_town06 = parser.parse_routes_file(filename)
        filename = os.path.join(self.root_route_file_position, 'routes_town03_test.xml')
        routes_town03 = parser.parse_routes_file(filename)

        # The routes for this file is in a different town, no posible scenario is expected
        for route in routes_town06:
            posible_scenarios = parser.scan_route_for_scenarios(route, annotations)
            self.assertEqual(len(posible_scenarios), 0)

        # For the positions into the first route route at least four scenarios can be placed
        route = routes_town03[0]
        posible_scenarios = parser.scan_route_for_scenarios(route, annotations)
        print("##################")
        print ('For The route ', route['id'], " We found these scenarios")
        print ("##################")
        pprint(posible_scenarios)
        # A routes Just stays on one part of scenario 1 .
        self.assertEqual(len(posible_scenarios), 4)


    def test_parse_trajectory(self):

        # It should be consistent after parsing
        pass

