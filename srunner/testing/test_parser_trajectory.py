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

import srunner.challenge.utils.route_configuration_parser as parser
from srunner.challenge.challenge_evaluator_routes import ChallengeEvaluator, parse_trajectory

from srunner.scenariomanager.carla_data_provider import CarlaActorPool

import carla

class Arguments():

    def __init__(self):
        self.agent = None
        self.use_docker = False
        self.carla_root = '../Carla94b/'
        self.host = '127.0.0.1'
        self.port = 2000


class TestParseTrajector(unittest.TestCase):

    def __init__(self, name='runTest'):
        unittest.TestCase.__init__(self, name)
        self.root_route_file_position = 'srunner/testing/test_files'



    def test_parse_trajectory(self):

        args = Arguments()

        client = carla.Client(args.host, int(args.port))
        client.set_timeout(25.0)
        # retrieve routes
        # Which type of file is expected ????

        filename = os.path.join(self.root_route_file_position, 'routes_town01.xml')
        list_route_descriptions = parser.parse_routes_file(filename)

        # For each of the routes to be evaluated.
        for route_description in list_route_descriptions:


            world = client.load_world(route_description['town_name'])
            settings = world.get_settings()
            settings.synchronous_mode = True
            world.apply_settings(settings)
            # Set the actor pool so the scenarios can prepare themselves when needed
            # find and filter potential scenarios
            # prepare route's trajectory
            gps_route, world_coordinates_route = parse_trajectory(world, route_description['trajectory'])


