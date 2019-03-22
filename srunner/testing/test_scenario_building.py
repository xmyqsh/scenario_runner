import unittest
import os
from pprint import pprint
#import torch

#from configs import g_conf, set_type_of_process, merge_with_yaml
#from input import RandomSequenceSampler, RandomSampler
#from utils.general import create_log_folder, create_exp_path


import srunner.challenge.utils.route_configuration_parser as parser
from srunner.challenge.challenge_evaluator_routes import ChallengeEvaluator

from srunner.scenariomanager.carla_data_provider import CarlaActorPool

from srunner.challenge.utils.route_manipulation import interpolate_trajectory
import carla




class Arguments():

    def __init__(self):
        self.agent = None
        self.use_docker = False
        self.carla_root = '../Carla94b/'
        self.host = '127.0.0.1'
        self.port = 2000
        self.split = 'dev_track_1'

class TestScenarioBuilder(unittest.TestCase):

    def __init__(self, name='runTest'):
        unittest.TestCase.__init__(self, name)
        self.root_route_file_position = 'srunner/testing/test_files'



    def test_build_scenarios(self):

        args = Arguments()
        client = carla.Client(args.host, int(args.port))
        client.set_timeout(25.0)
        challenge = ChallengeEvaluator(args)

        filename = os.path.join(self.root_route_file_position, 'all_towns_traffic_scenarios.json')
        world_annotations = parser.parse_annotations_file(filename)
        # retrieve routes
        # Which type of file is expected ????

        filename = os.path.join(self.root_route_file_position, 'routes_training.xml')
        list_route_descriptions = parser.parse_routes_file(filename)

        # For each of the routes to be evaluated.
        for route_description in list_route_descriptions:
            #route_description = list_route_descriptions[0]
            challenge.world = client.load_world(route_description['town_name'])
            # Set the actor pool so the scenarios can prepare themselves when needed
            CarlaActorPool.set_world(challenge.world)
            # find and filter potential scenarios
            # Returns the iterpolation in a different format

            challenge.world.wait_for_tick()
            gps_route, route_description['trajectory'] = interpolate_trajectory(challenge.world,
                                                                                route_description['trajectory'])
            #print (" FULL ROUTE ")
            #for wp in route_description['trajectory']:

            #    print (wp[0].transform.location)

            potential_scenarios_definitions = parser.scan_route_for_scenarios(route_description, world_annotations)

            # prepare route's trajectory
            challenge.prepare_ego_car(route_description['trajectory'][0][0].transform)


            # build the master scenario based on the route and the target.
            master_scenario = challenge.build_master_scenario(route_description['trajectory'], route_description['town_name'])
            list_scenarios = [master_scenario]
            print (" Built the master scenario ")
            # build the instance based on the parsed definitions.
            print (potential_scenarios_definitions)
            list_scenarios += challenge.build_scenario_instances(potential_scenarios_definitions, route_description['town_name'])

            print (" Scenarios present ", list_scenarios)

            challenge.cleanup(ego=True)
