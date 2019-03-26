import unittest
import os

import srunner.challenge.utils.route_configuration_parser as parser
from srunner.challenge.challenge_evaluator_routes import ChallengeEvaluator, convert_json_to_transform

from srunner.scenariomanager.carla_data_provider import CarlaActorPool

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.challenge.utils.route_manipulation import interpolate_trajectory
import carla




def convert_waypoint_float(waypoint):

    waypoint['x'] = float(waypoint['x'])
    waypoint['y'] = float(waypoint['y'])
    waypoint['z'] = float(waypoint['z'])
    waypoint['yaw'] = float(waypoint['yaw'])

class Arguments():

    def __init__(self):
        self.agent = None
        self.use_docker = False
        self.host = '127.0.0.1'
        self.port = 2000
        self.split = 'dev_track_1'

class TestSpawn(unittest.TestCase):

    def __init__(self, name='runTest'):
        unittest.TestCase.__init__(self, name)
        self.root_route_file_position = 'srunner/challenge/'


    def test_possible_spawns(self):

        args = Arguments()
        client = carla.Client(args.host, int(args.port))
        client.set_timeout(25.0)
        challenge = ChallengeEvaluator(args)

        filename = os.path.join(self.root_route_file_position, 'all_towns_traffic_scenarios.json')
        world_annotations = parser.parse_annotations_file(filename)
        # retrieve routes
        # Which type of file is expected ????


        # For each of the routes to be evaluated.
        for town_name in world_annotations.keys():
            challenge.world = client.load_world(town_name)
            CarlaActorPool.set_world(challenge.world)

            CarlaDataProvider.set_world(challenge.world)

            scenarios = world_annotations[town_name]
            for scenario in scenarios:  # For each existent scenario
                for event in scenario["available_event_configurations"]:
                    waypoint = event['transform']

                    convert_waypoint_float(waypoint)

                    challenge.prepare_ego_car(convert_json_to_transform(waypoint))

                    if event['other_actors'] is not None:
                        if 'left' in event['other_actors']:
                            for other_waypoint in event['other_actors']['left']:
                                challenge.prepare_ego_car(convert_json_to_transform(other_waypoint))
                        if 'right' in event['other_actors']:
                            for other_waypoint in event['other_actors']['right']:
                                challenge.prepare_ego_car(convert_json_to_transform(other_waypoint))
                        if 'front' in event['other_actors']:
                            for other_waypoint in event['other_actors']['front']:
                                challenge.prepare_ego_car(convert_json_to_transform(other_waypoint))



            challenge.cleanup(ego=True)


