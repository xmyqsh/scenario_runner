import scipy.misc

import carla
from agents.navigation.basic_agent import *

from srunner.scenariomanager.carla_data_provider import CarlaActorPool

from srunner.challenge.autoagents.autonomous_agent import AutonomousAgent, Track

class NPCAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        self.track = Track.ALL_SENSORS_HDMAP_WAYPOINTS
        self.route_assigned = False

        hero_actor = None
        list_actors = list(CarlaActorPool.get_actors())
        for _, actor in list_actors:
            if actor.attributes['role_name'] == 'hero':
                hero_actor = actor
                break

        if hero_actor:
            self._agent = BasicAgent(hero_actor)


    def sensors(self):
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}


        """
        sensors = [{'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll':0.0, 'pitch':0.0, 'yaw': 0.0,
                    'width': 800, 'height': 600, 'fov':100, 'id': 'Center'},
                   {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
                    'yaw': -45.0, 'width': 800, 'height': 600, 'fov': 100, 'id': 'Left'},
                   {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 45.0,
                    'width': 800, 'height': 600, 'fov': 100, 'id': 'Right'},
                   {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
                    'yaw': -45.0, 'id': 'LIDAR'},
                   {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'},
                   {'type': 'sensor.can_bus', 'reading_frequency': 25, 'id': 'can_bus'},
                   {'type': 'sensor.hd_map', 'reading_frequency': 1, 'id': 'hdmap'},
                  ]

        return sensors

    def run_step(self, input_data):
        if not self.route_assigned:
            if self._global_plan:
                import pdb; pdb.set_trace()
                self._agent._local_planner.set_global_plan(self._global_plan)
                self.route_assigned = True

            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.0
            control.hand_brake = False

        else:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.0
            control.hand_brake = False

        return control
