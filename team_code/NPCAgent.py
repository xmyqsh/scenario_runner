import scipy.misc

import carla
from agents.navigation.basic_agent import *

from srunner.challenge.autoagents.autonomous_agent import AutonomousAgent, Track

class NPCAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        self.track = Track.ALL_SENSORS_HDMAP_WAYPOINTS
        self._agent = BasicAgent(actor)


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



        return control
