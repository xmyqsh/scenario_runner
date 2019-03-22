
import carla
from srunner.scenarios.object_crash_intersection import VehicleTurningRight
from srunner.scenarios.master import Master
from srunner.scenariomanager.carla_data_provider import CarlaActorPool, CarlaDataProvider
from srunner.scenarios.config_parser import ScenarioConfiguration, ActorConfigurationData
import time
client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)
world = client.load_world('Town01')
world.wait_for_tick()

CarlaActorPool.set_world(world)
CarlaDataProvider.set_world(world)

ego_transform = carla.Transform(location = carla.Location(x=338.703, y=227.451, z=0.0),
                rotation = carla.Rotation(roll=0.0,pitch=0.0,yaw=-90.0))

ego_trigger_transform = carla.Transform(location = carla.Location(x=88.23, y=297.43, z=1.0),
                rotation = carla.Rotation(roll=0.0, pitch=0.0, yaw=90.0))



ego_vehicle = CarlaActorPool.setup_actor('vehicle.lincoln.mkz2017', ego_transform, True)
world.wait_for_tick()

time.sleep(0.2)

# Build a master first

master_scenario_configuration = ScenarioConfiguration()
master_scenario_configuration.target = ''  # Take the last point and add as target.
master_scenario_configuration.route = ''
master_scenario_configuration.town = 'Town01'
master_scenario_configuration.ego_vehicle = ActorConfigurationData('vehicle.lincoln.mkz2017', ego_transform)

Master(world, ego_vehicle, master_scenario_configuration)

scenario_configuration = ScenarioConfiguration()
scenario_configuration.other_actors = None
scenario_configuration.town = 'Town01'
scenario_configuration.ego_vehicle = ActorConfigurationData('vehicle.lincoln.mkz2017', ego_trigger_transform)


scenario_instance = VehicleTurningRight(world, ego_vehicle, scenario_configuration)




#Scenario4  built with  {'left': [{'z': '1.0', 'x': '117.32', 'y': '258.27', 'pitch': '0.0', 'yaw': '180.000046'}],
#                        'right': [{'z': '1.0', 'x': '121.45', 'y': '332.55', 'pitch': '0.0', 'yaw': '0.000031'}]}

#    ego  {'z': '1.0', 'x': '88.23', 'y': '297.43', 'pitch': '0', 'yaw': '90'}



# wp gent  Transform(Location(x=338.688, y=187.451, z=0), Rotation(pitch=360, yaw=269.979, roll=0))
