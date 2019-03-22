
import carla
from srunner.scenarios.object_crash_intersection import VehicleTurningRight
from srunner.scenariomanager.carla_data_provider import CarlaActorPool
from srunner.scenarios.config_parser import ScenarioConfiguration, ActorConfigurationData

client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)
world = client.load_world('Town01')
world.wait_for_tick()

CarlaActorPool.set_world(world)

ego_transform = carla.Transform(location = carla.Location(x=338.703, y=227.451, z=-0.00709011),
                rotation = carla.Rotation(roll=0.0,pitch=0.0,yaw=-90.0))

ego_trigger_transform = carla.Transform(location = carla.Location(x=88.23, y=297.43, z=1.0),
                rotation = carla.Rotation(roll=0.0, pitch=0.0, yaw=90.0))

ego_vehicle = CarlaActorPool.setup_actor('vehicle.lincoln.mkz2017', ego_transform, True)


scenario_configuration = ScenarioConfiguration()
scenario_configuration.other_actors = None
scenario_configuration.town = 'Town01'
scenario_configuration.ego_vehicle = ActorConfigurationData('vehicle.lincoln.mkz2017', ego_trigger_transform)



scenario_instance = VehicleTurningRight(world, ego_vehicle, scenario_configuration)




#Scenario4  built with  {'left': [{'z': '1.0', 'x': '117.32', 'y': '258.27', 'pitch': '0.0', 'yaw': '180.000046'}],
#                        'right': [{'z': '1.0', 'x': '121.45', 'y': '332.55', 'pitch': '0.0', 'yaw': '0.000031'}]}

#    ego  {'z': '1.0', 'x': '88.23', 'y': '297.43', 'pitch': '0', 'yaw': '90'}



# wp gent  Transform(Location(x=338.688, y=187.451, z=0), Rotation(pitch=360, yaw=269.979, roll=0))
