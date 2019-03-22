
import carla
from srunner.scenarios.object_crash_intersection import VehicleTurningRight, VehicleTurningLeft
from srunner.scenariomanager.carla_data_provider import CarlaActorPool

client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)
world = client.get_world()
world.wait_for_tick()

CarlaActorPool.set_world(world)


ego_transform = carla.Transform(location = carla.Location(x=88.23, y=297.43, z=1.0),
                rotation = carla.Rotation(roll=0.0,pitch=0.0,yaw=90.0))




ego_vehicle = CarlaActorPool.setup_actor('vehicle.lincoln.mkz2017', ego_transform, True)

scenario_instance = ScenarioClass(world, ego_vehicle, Sc)
scenario_instance_vec.append(scenario_instance)

VehicleTurningRight()

scenario_configuration = ScenarioConfiguration()
scenario_configuration.other_actors = list_of_actor_conf_instances
scenario_configuration.town = town_name
scenario_configuration.ego_vehicle = egoactor_trigger_position


Scenario4  built with  {'left': [{'z': '1.0', 'x': '117.32', 'y': '258.27', 'pitch': '0.0', 'yaw': '180.000046'}],
                        'right': [{'z': '1.0', 'x': '121.45', 'y': '332.55', 'pitch': '0.0', 'yaw': '0.000031'}]}

    ego  {'z': '1.0', 'x': '88.23', 'y': '297.43', 'pitch': '0', 'yaw': '90'}



 wp gent  Transform(Location(x=338.688, y=187.451, z=0), Rotation(pitch=360, yaw=269.979, roll=0))
