
import carla

client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)

# For each of the routes to be evaluated.
world = client.load_world('Town01')
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)

world = client.load_world('Town02')
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)