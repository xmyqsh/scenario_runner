import carla
client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)
world = client.load_world('Town01')
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)

world.tick()

world = client.load_world('Town02')
