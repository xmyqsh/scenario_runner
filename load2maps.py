"""
import carla
client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)
world = client.load_world('Town01')
settings = world.get_settings()
settings.synchronous_mode = True
world.apply_settings(settings)

world.tick()

world = client.load_world('Town02')
"""

import carla
import time

TIME_ACTION = 1.0 / 100.0
maps = ['Town01', 'Town02', 'Town03', 'Town04', 'Town05']

# preamble
client = carla.Client('localhost', int(2000))
client.set_timeout(25.0)
world = client.get_world()
world.wait_for_tick()

for m in maps:
    settings = world.get_settings()
    settings.synchronous_mode = False
    world.apply_settings(settings)

    world = client.load_world(m)
    world.wait_for_tick()
    settings = world.get_settings()
    settings.synchronous_mode = True
    world.apply_settings(settings)

    # do something interesting
    for i in range(20):
        time.sleep(TIME_ACTION)
        world.tick()
        print("[{}] Tick {}".format(m, i))

# epilogue
settings = world.get_settings()
settings.synchronous_mode = False
world.apply_settings(settings)

