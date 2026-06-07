import carla, time

c = carla.Client("localhost", 2000)
c.set_timeout(120.0)
t = time.time()
w = c.get_world()
print(f"ready: {w.get_map().name} in {time.time() - t:.0f}s")
