import carla

c = carla.Client("localhost", 2000)
c.set_timeout(10.0)
print("server:", c.get_server_version())
w = c.get_world()
s = w.get_settings()
print("was sync:", s.synchronous_mode, "| map:", w.get_map().name)
s.synchronous_mode = False
s.fixed_delta_seconds = None
w.apply_settings(s)
print("reset to async")
