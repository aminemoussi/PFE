import carla, time, math, random

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
bp = world.get_blueprint_library()

vehicle = imu = None
rows = []
try:
    sp = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp.filter("vehicle.*")[0], sp)

    imu_bp = bp.find("sensor.other.imu")
    imu_bp.set_attribute("sensor_tick", "0.1")  # 10 Hz, noise default 0
    imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

    def on_imu(m):
        w = vehicle.get_angular_velocity()  # deg/s (actor API)
        rows.append(
            (
                m.accelerometer.x,
                m.accelerometer.y,
                m.accelerometer.z,
                m.gyroscope.z,
                w.z,
            )
        )  # gyro in rad/s

    imu.listen(on_imu)

    vehicle.apply_control(
        carla.VehicleControl(throttle=0.5, steer=0.3)
    )  # no autopilot/TM
    time.sleep(8)
finally:
    if imu:
        imu.stop()
        imu.destroy()
    if vehicle:
        vehicle.destroy()

print(f"{len(rows)} samples")
print(" acc_x   acc_y   acc_z   gyro(rad/s)  gt_w(deg/s)  gyro*180/pi")
for r in rows[-6:]:
    print(
        f"{r[0]:+6.2f} {r[1]:+6.2f} {r[2]:+6.2f}   {r[3]:+8.3f}    {r[4]:+8.2f}   {r[3] * 180 / math.pi:+8.2f}"
    )
