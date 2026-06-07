import carla, time, math

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
bp = world.get_blueprint_library()

vehicle = world.spawn_actor(
    bp.filter("vehicle.*")[0], world.get_map().get_spawn_points()[0]
)
vehicle.set_autopilot(True)  # let it drive around

imu_bp = bp.find("sensor.other.imu")
imu_bp.set_attribute("sensor_tick", "0.1")  # 10 Hz, noise left at default (0)
imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)

rows = []


def on_imu(m):
    w = vehicle.get_angular_velocity()  # deg/s  (actor API)
    rows.append(
        (
            m.accelerometer.x,
            m.accelerometer.y,
            m.accelerometer.z,
            m.gyroscope.z,  # rad/s  (sensor API)
            w.z,
        )
    )


imu.listen(on_imu)

try:
    time.sleep(8)  # drive for 8 s
finally:
    imu.stop()
    imu.destroy()
    vehicle.destroy()

print(f"{len(rows)} samples")
print(" acc_x   acc_y   acc_z   gyro(rad/s)  gt_w(deg/s)  gyro*180/pi")
for r in rows[-6:]:
    print(
        f"{r[0]:+6.2f} {r[1]:+6.2f} {r[2]:+6.2f}   {r[3]:+8.3f}    {r[4]:+8.2f}   {r[3] * 180 / math.pi:+8.2f}"
    )
