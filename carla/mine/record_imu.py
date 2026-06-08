import carla
import random
import time
from queue import Queue
from queue import Empty
import math
import os

CAMERA_ON = False  # skip image dump (you have Will's camera/VO)
OUTPUT_DIR = "/home/fablab/Documents/PFE_Moussi/output/30HzSlow2"


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    actor_list = []
    random.seed(0)

    client = carla.Client("localhost", 2000)
    client.set_timeout(300.0)

    world = client.get_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.00825
    settings.no_rendering_mode = True
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(8001)
    traffic_manager.set_random_device_seed(1)
    traffic_manager.set_synchronous_mode(True)

    spawn_points = world.get_map().get_spawn_points()

    ego_vehicle_bp = world.get_blueprint_library().find("vehicle.mercedes.coupe_2020")
    ego_vehicle = world.try_spawn_actor(ego_vehicle_bp, random.choice(spawn_points))

    transform = ego_vehicle.get_transform()
    transform.location.x = 55.46192169189453
    transform.location.y = -4.3009796142578125
    transform.location.z = 1
    transform.rotation.yaw = 180
    ego_vehicle.set_transform(transform)

    spectator = world.get_spectator()
    spectator.set_transform(transform)

    actor_list.append(ego_vehicle)
    print("created ego vehicle %s" % ego_vehicle.type_id)
    ego_vehicle.set_autopilot(True, traffic_manager.get_port())
    traffic_manager.vehicle_percentage_speed_difference(ego_vehicle, 50)

    # ONE shared queue. Every sensor callback ONLY drops raw numbers in -> instant, never blocks.
    sensor_q = Queue()

    if CAMERA_ON:
        camera_init_trans = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", "1920")
        camera_bp.set_attribute("image_size_y", "1080")
        camera_bp.set_attribute("fov", "110")
        camera_bp.set_attribute("motion_blur_intensity", "0.0")
        camera_bp.set_attribute("sensor_tick", str(settings.fixed_delta_seconds * 4))
        camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
        actor_list.append(camera)
        print("created sensor %s" % camera.type_id)
        camera.listen(lambda image: sensor_q.put(("cam", image)))

    # Perfect (noise-free) GNSS = ground-truth position
    GNSS_init_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    GNSS_bp = world.get_blueprint_library().find("sensor.other.gnss")
    GNSS_bp.set_attribute("noise_alt_bias", "0.0")
    position_sensor = world.spawn_actor(
        GNSS_bp, GNSS_init_transform, attach_to=ego_vehicle
    )
    actor_list.append(position_sensor)
    print("created sensor %s" % position_sensor.type_id)
    position_sensor.listen(
        lambda g: sensor_q.put(
            (
                "gt",
                (
                    g.transform.location.x,
                    g.transform.location.y,
                    g.transform.location.z,
                ),
            )
        )
    )

    GNSS_bp.set_attribute("sensor_tick", "0.0333")
    noisy_gnss = world.spawn_actor(GNSS_bp, GNSS_init_transform, attach_to=ego_vehicle)
    actor_list.append(noisy_gnss)
    print("created sensor %s" % noisy_gnss.type_id)
    noisy_gnss.listen(
        lambda g: sensor_q.put(
            (
                "noisy",
                (
                    g.transform.location.x,
                    g.transform.location.y,
                    g.transform.location.z,
                ),
            )
        )
    )

    # IMU (noise OFF): callback ONLY enqueues clean numbers -- no file write, no RPC (that was the slow part)
    imu_bp = world.get_blueprint_library().find("sensor.other.imu")
    imu_bp.set_attribute("sensor_tick", "0.1")  # 10 Hz, to match the PINN
    imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego_vehicle)
    actor_list.append(imu_sensor)
    print("created sensor %s" % imu_sensor.type_id)
    imu_sensor.listen(
        lambda m: sensor_q.put(
            ("imu", (m.timestamp, m.accelerometer.x, m.accelerometer.y, m.gyroscope.z))
        )
    )

    f1 = open(OUTPUT_DIR + "/ground_truth.txt", "w")
    f2 = open(OUTPUT_DIR + "/vehicle_sensors.txt", "w")
    f3 = open(OUTPUT_DIR + "/noisy_gnss.txt", "w")
    f4 = open(OUTPUT_DIR + "/imu.txt", "w")

    def drain():
        # write everything sitting in the queue, then return. Keeps memory flat.
        while True:
            try:
                tag, data = sensor_q.get_nowait()
            except Empty:
                return
            if tag == "gt":
                f1.write("X, %s, Y, %s, Z, %s\n" % data)
            elif tag == "noisy":
                f3.write("X, %s, Y, %s, Z, %s\n" % data)
            elif tag == "imu":
                f4.write("t, %s, ax, %s, ay, %s, wz, %s\n" % data)
            elif tag == "cam":
                data.save_to_disk(OUTPUT_DIR + "/%06d.png" % data.frame)

    try:
        for x in range(4242):
            world.tick()

            # telemetry -- RPCs are fine HERE (main thread), unlike inside a sensor callback
            velocity = ego_vehicle.get_velocity()
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            vehicle_transform = ego_vehicle.get_transform()
            f2.write(
                "SteeringAngle, "
                + str(
                    ego_vehicle.get_wheel_steer_angle(
                        carla.VehicleWheelLocation.Front_Wheel
                    )
                )
                + ", Speed, "
                + str(speed)
                + ", Rotation, "
                + str(vehicle_transform.rotation.yaw)
                + "\n"
            )

            drain()  # consume this step's sensor data -> bounded memory
            time.sleep(0.001)  # yield so the sensor thread can deliver; cheap insurance
    finally:
        try:
            time.sleep(0.1)
            drain()  # flush any last stragglers
        except Exception:
            pass
        for f in (f1, f2, f3, f4):
            try:
                f.close()
            except Exception:
                pass
        for s in actor_list:
            try:
                s.destroy()
            except Exception:
                pass
        try:
            s2 = world.get_settings()
            s2.synchronous_mode = False
            s2.fixed_delta_seconds = None
            world.apply_settings(s2)
        except Exception:
            pass


if __name__ == "__main__":
    main()
