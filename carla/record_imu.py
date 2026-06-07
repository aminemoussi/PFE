import carla
import random
import time
from queue import Queue
from queue import Empty
import math
import os

CAMERA_ON = False  # CHANGED: skip image dump (you have Will's camera/VO)
OUTPUT_DIR = "/home/fablab/Documents/PFE_Moussi/output/30HzSlow2"  # CHANGED: Linux path (Will's was F:/...)


def queue_sensor_data(data, queue):
    queue.put(data)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    actor_list = []
    random.seed(0)

    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)

    world = client.load_world("Town03")
    client.reload_world()

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.00825
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(8000)
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
    ego_vehicle.set_autopilot(True)
    traffic_manager.vehicle_percentage_speed_difference(ego_vehicle, 50)

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
        image_queue = Queue()
        camera.listen(lambda image: queue_sensor_data(image, image_queue))

    # Perfect (noise-free) GNSS = ground-truth position
    GNSS_init_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    GNSS_bp = world.get_blueprint_library().find("sensor.other.gnss")
    GNSS_bp.set_attribute("noise_alt_bias", "0.0")
    position_sensor = world.spawn_actor(
        GNSS_bp, GNSS_init_transform, attach_to=ego_vehicle
    )
    actor_list.append(position_sensor)
    print("created sensor %s" % position_sensor.type_id)

    f1 = open(OUTPUT_DIR + "/ground_truth.txt", "w")
    position_sensor.listen(
        lambda g: f1.write(
            "X, "
            + str(g.transform.location.x)
            + ", Y, "
            + str(g.transform.location.y)
            + ", Z, "
            + str(g.transform.location.z)
            + "\n"
        )
    )

    f2 = open(OUTPUT_DIR + "/vehicle_sensors.txt", "w")

    GNSS_bp.set_attribute("sensor_tick", "0.0333")
    noisy_gnss = world.spawn_actor(GNSS_bp, GNSS_init_transform, attach_to=ego_vehicle)
    actor_list.append(noisy_gnss)
    print("created sensor %s" % noisy_gnss.type_id)

    f3 = open(OUTPUT_DIR + "/noisy_gnss.txt", "w")
    noisy_gnss.listen(
        lambda g: f3.write(
            "X, "
            + str(g.transform.location.x)
            + ", Y, "
            + str(g.transform.location.y)
            + ", Z, "
            + str(g.transform.location.z)
            + "\n"
        )
    )

    # ===== NEW: IMU sensor (noise OFF), logging the clean body-frame accel + yaw rate =====
    imu_bp = world.get_blueprint_library().find("sensor.other.imu")
    imu_bp.set_attribute("sensor_tick", "0.1")  # 10 Hz, to match the PINN
    imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego_vehicle)
    actor_list.append(imu_sensor)
    print("created sensor %s" % imu_sensor.type_id)

    f4 = open(OUTPUT_DIR + "/imu.txt", "w")

    def log_imu(m):
        gt_w = ego_vehicle.get_angular_velocity()  # deg/s, sanity cross-check only
        f4.write(
            "t, "
            + str(m.timestamp)
            + ", ax, "
            + str(m.accelerometer.x)
            + ", ay, "
            + str(m.accelerometer.y)
            + ", wz, "
            + str(m.gyroscope.z)  # rad/s, clean
            + ", gt_wz_deg, "
            + str(gt_w.z)
            + "\n"
        )

    imu_sensor.listen(log_imu)
    # ======================================================================================

    for x in range(4242):
        world.tick()
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

        if CAMERA_ON and not (x % 4):
            try:
                image_data = image_queue.get(True, 0.1)
                image_data.save_to_disk(OUTPUT_DIR + "/%06d.png" % image_data.frame)
            except Empty:
                print("[Warning] Some sensor data has been missed")
            continue

    f1.close()
    f2.close()
    f3.close()
    f4.close()  # NEW


if __name__ == "__main__":
    main()
