import carla
import random
import time
from queue import Queue
from queue import Empty 

def queue_sensor_data(data, queue):
    # Store data in a queue
    queue.put(data)


def main():
    actor_list = []
    
    random.seed(0);
    
    # Connect to the client and retrieve the world object.
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0) # Slow HDD Load means long timeout
    
    world = client.load_world('Town04')
    client.reload_world()
    
    # Enables synchronous mode to ensure deterministic output.
    settings = world.get_settings()
    settings.synchronous_mode = True 
    settings.fixed_delta_seconds = 0.1
    world.apply_settings(settings)
    
    # Connect to the traffic manager port to make sure it is deterministic.
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_random_device_seed(0)
    traffic_manager.set_synchronous_mode(True)

    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')

    spawn_points = world.get_map().get_spawn_points()
     
    ego_vehicle_bp = world.get_blueprint_library().find('vehicle.yamaha.yzf')
    ego_vehicle = world.try_spawn_actor(ego_vehicle_bp, random.choice(spawn_points))
    actor_list.append(ego_vehicle)
    print('created ego vehicle %s' % ego_vehicle.type_id)
    ego_vehicle.set_autopilot(True)

    # Create a transform to place the camera on top of the vehicle.
    camera_init_trans = carla.Transform(carla.Location(x=1.5, z=2.4))

    # We create the camera through a blueprint that defines its properties.
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    
    # Modify the attributes of the blueprint to set image resolution and field of view.
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '110')
    
    # Remove Motion Blur
    camera_bp.set_attribute('motion_blur_intensity', '0.0')
    
    # Set the time in seconds between sensor captures.
    # camera_bp.set_attribute('sensor_tick', '0.03')
    # Removed to allow rate proportional to fixed delta
    
    # Spawn the camera and attach it to our ego vehicle.
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
    actor_list.append(camera)
    print('created sensor %s' % camera.type_id)
        
    # Listen for the camera frames and queue up the images.
    image_queue = Queue()
    camera.listen(lambda image: queue_sensor_data(image, image_queue))
    
    # Make "perfect" location sensor for the ground truth.
    GNSS_init_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    GNSS_bp = world.get_blueprint_library().find('sensor.other.gnss')
    GNSS_bp.set_attribute('noise_alt_bias', '0.0')
    position_sensor = world.spawn_actor(GNSS_bp, GNSS_init_transform, attach_to =ego_vehicle)
    actor_list.append(position_sensor)
    print('created sensor %s' % position_sensor.type_id)
    
    f1 = open("./OpticalCameraOutput/10Hz/ground_truth.txt", "w")
    position_sensor.listen(lambda GNSSMeasurement: f1.write(("X, " + str(GNSSMeasurement.transform.location.x) + ", Y, " + str(GNSSMeasurement.transform.location.y) + ", Z, " + str(GNSSMeasurement.transform.location.z) + "\n")))
    
    for x in range(300):
        world.tick()
        
        try:
            # Save the data once it is put into the queue.
            image_data = image_queue.get(True, 1.0)
            image_data.save_to_disk('OpticalCameraOutput/10Hz/%06d.png' % image_data.frame)
        except Empty:
            print("[Warning] Some sensor data has been missed")
        continue   
    f1.close()


if __name__ == '__main__':

    main()