from datetime import datetime
import matplotlib.pyplot as plt
import time
import carla
import random
import numpy as np
import cv2 as cv
import csv
import os

from plot import *

def inertial_data(sim_time = 300, town='Town01', vehicle='vehicle.ford.mustang', save_data=True, plot_data=True, save_images=True):

    sensors = []

    client = carla.Client('localhost',2000)
    world = client.load_world(town)

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05

    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)


    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        sun_altitude_angle=10.0,
        sun_azimuth_angle = 70.0,
        precipitation_deposits = 0.0,
        wind_intensity = 0.0,
        fog_density = 0.0,
        wetness = 0.0, 
    )
    world.set_weather(weather)

    bp_lib = world.get_blueprint_library() 
    spawn_points = world.get_map().get_spawn_points()

    vehicle_bp = bp_lib.find('vehicle.audi.etron')
    ego_vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

    for _ in range(1):  # Adiciona 10 veículos NPC
        npc_bp = random.choice(bp_lib.filter('vehicle.*'))
        spawn_point = random.choice(spawn_points)
        world.try_spawn_actor(npc_bp, spawn_point)


    spectator = world.get_spectator()
    transform = carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),ego_vehicle.get_transform().rotation)
    spectator.set_transform(transform)

    ego_vehicle.set_autopilot(True)

    # Add RGB camera
    camera_bp = bp_lib.find('sensor.camera.rgb') 
    camera_init_trans = carla.Transform(carla.Location(x =-0.1,z=1.7)) 
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

    # Storing image width and height values
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()

    # Add depth camera
    depth_camera_bp = bp_lib.find('sensor.camera.depth') 
    depth_camera = world.spawn_actor(depth_camera_bp, camera_init_trans, attach_to=ego_vehicle)

    # Add navigation sensor
    gnss_bp = bp_lib.find('sensor.other.gnss')
    gnss_sensor = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=ego_vehicle)

    # Add IMU sensor
    imu_bp = bp_lib.find('sensor.other.imu')
    imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego_vehicle)

    # Callback functions for all the sensors used here.
    def rgb_callback(image, data_dict):
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)) #Reshaping with alpha channel
        img[:,:,3] = 255 #Setting the alpha to 255 
        data_dict['rgb_image'] = img

    def depth_callback(image, data_dict):
        image.convert(carla.ColorConverter.LogarithmicDepth)
        data_dict['depth_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    def gnss_callback(data, data_dict):
        location = ego_vehicle.get_location()
        data_dict['gps'] = [location.x, location.y, location.z] 
        data_dict['gnss'] = [data.latitude, data.longitude, data.altitude]

    def imu_callback(data, data_dict):
        data_dict['imu'] = {
            'gyro': data.gyroscope,
            'accel': data.accelerometer,
            'compass': data.compass
        }

    # Update the sensor_data dictionary to include the depth_image, gnss and imu keys and default values
    sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4)),
                   'depth_image': np.zeros((image_h, image_w, 4)),
                   'gps': [0,0,0],
                   'gnss': [0,0,0],
                   'imu': {
                            'gyro': carla.Vector3D(),
                            'accel': carla.Vector3D(),
                            'compass':0
                        }}

    timestamp = datetime.now().strftime('%d-%m-%Y_%Hh-%Mm-%Ss')
    experiment_dir = f'data/exp_{timestamp}'
    filename = experiment_dir + '/data.csv'

    def rgb_image_creator(image,date_time):
        cv.imwrite(f'{experiment_dir}/images/rgb/rgb_{date_time}.png',image)
    
    def depth_image_creator(image,date_time):
        cv.imwrite(f'{experiment_dir}/images/depth/depth_{date_time}.png',image)

    if save_data:
        os.makedirs(experiment_dir)
        os.makedirs(experiment_dir + '/images/rgb')
        os.makedirs(experiment_dir + '/images/depth')
        os.makedirs(experiment_dir + '/plots')
        os.makedirs(experiment_dir + '/videos')
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'latitude', 'longitude', 'altitude', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'compass', 'gps_x', 'gps_y', 'gps_z'])

    sensors.append(camera)
    sensors.append(depth_camera)
    sensors.append(gnss_sensor)
    sensors.append(imu_sensor)

    # Listen to the sensor feed
    camera.listen(lambda image: rgb_callback(image, sensor_data))
    depth_camera.listen(lambda image: depth_callback(image, sensor_data))
    gnss_sensor.listen(lambda event: gnss_callback(event, sensor_data))
    imu_sensor.listen(lambda event: imu_callback(event, sensor_data))

    # Define the duration for the loop in seconds (e.g., 60 seconds)
    count = 0
    start_time = time.time()
    actual_time = time.time()

    while actual_time - start_time < sim_time:

        actual_time = time.time()

        # Skip the first two readings
        if count < 2:
            count += 1
            world.tick()
            continue
        
        current_time = datetime.now()
        date_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
        
        print(actual_time - start_time)

        lat = sensor_data['gnss'][0]
        long = sensor_data['gnss'][1]
        alt = sensor_data['gnss'][2]
        gyro = sensor_data['imu']['gyro']        
        accel = sensor_data['imu']['accel'] - carla.Vector3D(x=0,y=0,z=9.81)
        compass = sensor_data['imu']['compass']
        gps_x = sensor_data['gps'][0]
        gps_y = sensor_data['gps'][1]
        gps_z = sensor_data['gps'][2]

        data_list = [actual_time,
                     lat,long,alt,
                     accel.x,accel.y,accel.z,
                     gyro.x,gyro.y,gyro.z, compass,
                     gps_x,gps_y,gps_z]
        try:
            with open(filename,'a') as file:
                writer = csv.writer(file)
                writer.writerow(data_list)
        
        except Exception as e:
            print(f"Error writing row to CSV: {e}")
        
        if save_images:
            rgb_image_creator(sensor_data['rgb_image'],date_time)
            depth_image_creator(sensor_data['depth_image'],date_time)

        world.tick()
    
    if plot_data:
        plot_gnss(filename, experiment_dir)
        plot_accel(filename, experiment_dir)
        plot_gyro(filename, experiment_dir)
        plot_gps(filename, experiment_dir)
        plot_compass(filename, experiment_dir)
    # if save_images:
    #     img2vid(experiment_dir + '/images/rgb', experiment_dir + '/videos/rgb.avi')
    #     img2vid(experiment_dir + '/images/depth', experiment_dir + '/videos/depth.avi')


    for sensor in sensors:
        sensor.destroy()

if __name__ == '__main__':
    inertial_data(sim_time=100, town="Town03", save_images=True)