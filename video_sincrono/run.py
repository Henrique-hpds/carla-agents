from datetime import datetime
import carla
import random
import os
import time

from modules.Camera import Camera
from modules.IMU import IMU
from modules.PositionModule import PositionModule

agentes = []
pedestres = []
tick_time = 0.05
sim_time = 30

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.load_world('Town01')

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = tick_time
world.apply_settings(settings)

traffic_manager = client.get_trafficmanager(8001)
traffic_manager.set_synchronous_mode(True)
traffic_manager.set_global_distance_to_leading_vehicle(2.0)
traffic_manager.set_hybrid_physics_mode(True)  # Para lidar com muitos veículos
# traffic_manager.set_random_device_seed(0)  # Para resultados consistentes

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

blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.*')
pedestrian_bp = blueprint_library.filter('*walker.pedestrian*')
pedestrian_controller_bp = blueprint_library.find('controller.ai.walker')

spawn_points = world.get_map().get_spawn_points()

for i in range(3):
    spawn_point = random.choice(spawn_points)
    actual_vehicle = world.try_spawn_actor(random.choice(vehicle_bp), spawn_point)
    spawn_points.remove(spawn_point)
    if actual_vehicle is not None:
        agentes.append(actual_vehicle)
        actual_vehicle.set_autopilot(True, traffic_manager.get_port())


vehicle = world.try_spawn_actor(vehicle_bp.find("vehicle.bmw.grandtourer"), spawn_points[0])
vehicle.set_autopilot(True, traffic_manager.get_port())
agentes.append(vehicle)

spawn_points.remove(spawn_points[0])

timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
experiment_dir = f'data/exp_{timestamp}'
experiment_dir_pedestres = f'data/exp_{timestamp}/pedestres'

if not os.path.exists(experiment_dir):
    os.makedirs(experiment_dir)

controllers = []

lista_imus_pedestres = []

for i in range(4):
    spawn_point_pedestrian = world.get_random_location_from_navigation()

    if spawn_point_pedestrian is None:
        continue

    spawn_transform = carla.Transform(spawn_point_pedestrian)
    actual_pedestrian = world.try_spawn_actor(random.choice(pedestrian_bp), spawn_transform)

    if actual_pedestrian is not None:
        pedestres.append(actual_pedestrian)
        
        imu_ped = IMU("pedestrian", world, 0, 0, 1.7, 0, tick_time, attached_ob=actual_pedestrian)
        lista_imus_pedestres.append(imu_ped)
        
        controller = world.spawn_actor(pedestrian_controller_bp, carla.Transform(), attach_to=actual_pedestrian)
        controller.start()
        controller.go_to_location(world.get_random_location_from_navigation())
        controller.set_max_speed(1.0)
        controllers.append(controller)
        
        

world.tick()

# camera = Camera("vehicle", world, -5 , 0, 2, 0, vehicle)
imu = IMU("vehicle", world, 0, 0, 0, 0, tick_time, vehicle)

# camera.start(experiment_dir)
imu.start(experiment_dir)

i = 0
for imu_ped in lista_imus_pedestres:
    imu_ped.start(experiment_dir_pedestres + f"/{i}")
    i += 1

count = 0
start_time = time.time()
actual_time = time.time()

while actual_time - start_time < sim_time:
    world.tick()
    time.sleep(0.05)
    actual_time = time.time()
    print(actual_time - start_time)    
    
i = 0
for imu_ped in lista_imus_pedestres:
    imu_ped.stop()
    imu_ped.save_data(experiment_dir_pedestres + f"/{i}")
    imu_ped.plot_data(experiment_dir_pedestres + f"/{i}")    
    i += 1

# camera.stop()
imu.stop()
imu.save_data(experiment_dir)
imu.plot_data(experiment_dir)

for controller in controllers:
    controller.stop()
for controller in controllers:
    controller.destroy()
for vehicle in agentes:
    vehicle.destroy()