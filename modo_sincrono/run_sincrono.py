from datetime import datetime
import carla
import random
import os
import time

from modules.IMU import IMU

timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
experiment_dir = f'data/exp_{timestamp}'
os.makedirs(experiment_dir, exist_ok=True)

SIMULATION_TIME = 60
NUM_VEHICLES = 3

agentes = []
imus = []

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.load_world('Town01')

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

traffic_manager = client.get_trafficmanager(8001)
traffic_manager.set_synchronous_mode(True)
traffic_manager.set_global_distance_to_leading_vehicle(2.0)
traffic_manager.set_hybrid_physics_mode(True)

blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.*')

spawn_points = world.get_map().get_spawn_points()

for i in range(NUM_VEHICLES):
    spawn_point = random.choice(spawn_points)
    actual_vehicle = world.try_spawn_actor(random.choice(vehicle_bp), spawn_point)
    spawn_points.remove(spawn_point)
    
    imu = IMU(f"IMU_carro_{i}", world, 0, 0, 0, 0, 0.05, attached_ob=actual_vehicle)
    imus.append(imu)
    
    if actual_vehicle is not None:
        agentes.append(actual_vehicle)
        actual_vehicle.set_autopilot(True, traffic_manager.get_port())


vehicle = world.try_spawn_actor(vehicle_bp.find("vehicle.bmw.grandtourer"), spawn_points[0])
vehicle.set_autopilot(True, traffic_manager.get_port())
agentes.append(vehicle)

timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
experiment_dir = f'data/exp_{timestamp}'

os.makedirs(experiment_dir, exist_ok=True)

for imu in imus:
    imu.start(experiment_dir)
    
try:
    frame = 0
    while frame < int(SIMULATION_TIME / 0.05):
        world.tick()
        frame += 1
        
finally:
    for imu in imus:
        imu.stop()
        imu.save_data(experiment_dir)
        imu.plot_data(experiment_dir)
        imu.destroy()

    for actor in agentes:
        actor.destroy()
