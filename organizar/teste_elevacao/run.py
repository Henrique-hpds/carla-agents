import random
import carla
import os
import time

# custom modules
from modules.Camera import Camera

from datetime import datetime

xodr_path = "unicamp.xodr"
with open(xodr_path, 'r') as od_file:
    xodr_data = od_file.read()

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

client.generate_opendrive_world(xodr_data)
world = client.get_world()

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

traffic_manager = client.get_trafficmanager(8001)
traffic_manager.set_synchronous_mode(True)
traffic_manager.set_global_distance_to_leading_vehicle(2.0)
# traffic_manager.set_hybrid_physics_mode(True)  # Para lidar com muitos veículos

blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.*')
spawn_points = world.get_map().get_spawn_points()
if not spawn_points:
    print("Nenhum ponto de spawn disponível no mapa!")
    exit(1)
    
for _ in range(1):  # Adiciona 10/10 veículos NPC
    vehicle_2 = world.try_spawn_actor(random.choice(vehicle_bp), random.choice(spawn_points))
    if vehicle_2 is not None:
        vehicle_2.set_autopilot(True, traffic_manager.get_port())
        
vehicle = world.spawn_actor(random.choice(vehicle_bp), spawn_points[0])
vehicle.set_autopilot(True, traffic_manager.get_port())

timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
experiment_dir = f'data/exp_{timestamp}'

if not os.path.exists(experiment_dir):
    os.makedirs(experiment_dir)
    
camera = Camera("camera", world, 0, 0, 10, -45, vehicle)
camera.start(experiment_dir)

start_time = time.time()
actual_time = time.time()

while actual_time - start_time < 50:

    world.tick()
    actual_time = time.time()
    print(actual_time - start_time)
    
camera.stop()
camera.destroy()
vehicle.destroy()
for actor in world.get_actors():
    if actor.id != vehicle.id:
        actor.destroy()