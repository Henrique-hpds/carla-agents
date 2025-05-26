import matplotlib.pyplot as plt
import pandas as pd
import random
import carla
import time
import os

from agents.navigation.basic_agent import BasicAgent

# custom modules
from modules.IMU import IMU

from datetime import datetime

N_VEHICLES = 2
SIM_TIME = 30#s

timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
experiment_dir = f'data/sincrono/exp_{timestamp}'

if not os.path.exists(experiment_dir):
    os.makedirs(experiment_dir)


def imu_listener(imu_actor, data_storage):
    def callback(imu_data):
        data_storage[imu_data.timestamp] = {
            'timestamp': imu_data.timestamp,
            'accel_x': imu_data.accelerometer.x,
            'accel_y': imu_data.accelerometer.y,
            'accel_z': imu_data.accelerometer.z - 9.81,
            'gyro_x': imu_data.gyroscope.x,
            'gyro_y': imu_data.gyroscope.y,
            'gyro_z': imu_data.gyroscope.z
        }
    imu_actor.listen(callback)
    
def plot_and_save_imu(agent_name, data, save_dir):
    # Filtra entradas válidas
    clean_data = [d for d in data if isinstance(d, dict) and len(d) == 7]

    # Ignora os 5 primeiros dados
    clean_data = clean_data[5:]

    if not clean_data:
        print(f"[Aviso] Nenhum dado válido restante para {agent_name}.")
        return

    df = pd.DataFrame(clean_data)
    df.to_csv(f"{save_dir}/imu.csv", index=False)

    # Plot
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    plt.plot(df['timestamp'], df['accel_x'], label='accel_x')
    plt.plot(df['timestamp'], df['accel_y'], label='accel_y')
    plt.plot(df['timestamp'], df['accel_z'], label='accel_z')
    plt.title(f'{agent_name} - Accelerometer')
    plt.xlabel("Tempo (s)")
    plt.ylabel("Aceleração (m/s²)")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(df['timestamp'], df['gyro_x'], label='gyro_x')
    plt.plot(df['timestamp'], df['gyro_y'], label='gyro_y')
    plt.plot(df['timestamp'], df['gyro_z'], label='gyro_z')
    plt.title(f'{agent_name} - Gyroscope')
    plt.xlabel("Tempo (s)")
    plt.ylabel("Gyro (rad/s)")
    plt.legend()

    plt.tight_layout()
    plt.savefig(f"{save_dir}/imu_plot.png")
    plt.close()

try:
    agentes = []
    imu_data = {}
    gnss_data = {}

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
    traffic_manager.set_hybrid_physics_mode(False)  # Para lidar com muitos veículos

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.*')
    spawn_points = world.get_map().get_spawn_points()
    
    vehicle = world.spawn_actor(vehicle_bp.find("vehicle.bmw.grandtourer"), spawn_points[0])
    vehicle.set_autopilot(True, traffic_manager.get_port())

    if vehicle is not None:
        agentes.append(vehicle)
    else:
        raise Exception("Failed to spawn vehicle")
    
    for _ in range(N_VEHICLES - 1): # Veículos NPC
        vehicle_npc = world.try_spawn_actor(random.choice(vehicle_bp), random.choice(spawn_points))
        if vehicle_npc is not None:
            agentes.append(vehicle_npc)
            vehicle_npc.set_autopilot(True, traffic_manager.get_port())

    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_transform = carla.Transform(carla.Location(x=0.0, z=0.0))
    imu_sensor = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
    imu_listener(imu_sensor, imu_data)    
    
    tempo = 0.0
    times = []
    xs = []
    ys = []
    
    
    while tempo < SIM_TIME:
        world.tick()  # avança o mundo em modo síncrono
        time.sleep(0.05)
        transform = vehicle.get_transform()
        location = transform.location
        times.append(tempo)
        xs.append(location.x)
        ys.append(location.y)
        print(tempo, location)
        tempo += 0.05
        
    imu_sensor.stop()

    plt.figure(figsize=(10, 5))
    plt.plot(times, xs, label='Posição X')
    plt.plot(times, ys, label='Posição Y')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Posição (m)')
    plt.title('Posição do veículo ao longo do tempo (modo síncrono)')
    plt.legend()
    plt.grid()
    plt.savefig(f"{experiment_dir}/vehicle_position.png")
    plot_and_save_imu("vehicle", list(imu_data.values()), experiment_dir)
    
finally:
    for agent in agentes:
        if agent.is_alive:
            agent.destroy()
    print("All actors destroyed.")
    
    imu_sensor.destroy()
    