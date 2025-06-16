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
SIM_TIME = 300#s

timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
experiment_dir = f'data/assincrono/exp_{timestamp}'
# experiment_dir = f'data/assincrono/300s'

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
    settings.synchronous_mode = False
    world.apply_settings(settings)
    
    traffic_manager = client.get_trafficmanager(8001)
    traffic_manager.set_synchronous_mode(False)
    traffic_manager.set_global_distance_to_leading_vehicle(2.0)
    traffic_manager.set_hybrid_physics_mode(False)  # Para lidar com muitos veículos

    # Deixa todos os semáforos verdes
    for actor in world.get_actors().filter('traffic.traffic_light*'):
        actor.set_state(carla.TrafficLightState.Green)
        actor.set_green_time(9999)  # Mantém verde por muito tempo

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.*')
    spawn_points = world.get_map().get_spawn_points()
    
    vehicle = world.spawn_actor(vehicle_bp.find("vehicle.bmw.grandtourer"), spawn_points[0])
    # vehicle.set_autopilot(True, traffic_manager.get_port())
    
    if vehicle is not None:
        agentes.append(vehicle)
    else:
        raise Exception("Failed to spawn vehicle")
    
    for _ in range(N_VEHICLES - 1): # Veículos NPC
        vehicle_npc = world.try_spawn_actor(random.choice(vehicle_bp), random.choice(spawn_points))
        if vehicle_npc is not None:
            agentes.append(vehicle_npc)
            vehicle_npc.set_autopilot(True, traffic_manager.get_port())

    agent = BasicAgent(vehicle)
    agent_destination = spawn_points[-1].location
    agent.set_destination(agent_destination)

    imu_bp = blueprint_library.find('sensor.other.imu')
    # imu_bp.set_attribute('sensor_tick', '0.01') # não faz sentido no modo assíncrono
    imu_transform = carla.Transform(carla.Location(x=0.0, z=0.0))
    imu_sensor = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
    imu_listener(imu_sensor, imu_data)    
    
    elapsed_time = 0.0
    start_time = time.time()
    times = []
    xs = []
    ys = []
    
    
    # while elapsed_time < SIM_TIME:
    while True:
        elapsed_time = time.time() - start_time
        
        if agent.done():
            print("Agent reached the destination.")
            break
        
        control = agent.run_step()
        vehicle.apply_control(control) 
        
        transform = vehicle.get_transform()
        location = transform.location
        times.append(elapsed_time)
        xs.append(location.x)
        ys.append(location.y)
        print(elapsed_time, location)

    imu_sensor.stop()

    plt.figure(figsize=(10, 5))
    plt.plot(times, xs, label='Posição X')
    plt.plot(times, ys, label='Posição Y')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Posição (m)')
    plt.title('Posição do veículo ao longo do tempo (modo assíncrono)')
    plt.legend()
    plt.grid()
    plt.savefig(f"{experiment_dir}/vehicle_position.png")
    plot_and_save_imu("vehicle", list(imu_data.values()), experiment_dir)
    

    fig, axs = plt.subplots(2, 1, figsize=(15, 15))

    # Use 'times' ao invés de 'time'
    axs[0].plot(times, xs, label='Position X', color='blue')
    axs[0].plot(times, ys, label='Position Y', color='orange')
    axs[0].set_ylabel('Position Coordinates (m)')
    axs[0].set_xlabel('Tempo (s)')
    axs[0].legend()
    axs[0].grid()

    # Gráfico de trajetória GPS com gradiente de cores
    sc = axs[1].scatter(xs, ys, c=times, cmap='viridis', s=10, label='Trajectory')
    axs[1].scatter(xs[0], ys[0], label='Start', color='green', s=100)
    axs[1].scatter(xs[-1], ys[-1], label='End', color='orange', s=100)
    axs[1].set_xlabel('Position X (m)')
    axs[1].set_ylabel('Position Y (m)')
    axs[1].legend()
    axs[1].grid()
    fig.colorbar(sc, ax=axs[1], label='Time (s)')

    fig.suptitle('Position Data over Time and Trajectory')
    plt.tight_layout()
    plt.savefig(f'{experiment_dir}/Position.png')
    
except Exception as e:
    print(f"Erro: {e}")
finally:
    # Destrua o sensor primeiro, se existir
    if 'imu_sensor' in locals():
        try:
            imu_sensor.destroy()
        except Exception as e:
            print(f"Erro ao destruir imu_sensor: {e}")

    # Depois destrua os veículos
    for agent in agentes:
        try:
            if agent.is_alive:
                agent.destroy()
        except Exception as e:
            print(f"Erro ao destruir agente: {e}")
    print("All actors destroyed.")