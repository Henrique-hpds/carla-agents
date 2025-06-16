import carla
import random
import time
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

SAVE_DIR = "saida_agentes"
SIMULATION_TIME = 20  # segundos
AGENTS_PER_TYPE = 2

imu_data_storage = defaultdict(list)
gnss_data_buffer = {}

def create_imu_sensor(blueprint_library):
    imu_bp = blueprint_library.find('sensor.other.imu')
    imu_bp.set_attribute('sensor_tick', '0.05')
    return imu_bp

def create_gnss_sensor(blueprint_library):
    gnss_bp = blueprint_library.find('sensor.other.gnss')
    gnss_bp.set_attribute('sensor_tick', '0.05')
    return gnss_bp

def imu_listener(agent_name, data_storage):
    def callback(imu_data):
        for item in reversed(data_storage[agent_name]):
            if item['accel_x'] is None:
                item.update({
                    'accel_x': imu_data.accelerometer.x,
                    'accel_y': imu_data.accelerometer.y,
                    'accel_z': imu_data.accelerometer.z,
                    'gyro_x': imu_data.gyroscope.x,
                    'gyro_y': imu_data.gyroscope.y,
                    'gyro_z': imu_data.gyroscope.z
                })
                break
    return callback

def gnss_listener(agent_name):
    def callback(data):
        gnss_data_buffer[agent_name] = {
            'lat': data.latitude,
            'lon': data.longitude,
            'alt': data.altitude
        }
    return callback

def plot_and_save(agent_name, data):
    os.makedirs(os.path.join(SAVE_DIR, agent_name), exist_ok=True)

    clean_data = [d for d in data if isinstance(d, dict) and len(d) >= 16]
    clean_data = clean_data[5:]  # Ignora os 5 primeiros

    if not clean_data:
        print(f"[Aviso] Nenhum dado válido para {agent_name}.")
        return

    df = pd.DataFrame(clean_data)
    df.to_csv(os.path.join(SAVE_DIR, agent_name, "imu.csv"), index=False)

    # IMU Plot
    plt.figure(figsize=(12, 6))
    plt.subplot(2, 1, 1)
    plt.plot(df['timestamp'], df['accel_x'], label='accel_x')
    plt.plot(df['timestamp'], df['accel_y'], label='accel_y')
    plt.plot(df['timestamp'], df['accel_z'], label='accel_z')
    plt.title(f'{agent_name} - Acelerômetro')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(df['timestamp'], df['gyro_x'], label='gyro_x')
    plt.plot(df['timestamp'], df['gyro_y'], label='gyro_y')
    plt.plot(df['timestamp'], df['gyro_z'], label='gyro_z')
    plt.title(f'{agent_name} - Giroscópio')
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR, agent_name, "imu_plot.png"))
    plt.close()

    # GNSS Path
    plt.figure(figsize=(6, 6))
    plt.plot(df['longitude'], df['latitude'], marker='o', linestyle='-')
    plt.title(f'{agent_name} - GNSS Trajetória')
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.savefig(os.path.join(SAVE_DIR, agent_name, "gnss_path.png"))
    plt.close()

    # Posição XY
    plt.figure()
    plt.plot(df['pos_x'], df['pos_y'])
    plt.title(f'{agent_name} - Posição XY')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.savefig(os.path.join(SAVE_DIR, agent_name, "position_xy.png"))
    plt.close()

    # Velocidade
    vel_mag = np.sqrt(df['vel_x']**2 + df['vel_y']**2 + df['vel_z']**2)
    plt.figure()
    plt.plot(df['timestamp'], vel_mag)
    plt.title(f'{agent_name} - Velocidade')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Velocidade (m/s)')
    plt.grid(True)
    plt.savefig(os.path.join(SAVE_DIR, agent_name, "velocity.png"))
    plt.close()

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(8001)

    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    all_agents = []
    actors_to_destroy = []

    try:
        random.shuffle(spawn_points)

        for i in range(AGENTS_PER_TYPE):
            # Spawn vehicle
            vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
            vehicle = world.spawn_actor(vehicle_bp, spawn_points[i])
            vehicle.set_autopilot(True, traffic_manager.get_port())
            name = f"vehicle_{vehicle.id}"

            # IMU
            imu_bp = create_imu_sensor(blueprint_library)
            imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)
            imu.listen(imu_listener(name, imu_data_storage))

            # GNSS
            gnss_bp = create_gnss_sensor(blueprint_library)
            gnss = world.spawn_actor(gnss_bp, carla.Transform(carla.Location(z=2)), attach_to=vehicle)
            gnss.listen(gnss_listener(name))

            all_agents.append((name, vehicle))
            actors_to_destroy.extend([vehicle, imu, gnss])

        for i in range(AGENTS_PER_TYPE):
            # Spawn pedestrian
            walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
            walker_transform = spawn_points[i + AGENTS_PER_TYPE]
            pedestrian = world.spawn_actor(walker_bp, walker_transform)
            pedestrian_control = carla.WalkerControl()
            pedestrian_control.direction = carla.Vector3D(1, 0, 0)
            pedestrian_control.speed = 1.0
            pedestrian.apply_control(pedestrian_control)

            name = f"pedestrian_{pedestrian.id}"

            # IMU
            imu_bp = create_imu_sensor(blueprint_library)
            imu = world.spawn_actor(imu_bp, carla.Transform(), attach_to=pedestrian)
            imu.listen(imu_listener(name, imu_data_storage))

            # GNSS
            gnss_bp = create_gnss_sensor(blueprint_library)
            gnss = world.spawn_actor(gnss_bp, carla.Transform(carla.Location(z=2)), attach_to=pedestrian)
            gnss.listen(gnss_listener(name))

            all_agents.append((name, pedestrian))
            actors_to_destroy.extend([pedestrian, imu, gnss])

        print(">> Simulando...")
        frame = 0
        while frame < int(SIMULATION_TIME / 0.05):
            world.tick()
            timestamp = world.get_snapshot().timestamp.elapsed_seconds
            for name, actor in all_agents:
                loc = actor.get_location()
                vel = actor.get_velocity()
                gnss = gnss_data_buffer.get(name, {'lat': None, 'lon': None, 'alt': None})

                imu_data_storage[name].append({
                    'timestamp': timestamp,
                    'accel_x': None, 'accel_y': None, 'accel_z': None,
                    'gyro_x': None, 'gyro_y': None, 'gyro_z': None,
                    'latitude': gnss['lat'], 'longitude': gnss['lon'], 'altitude': gnss['alt'],
                    'pos_x': loc.x, 'pos_y': loc.y, 'pos_z': loc.z,
                    'vel_x': vel.x, 'vel_y': vel.y, 'vel_z': vel.z
                })
            frame += 1

    finally:
        print("Encerrando simulação e limpando...")
        world.apply_settings(original_settings)
        for actor in actors_to_destroy:
            if actor.is_alive:
                actor.destroy()

        for name in imu_data_storage:
            plot_and_save(name, imu_data_storage[name])
        print("Pronto.")

if __name__ == "__main__":
    main()
