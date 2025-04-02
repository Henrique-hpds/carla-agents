import random
import carla
import os
import time

# Gambiarra forte, cuidado
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner

# custom modules
from modules.Camera import Camera
from modules.GNSS import GNSS
from modules.IMU import IMU
from modules.PositionModule import PositionModule
from modules.VelocityModule import VelocityModule

from datetime import datetime

def main():

    camera_drone = None
    camera_media = None
    camera_carro = None
    gnss_sensor = None
    imu_sensor = None

    sensores = [camera_drone, camera_media, camera_carro, gnss_sensor]
    agentes = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        world = client.load_world('Town01')

        settings = world.get_settings()
        settings.synchronous_mode = False
        # settings.fixed_delta_seconds = tick_time
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Nenhum ponto de spawn disponível no mapa!")
            return
            
        for _ in range(1):  # Adiciona 10/10 veículos NPC
            vehicle_2 = world.try_spawn_actor(random.choice(vehicle_bp), random.choice(spawn_points))
            if vehicle_2 is not None:
                agentes.append(vehicle_2)
                vehicle_2.set_autopilot(True)

        vehicle = world.spawn_actor(random.choice(vehicle_bp), spawn_points[0])

        if vehicle is not None:
            agentes.append(vehicle)
            print(f"Veículo {vehicle.type_id} spawnado com sucesso!")
        else:
            print("Falha ao spawnar o veículo!")
            return

        timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
        experiment_dir = f'data/exp_{timestamp}'

        if not os.path.exists(experiment_dir):
            os.makedirs(experiment_dir)

        if enable_camera:
            camera_drone = Camera("drone", world, 0, 0, 1000, -90, vehicle)
            camera_media = Camera("media", world, 0, 0, 100, -90, vehicle)
            camera_carro = Camera("carro", world, 1, 0, 2, 0, vehicle)
        gnss_sensor = GNSS("vehicle", world, 0, 0, 0, 0, tick_time, vehicle)
        imu_sensor = IMU("imu", world, 0, 0, 0, 0, tick_time, vehicle)
        position_sensor = PositionModule("vehicle", vehicle, tick_time)
        velocity_sensor = VelocityModule("vehicle", vehicle, tick_time)

        if enable_camera:
            camera_drone.start(experiment_dir)
            camera_media.start(experiment_dir)
            camera_carro.start(experiment_dir)
        gnss_sensor.start(experiment_dir)
        imu_sensor.start(experiment_dir)
        position_sensor.start(experiment_dir)
        velocity_sensor.start(experiment_dir)

        if has_behavior:
            agent = BasicAgent(vehicle)
        else:
            agent = BehaviorAgent(vehicle, behavior=car_behavior)

        destination = spawn_points[-1].location

        mid_point = spawn_points[len(spawn_points)//2].location
        agent.set_destination(mid_point)

        cont_routes = 0
        while True:
            if agent.done():
                agent.set_destination(destination)
                print("Chegou ao destino!")
                cont_routes += 1
                
                for _ in range(10): # fica parado por 10s
                    position_sensor.tick()
                    
                if cont_routes == 2:
                    break
                continue
            position_sensor.tick()
            velocity_sensor.tick()
            vehicle.apply_control(agent.run_step())      
            world.wait_for_tick()  

        if enable_camera:        
            camera_drone.stop()
            camera_media.stop()
            camera_carro.stop()
        gnss_sensor.stop()
        imu_sensor.stop()

        gnss_sensor.save_data(experiment_dir)
        gnss_sensor.plot_data(experiment_dir)

        imu_sensor.save_data(experiment_dir)
        imu_sensor.plot_data(experiment_dir)

        position_sensor.save_data(experiment_dir)
        position_sensor.plot_data(experiment_dir, mid_point)
        velocity_sensor.save_data(experiment_dir)
        velocity_sensor.plot_data(experiment_dir)

        print("Captura de imagens finalizada.")

    finally:

        for sensor in sensores:
            if sensor is not None:
                sensor.destroy()
                print(f"Sensor {sensor} destruído.")
            else:
                print(f"Sensor {sensor} já destruído ou não inicializado.")

        for agent in agentes:
            if agent is not None:
                agent.destroy()
                print(f"Agente {agent} destruído.")
            else:
                print(f"Agente já destruído ou não inicializado.")

        print("Finalizado e atores destruídos.")

if __name__ == '__main__':
    xodr_path = 'maps/map.xodr'
    tick_time = 0.1 # tempo de amostragem (máximo 0.1)
    enable_camera = False
    has_behavior = True
    car_behavior = 'aggressive'

    main()