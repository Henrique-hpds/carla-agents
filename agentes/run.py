import random
import carla
import os
import time

# Gambiarra forte, cuidado
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.tools.misc import get_speed

# custom modules
from modules.Camera import Camera
from modules.GNSS import GNSS
from modules.IMU import IMU
from modules.PositionModule import PositionModule
from modules.VelocityModule import VelocityModule

from datetime import datetime

def main():

    xodr_path = 'maps/map.xodr'
    tick_time = 0.1 # tempo de amostragem (máximo 0.1)
    enable_camera_pedestrian = False
    enable_camera_vehicle = True
    has_behavior = True
    car_behavior = 'cautious' # cautious, aggressive, normal

    camera_drone = None
    camera_media = None
    camera_carro = None
    gnss_sensor = None
    imu_sensor = None

    sensores = [camera_drone, camera_media, camera_carro, gnss_sensor]
    agentes = []
    pedestres = []
    pedestres_controller = []

    sensores_pedestres = {}

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        world = client.load_world('Town01')

        settings = world.get_settings()
        settings.synchronous_mode = False
        # settings.fixed_delta_seconds = tick_time
        world.apply_settings(settings)
        
        traffic_manager = client.get_trafficmanager(8001)
        traffic_manager.set_synchronous_mode(False)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        traffic_manager.set_hybrid_physics_mode(True)  # Para lidar com muitos veículos

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
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Nenhum ponto de spawn disponível no mapa!")
            return
            
        for _ in range(1): # Veículos NPC
            vehicle_2 = world.try_spawn_actor(random.choice(vehicle_bp), random.choice(spawn_points))
            if vehicle_2 is not None:
                agentes.append(vehicle_2)
                vehicle_2.set_autopilot(True)

        vehicle = world.spawn_actor(vehicle_bp.find("vehicle.bmw.grandtourer"), spawn_points[0])

        if vehicle is not None:
            agentes.append(vehicle)
            print(f"Veículo {vehicle.type_id} spawnado com sucesso!")
        else:
            print("Falha ao spawnar o veículo!")
            return

        timestamp = datetime.now().strftime('%Y-%m-%d_%Hh-%Mm-%Ss')
        experiment_dir = f'data/exp_{timestamp}'
        experiment_dir_pedestres = f'data/exp_{timestamp}/pedestres'

        if not os.path.exists(experiment_dir):
            os.makedirs(experiment_dir)

        pedestrian_bp = blueprint_library.filter('walker.pedestrian.*')
        pedestrian_controller_bp = blueprint_library.find('controller.ai.walker')

        for i in range(1):  # Número de pedestres a serem criados
            spawn_point_pedestrian = world.get_random_location_from_navigation()
            spawn_transform = carla.Transform(spawn_point_pedestrian)
            pedestrian = world.try_spawn_actor(random.choice(pedestrian_bp), spawn_transform)
            if pedestrian is not None:

                controller = world.spawn_actor(pedestrian_controller_bp, carla.Transform(), attach_to=pedestrian)
                pedestres_controller.append(controller)
                controller.start()
                controller.go_to_location(world.get_random_location_from_navigation())
                controller.set_max_speed(1.5)
                pedestres.append(pedestrian)

                sensores_pedestres[pedestrian] = {}
                if enable_camera_pedestrian:
                    camera_pedestre = Camera(f"drone_pedestre_{i}", world, 0, 0, 20, -90, pedestrian)
                    sensores_pedestres[pedestrian]['camera_drone'] = camera_pedestre
                    
                    camera_pedestre = Camera(f"visao_pedestre_{i}", world, 0, 0, 0, 0, pedestrian)
                    sensores_pedestres[pedestrian]['camera_visao'] = camera_pedestre

                gnss_pedestre = GNSS(f"pedestre_{i}", world, 0, 0, 0, 0, tick_time, pedestrian)
                sensores_pedestres[pedestrian]['gnss'] = gnss_pedestre

                imu_pedestre = IMU(f"pedestre_{i}", world, 0, 0, 0, 0, tick_time, pedestrian)
                sensores_pedestres[pedestrian]['imu'] = imu_pedestre

                position_sensor_pedestre = PositionModule(f"pedestre_{i}", pedestrian, tick_time)
                sensores_pedestres[pedestrian]['position'] = position_sensor_pedestre

                velocity_sensor_pedestre = VelocityModule(f"pedestre_{i}", pedestrian, tick_time)
                sensores_pedestres[pedestrian]['velocity'] = velocity_sensor_pedestre

                print(f"Pedestre {i} spawnado com sucesso!")
                    
        if enable_camera_vehicle:
            # camera_drone = Camera("drone", world, 0, 0, 1000, -90, vehicle)
            # camera_media = Camera("media", world, 0, 0, 100, -90, vehicle)
            camera_carro = Camera("carro", world, -5, 0, 2, 0, vehicle)
        gnss_sensor = GNSS("vehicle", world, 0, 0, 0, 0, tick_time, vehicle)
        imu_sensor = IMU("vehicle", world, 0, 0, 0, 0, tick_time, vehicle)
        position_sensor = PositionModule("vehicle", vehicle, tick_time)
        velocity_sensor = VelocityModule("vehicle", vehicle, tick_time)

        if enable_camera_vehicle:
            # camera_drone.start(experiment_dir)
            # camera_media.start(experiment_dir)
            camera_carro.start(experiment_dir)
        gnss_sensor.start(experiment_dir)
        imu_sensor.start(experiment_dir)
        position_sensor.start(experiment_dir)
        velocity_sensor.start(experiment_dir)

        for pedestrian in pedestres:
            if enable_camera_pedestrian:
                sensores_pedestres[pedestrian]['camera_drone'].start(experiment_dir_pedestres)
                sensores_pedestres[pedestrian]['camera_visao'].start(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['gnss'].start(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['imu'].start(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['position'].start(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['velocity'].start(experiment_dir_pedestres)
                

        if not has_behavior:
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
                
                if cont_routes == 2:
                    break
                
                for _ in range(10): # fica parado por 10s
                    position_sensor.tick()
                    for pedestrian in pedestres:
                        sensores_pedestres[pedestrian]['position'].tick()
                        sensores_pedestres[pedestrian]['velocity'].tick()

                continue

            for pedestrian in pedestres:
                sensores_pedestres[pedestrian]['position'].tick()
                sensores_pedestres[pedestrian]['velocity'].tick()
            print(vehicle.get_transform().location)
            position_sensor.tick()
            velocity_sensor.tick()
            vehicle.apply_control(agent.run_step())
            world.wait_for_tick()

        if enable_camera_vehicle:
            # camera_drone.stop()
            # camera_media.stop()
            camera_carro.stop()
        gnss_sensor.stop()
        imu_sensor.stop()

        for pedestrian in pedestres:
            if enable_camera_pedestrian:
                sensores_pedestres[pedestrian]['camera_drone'].stop()
                sensores_pedestres[pedestrian]['camera_drone'].save_data(experiment_dir_pedestres)
                sensores_pedestres[pedestrian]['camera_visao'].plot_data(experiment_dir_pedestres)

                sensores_pedestres[pedestrian]['camera_visao'].stop()
                sensores_pedestres[pedestrian]['camera_visao'].save_data(experiment_dir_pedestres)
                sensores_pedestres[pedestrian]['camera_visao'].plot_data(experiment_dir_pedestres)
                
            sensores_pedestres[pedestrian]['gnss'].stop()
            sensores_pedestres[pedestrian]['gnss'].save_data(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['gnss'].plot_data(experiment_dir_pedestres)

            sensores_pedestres[pedestrian]['imu'].stop()
            sensores_pedestres[pedestrian]['imu'].save_data(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['imu'].plot_data(experiment_dir_pedestres)

            sensores_pedestres[pedestrian]['position'].save_data(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['position'].plot_data(experiment_dir_pedestres, mid_point)

            sensores_pedestres[pedestrian]['velocity'].save_data(experiment_dir_pedestres)
            sensores_pedestres[pedestrian]['velocity'].plot_data(experiment_dir_pedestres)

        gnss_sensor.save_data(experiment_dir)
        gnss_sensor.plot_data(experiment_dir)

        imu_sensor.save_data(experiment_dir)
        imu_sensor.plot_data(experiment_dir)

        position_sensor.save_data(experiment_dir)
        position_sensor.plot_data(experiment_dir, mid_point)
        velocity_sensor.save_data(experiment_dir)
        velocity_sensor.plot_data(experiment_dir)

        print("Captura finalizada.")

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

        for pedestrian in pedestres:
            if enable_camera_pedestrian:
                sensores_pedestres[pedestrian]['camera_drone'].destroy()
                sensores_pedestres[pedestrian]['camera_visao'].destroy()
            sensores_pedestres[pedestrian]['gnss'].destroy()
            sensores_pedestres[pedestrian]['imu'].destroy()
            pedestrian.destroy()
            print(f"Pedestre {pedestrian} e seus sensores destruído.")

        for controller in pedestres_controller:
            if controller is not None:
                controller.destroy()
                print(f"Controlador {controller} destruído.")
            else:
                print(f"Controlador já destruído ou não inicializado.")

        print("Finalizado e atores destruídos.")

if __name__ == '__main__':
    main()