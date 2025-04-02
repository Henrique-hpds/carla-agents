from matplotlib import pyplot as plt
from carla import Vector3D
import numpy as np
import csv

def plot_gnss(filename: str, experiment_dir: str):
    time = []
    latitudes = []
    longitudes = []
    altitudes = []

    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            time.append(float(row['time']))
            latitudes.append(float(row['latitude']))
            longitudes.append(float(row['longitude']))
            altitudes.append(float(row['altitude']))

    fig, axs = plt.subplots(3, 1, figsize=(15, 15), sharex=True)

    axs[0].plot(time, latitudes, label='Latitude', color='blue')
    axs[0].set_ylabel('Latitude (°)')
    axs[0].legend()

    axs[1].plot(time, longitudes, label='Longitude', color='orange')
    axs[1].set_ylabel('Longitude (°)')
    axs[1].legend()

    axs[2].plot(time, altitudes, label='Altitude', color='green')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Altitude (m)')
    axs[2].legend()

    fig.suptitle('GNSS Data over Time')
    plt.tight_layout()
    plt.savefig(f'{experiment_dir}/plots/gnss.png')

def plot_accel(filename: str, experiment_dir: str):
    time = []
    accel_x = []
    accel_y = []
    accel_z = []

    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            time.append(float(row['time']))
            accel_x.append(float(row['accel_x']))
            accel_y.append(float(row['accel_y']))
            accel_z.append(float(row['accel_z']))

    fig, axs = plt.subplots(3, 1, figsize=(15, 15), sharex=True)

    axs[0].plot(time, accel_x, label='Accel X', color='red')
    axs[0].set_ylabel('Acceleration X (m/s²)')
    axs[0].legend()

    axs[1].plot(time, accel_y, label='Accel Y', color='green')
    axs[1].set_ylabel('Acceleration Y (m/s²)')
    axs[1].legend()

    axs[2].plot(time, accel_z, label='Accel Z', color='blue')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Acceleration Z (m/s²)')
    axs[2].legend()

    fig.suptitle('Acceleration Data over Time')
    plt.tight_layout()
    plt.savefig(f'{experiment_dir}/plots/accel.png')

def plot_gyro(filename: str, experiment_dir: str):
    time = []
    gyro_x = []
    gyro_y = []
    gyro_z = []

    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            time.append(float(row['time']))
            gyro_x.append(float(row['gyro_x']))
            gyro_y.append(float(row['gyro_y']))
            gyro_z.append(float(row['gyro_z']))

    fig, axs = plt.subplots(3, 1, figsize=(15, 15), sharex=True)

    axs[0].plot(time, gyro_x, label='Gyro X', color='red')
    axs[0].set_ylabel('Angular Velocity X (rad/s)')
    axs[0].legend()

    axs[1].plot(time, gyro_y, label='Gyro Y', color='green')
    axs[1].set_ylabel('Angular Velocity Y (rad/s)')
    axs[1].legend()

    axs[2].plot(time, gyro_z, label='Gyro Z', color='blue')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Angular Velocity Z (rad/s)')
    axs[2].legend()

    fig.suptitle('Gyroscope Data over Time')
    plt.tight_layout()
    plt.savefig(f'{experiment_dir}/plots/gyro.png')

def plot_gps(filename: str, experiment_dir: str):
    time = []
    gps_x = []
    gps_y = []
    gps_z = []

    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            time.append(float(row['time']))
            gps_x.append(float(row['gps_x']))
            gps_y.append(float(row['gps_y']))
            gps_z.append(float(row['gps_z']))

    fig, axs = plt.subplots(2, 1, figsize=(15, 15))

    # Gráfico de coordenadas GPS ao longo do tempo
    axs[0].plot(time, gps_x, label='GPS X', color='blue')
    axs[0].plot(time, gps_y, label='GPS Y', color='orange')
    axs[0].plot(time, gps_z, label='GPS Z', color='green')
    axs[0].set_ylabel('GPS Coordinates (m)')
    axs[0].legend()

    # Gráfico de trajetória GPS com gradiente de cores
    sc = axs[1].scatter(gps_x, gps_y, c=time, cmap='viridis', s=10, label='Trajectory')
    axs[1].scatter(gps_x[0], gps_y[0], label='Start', color='green', s=100)
    axs[1].scatter(gps_x[-1], gps_y[-1], label='End', color='orange', s=100)

    # # Adicionar setas para indicar direção
    # for i in range(0, len(gps_x) - 1, max(1, len(gps_x) // 50)):  # Adiciona setas espaçadas
    #     axs[1].arrow(gps_x[i], gps_y[i], gps_x[i + 1] - gps_x[i], gps_y[i + 1] - gps_y[i],
    #                  head_width=5, head_length=5, fc='black', ec='black', alpha=0.6)

    # Adicionar setas para indicar direção
    for i in range(0, len(gps_x) - 1, max(1, len(gps_x) // 50)):  # Adiciona setas espaçadas
        dx = gps_x[i + 1] - gps_x[i]
        dy = gps_y[i + 1] - gps_y[i]
        norm = np.sqrt(dx**2 + dy**2)  # Normaliza o vetor
        if norm > 0:  # Evita divisão por zero
            dx /= norm
            dy /= norm
        axs[1].arrow(gps_x[i], gps_y[i], dx * 5, dy * 5,  # Escala o tamanho da seta
                     head_width=5, head_length=5, fc='black', ec='black', alpha=0.6)

    axs[1].set_xlabel('GPS X (m)')
    axs[1].set_ylabel('GPS Y (m)')
    axs[1].legend()
    fig.colorbar(sc, ax=axs[1], label='Time (s)')

    fig.suptitle('GPS Data over Time and Trajectory')
    plt.tight_layout()
    plt.savefig(f'{experiment_dir}/plots/gps.png')

def plot_compass(filename: str, experiment_dir: str):
    time = []
    compass = []

    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            time.append(float(row['time']))
            compass.append(float(row['compass']))

    plt.figure(figsize=(15, 10))

    plt.plot(time, compass, label='Compass', color='purple')

    plt.xlabel('Time (s)')
    plt.ylabel('Compass (degrees)')
    plt.legend()
    plt.title('Compass Data over Time')

    plt.tight_layout()
    plt.savefig(f'{experiment_dir}/plots/compass.png')