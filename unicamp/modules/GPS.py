import matplotlib.pyplot as plt
import numpy as np
import carla
import time
import csv
import os

class GPS:
    label: str
    attached_ob: carla.Actor
    data: dict
    time: float
    tick_time: float

    def __init__(self, label, obj, thick_time):
        
        self.attached_ob = obj
        self.label = label
        self.tick_time = thick_time
        self.data = {}
        self.time = 0.0

    def start(self, experiment_dir):
         if not os.path.exists(f'{experiment_dir}/GPS_{self.label}'):
            os.makedirs(f'{experiment_dir}/GPS_{self.label}')                

    def tick(self):
        try:
            current_data = self.attached_ob.get_location()
            self.data[self.time] = {
                'x': current_data.x,
                'y': current_data.y,
                'z': current_data.z
            }
            self.time += self.tick_time
        except Exception as e:
            print(f"Error getting GPS data: {e}")

    def save_data(self, experiment_dir):
        file_path = f'{experiment_dir}/GPS_{self.label}/data.csv'
        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'X', 'Y', 'Z'])
            for time, data in self.data.items():
                writer.writerow([time, data['x'], data['y'], data['z']])

    def plot_data(self, experiment_dir, mid_point):
        time = []
        gps_x = []
        gps_y = []
        gps_z = []
        filename = f'{experiment_dir}/GPS_{self.label}/data.csv'
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                time.append(float(row['Time']))
                gps_x.append(float(row['X']))
                gps_y.append(float(row['Y']))
                gps_z.append(float(row['Z']))

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
        axs[1].scatter(mid_point.x, mid_point.y, label='Mid Point', color='red', s=100)
        axs[1].set_xlabel('GPS X (m)')
        axs[1].set_ylabel('GPS Y (m)')
        axs[1].legend()
        fig.colorbar(sc, ax=axs[1], label='Time (s)')

        fig.suptitle('GPS Data over Time and Trajectory')
        plt.tight_layout()
        plt.savefig(f'{experiment_dir}/GPS_{self.label}/plot.png')

