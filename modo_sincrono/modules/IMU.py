import matplotlib.pyplot as plt
import carla
import csv
import os

class IMU:
    label: str
    transform: carla.Transform
    sensor: carla.Actor
    data: dict
    time: float
    tick_time: float

    def __init__(self, label, world, x, y, z, rotation, tick_time, attached_ob=None):
        blueprint = world.get_blueprint_library().find('sensor.other.imu')
        blueprint.set_attribute('sensor_tick', str(0.05))
        self.transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=rotation))
        self.sensor = world.spawn_actor(blueprint, self.transform, attach_to=attached_ob)
        self.tick_time = tick_time
        self.label = label
        self.data = {}
        self.time = 0.0

    def callback(self, data):
        self.data[self.time] = {
            'gyro': data.gyroscope,
            'accel': data.accelerometer - carla.Vector3D(x=0,y=0,z=9.81),
            'compass': data.compass
        }
        self.time += self.tick_time

    def start(self, experiment_dir):
        if not os.path.exists(f'{experiment_dir}/IMU_{self.label}'):
            os.makedirs(f'{experiment_dir}/IMU_{self.label}')                

        if self.sensor is not None:
            self.sensor.listen(lambda event: self.callback(event))

    def stop(self):
        if self.sensor is not None:
            self.sensor.stop()

    def save_data(self, experiment_dir):
        file_path = f'{experiment_dir}/IMU_{self.label}/data.csv'
        with open(file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time', 'Accel_x', 'Accel_y', 'Accel_z', 'Gyro_x', 'Gyro_y', 'Gyro_z', 'Compass'])
            cont = 0
            for time, data in self.data.items():
                if cont < 2:
                    cont += 1
                    continue
                writer.writerow([time, data['accel'].x, data['accel'].y, data['accel'].z,
                                 data['gyro'].x, data['gyro'].y, data['gyro'].z,
                                 data['compass']])

    def plot_data(self, experiment_dir: str):
        time = []
        accel_x = []
        accel_y = []
        accel_z = []
        gyro_x = []
        gyro_y = []
        gyro_z = []
        compass = []

        filename = f'{experiment_dir}/IMU_{self.label}/data.csv'
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                time.append(float(row['Time']))
                accel_x.append(float(row['Accel_x']))
                accel_y.append(float(row['Accel_y']))
                accel_z.append(float(row['Accel_z']))
                gyro_x.append(float(row['Gyro_x']))
                gyro_y.append(float(row['Gyro_y']))
                gyro_z.append(float(row['Gyro_z']))
                compass.append(float(row['Compass']))

        # plot acceleration data
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
        plt.savefig(f'{experiment_dir}/IMU_{self.label}/accel.png')

        # plot gyroscope data
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
        plt.savefig(f'{experiment_dir}/IMU_{self.label}/gyro.png')

        # plot compass data
        plt.figure(figsize=(15, 10))

        plt.plot(time, compass, label='Compass', color='purple')

        plt.xlabel('Time (s)')
        plt.ylabel('Compass (degrees)')
        plt.legend()
        plt.title('Compass Data over Time')

        plt.tight_layout()
        plt.savefig(f'{experiment_dir}/IMU_{self.label}/compass.png')

    
    def destroy(self):
        if self.sensor is not None:
            self.sensor.destroy()
            self.sensor = None
        else:
            print("Sensor já destruído ou não inicializado.")
