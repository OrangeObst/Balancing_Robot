import socket
import json
import sys
import threading
import ntplib
import os
import time
import numpy as np
from typing import Callable
from math import degrees, sqrt, atan2
from scipy.interpolate import interp1d


class TimedTask:
    def __init__(self, delay: float, run: Callable[[float, float], None], start_delay: float = .0):
        self.delay = delay
        self.run = run
        self.start_delay = start_delay
        self.started = False
        self.last_exec_ts = .0

    def loop(self):
        now = time.time()
        dt = now - self.last_exec_ts
        if not self.started:
            if dt >= self.start_delay:
                self.started = True
                self.last_exec_ts = now
        else:
            if dt >= self.delay:
                self.last_exec_ts = now
                self.run(now, dt)

    def run(self, now, dt):
        pass

class ServerSocket:
    def __init__(self, host, port, ntp_server='pool.ntp.org'):
        self.host = host
        self.port = port
        self.ntp_server = ntp_server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        # self.sock.setblocking(False)
        self.lock = threading.Lock()

        self.ntp_server = 'pool.ntp.org'
        self.total_latency = 0
        self.message_counter = 0
        self.alpha = 0.98
        self.previous_pitch = 0.0
        self.angle = 0.0

        self.collected_data = []
        self.data = []

        self.synchronize_time()
        self.last_time = time.time()

        self.update_angle_task = TimedTask(delay=0.01, run=self.update_angle_handler)


    def synchronize_time(self):
        try:
            client = ntplib.NTPClient()
            response = client.request(self.ntp_server)
            os.system(f'date -s @{response.tx_time}')
            print(f"Time synchronized to {time.ctime(response.tx_time)}")
        except Exception as e:
            print(f"Failed to synchronize time: {e}")

    def synchronize_and_interpolate(self, interval=3):
        # timestamps = [ts for ts,_ in self.data_buffer[0]]
        # imu_data = np.array([dp for _,dp in self.data_buffer[0]])
        timestamps = []
        imu_data = []
        delays = []
        with self.lock:
            for data_set, t in self.collected_data:
                timestamps.append(data_set[0])
                imu_data.append(data_set)
                delays.append(t)
            print(imu_data, delays)
            self.collected_data.clear()

        # Interpolation
        interp_func = interp1d(timestamps, imu_data, axis=0, fill_value="extrapolate")
        new_timestamps = np.arange(timestamps[0], timestamps[-1], interval)
        # test interval = timestamps[-1] - timestamps[0]
        interpolated_data = interp_func(new_timestamps)
        print(interpolated_data)
        self.average_data(interpolated_data, len(interpolated_data))
    
    def average_data(self, interpolated_data, window_size=3):
        averaged_data = np.array([
            np.convolve(interpolated_data[:, i], np.ones(window_size) / window_size, mode='valid')
            for i in range(interpolated_data.shape[1])
        ]).T
        self.data = averaged_data[0]

    def update_angle_handler(self, now, ttdt):
        with self.lock:
            data_len = len(self.collected_data)

        if data_len > 3:
            # self.synchronize_and_interpolate()
            with self.lock:
                self.data = self.collected_data.copy()
                self.collected_data.clear()

            for data in self.data:
                # Calculate pitch from accelerometer and gyroscope
                dt = data[1] - self.last_time
                pitch_from_acceleration = degrees(atan2(data[0][0], sqrt(data[0][1]**2 + data[0][2]**2)))
                pitch_gyro_integration = self.previous_pitch + data[0][4] * dt

                # Apply complementary filter
                pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration

                self.last_time = data[1]
                self.previous_pitch = pitch
                self.angle = pitch
                print(f'Angle: {self.angle:.5}, imu_data: {data}, dt: {dt}')
                


    def send_data(self):
        while True:
            self.update_angle_task.loop()
            time.sleep(0.01)

    def receive_data(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(128)
            except socket.timeout as e:
                err = e.args[0]
                # this next if/else is a bit redundant, but illustrates how the
                # timeout exception is setup
                if err == 'timed out':
                    print('recv timed out, retry later')
                    continue
                else:
                    print(e)
                    sys.exit(1)
            except socket.error as e:
                # Something else happened, handle error, exit, etc.
                print(e)
                sys.exit(1)
            
            if len(data) == 0:
                print('orderly shutdown on server end')
                sys.exit(0)
            else:
                try:
                    message = json.loads(data.decode('utf-8'))
                    # dt = time.time() - message[1]
                    # self.update_angle_handler(message[0], dt)
                    with self.lock:
                        self.collected_data.append(message)
                    # print(self.collected_data)
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                    print(f"Received data: {data}")

    def run(self):
        send_thread = threading.Thread(target=self.send_data)
        receive_thread = threading.Thread(target=self.receive_data)
        send_thread.start()
        receive_thread.start()
        next_time = time.time() + 2
        try:
            while True:
                current_time = time.time()
                if current_time > next_time:
                    print("Still running!")
                    next_time = current_time +2
        except KeyboardInterrupt:
            print("Caught keyboard interrupt, exiting")
        finally:
            self.sock.close()
        
        send_thread.join()
        receive_thread.join()


if __name__ == "__main__":
    server = ServerSocket('192.168.1.189', 1883)    # '192.168.1.189' '192.168.1.237'
    server.run()