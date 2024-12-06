import asyncio
import json
import os
import socket
import time
import ntplib
import numpy as np
from scipy.interpolate import interp1d
from math import degrees, atan2, sqrt

class UDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, server):
        self.server = server

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        message = json.loads(data.decode('utf-8'))
        self.server.collected_data.append(message)
        print(f"Received data: {data}")

class ServerSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.total_latency = 0
        self.message_counter = 0
        self.alpha = 0.98
        self.previous_pitch = 0.0
        self.angle = 0.0
        self.collected_data = []
        self.data = []
        self.synchronize_time()

    def synchronize_time(self):
        try:
            client = ntplib.NTPClient()
            response = client.request('pool.ntp.org')
            os.system(f'date -s @{response.tx_time}')
            print(f"Time synchronized to {time.ctime(response.tx_time)}")
        except Exception as e:
            print(f"Failed to synchronize time: {e}")

    async def synchronize_and_interpolate(self):
        timestamps = []
        imu_data = []
        delays = []
        for data_set, dt in self.collected_data:
            timestamps.append(data_set[0])
            imu_data.append(data_set[1])
            delays.append(dt)

        async with asyncio.Lock():
            self.collected_data.clear()

        # Interpolation
        interp_func = interp1d(timestamps, imu_data, axis=0, fill_value="extrapolate")
        new_timestamps = np.arange(timestamps[0], timestamps[-1], 3)
        interpolated_data = interp_func(new_timestamps)
        self.average_data(interpolated_data, len(interpolated_data))

    async def average_data(self, interpolated_data, window_size=3):
        averaged_data = np.array([
            np.convolve(interpolated_data[:, i], np.ones(window_size) / window_size, mode='valid')
            for i in range(interpolated_data.shape[1])
        ]).T
        self.data = averaged_data[0]

    async def update_angle_handler(self, now, dt):
        if len(self.collected_data) > 3:
            await self.synchronize_and_interpolate()
            # Calculate pitch from accelerometer and gyroscope
            pitch_from_acceleration = degrees(atan2(self.data[0], sqrt(self.data[1]**2 + self.data[2]**2)))
            pitch_gyro_integration = self.previous_pitch + self.data[4] * dt
            # Apply complementary filter
            pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration
            self.previous_pitch = pitch
            self.angle = pitch
            print(f'Angle: {self.angle:.5}, imu_data: {self.data}, dt: {dt}')

    async def send_data(self):
        while True:
            await asyncio.sleep(0.01)
            await self.update_angle_handler(time.time(), 0.01)

    async def receive_data(self, reader, writer):
        while True:
            data = await reader.read(128)
            message = json.loads(data.decode('utf-8'))
            self.collected_data.append(message)
            print(f"Received data: {data}")

    async def run(self):
        server = await asyncio.start_server(self.receive_data, self.host, self.port)
        async with server:
            await server.serve_forever()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    server = ServerSocket('192.168.1.189', 1883)
    loop.create_task(server.run())
    loop.run_forever()
