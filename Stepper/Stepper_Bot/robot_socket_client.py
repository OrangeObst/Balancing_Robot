from smbus2 import SMBus
from util import mpu6050
import socket
import time
import json
import ntplib
import os
import threading

class RobotSocket:
    def __init__(self, host, port, ntp_server='pool.ntp.org'):
        self.host = host
        self.port = port
        self.ntp_server = ntp_server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port + 1))  # Bind to a different port for receiving commands
        self.sock.setblocking(False)  # Set socket to non-blocking mode

        bus = SMBus(1)
        self.mpu = mpu6050.MyMPU6050(bus)


    def synchronize_time(self):
        try:
            client = ntplib.NTPClient()
            response = client.request(self.ntp_server)
            os.system(f'date -s @{response.tx_time}')
            print(f"Time synchronized to {time.ctime(response.tx_time)}")
        except Exception as e:
            print(f"Failed to synchronize time: {e}")

    def send_data(self):
        start_time = time.time()
        end_time = start_time + 10
        while time.time() < end_time:
            imu_data = self.mpu.MPU_ReadData()
            imu_data = [round(value, 7) for value in imu_data]
            timestamp = time.time()
            rounded_timestamp = round(timestamp, 5)
            message = json.dumps([imu_data, rounded_timestamp]).encode('utf-8')
            self.sock.sendto(message, (self.host, self.port))
            print(f"Sent: {message}")
            time.sleep(0.005)  # Send data every 2ms

    def receive_data(self):
        start_time = time.time()
        end_time = start_time + 10
        while time.time() < end_time:
            try:
                data, addr = self.sock.recvfrom(128)  # Buffer size is 128 bytes
                command = json.loads(data.decode('utf-8'))
                self.process_command(command)
            except BlockingIOError:
                # No data received, continue the loop
                time.sleep(0.01)
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
                print(f"Received data: {data}")

    def process_command(self, command):
        # Placeholder for motor command processing logic
        print(f"Received command: {command}")

    def run(self):
        send_thread = threading.Thread(target=self.send_data)
        receive_thread = threading.Thread(target=self.receive_data)
        send_thread.start()
        receive_thread.start()
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Caught keyboard interrupt")
        finally:
            self.sock.close()
        
        send_thread.join()
        receive_thread.join()


if __name__ == "__main__":
    client = RobotSocket('192.168.1.189', 1883)   # '192.168.1.189' '192.168.1.237'
    client.run()
