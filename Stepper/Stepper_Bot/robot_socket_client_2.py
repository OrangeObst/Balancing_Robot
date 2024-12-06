import asyncio
import json
import time
import ntplib
import os
from smbus2 import SMBus
from util import mpu6050

class UDPProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport

    async def send_message(self, message):
        data = json.dumps(message).encode('utf-8')
        self.transport.sendto(data)
        print(f'Send: {data}')

class UDPClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        bus = SMBus(1)
        self.mpu = mpu6050.MyMPU6050(bus)

    def synchronize_time(self):
        try:
            client = ntplib.NTPClient()
            response = client.request('pool.ntp.org')
            os.system(f'date -s @{response.tx_time}')
            print(f"Time synchronized to {time.ctime(response.tx_time)}")
        except Exception as e:
            print(f"Failed to synchronize time: {e}")

    async def send(self):
        while True:
            loop = asyncio.get_event_loop()
            transport, protocol = await loop.create_datagram_endpoint(
                UDPProtocol,
                remote_addr=(self.host, self.port)
            )
            imu_data = self.mpu.MPU_ReadData()
            imu_data = [round(value, 7) for value in imu_data]
            timestamp = time.time()
            rounded_timestamp = round(timestamp, 5)
            await protocol.send_message([imu_data, rounded_timestamp])
            time.sleep(1)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    client = UDPClient('192.168.1.189', 1883)
    loop.create_task(client.send())
    loop.run_forever()
