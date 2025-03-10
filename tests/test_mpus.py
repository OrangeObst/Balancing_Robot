import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
from src.robot.mpu6050 import MyMPU6050 as mympu
from src.robot.mpu6050_copy import MyMPU6050 as othermpu
from smbus2 import SMBus

bus = SMBus(1)
other_mpu = othermpu(0x68)
my_mpu = mympu(bus)

def _get_other_data():
    accel_data = other_mpu.get_accel_data(g=True)
    # print(accel_data['x'])
    # print(accel_data['y'])
    # print(accel_data['z'])
    gyro_data = other_mpu.get_gyro_data()
    # print(gyro_data['x'])
    # print(gyro_data['y'])
    # print(gyro_data['z'])
    return accel_data['x'], accel_data['y'], accel_data['z'], gyro_data['x'], gyro_data['y'], gyro_data['z']

def get_my_data():
    return my_mpu.get_all_data()


if __name__ == "__main__":
    dt = 0.01
    timer = time.time()
    while ((time.time() - timer) < 10):
        loop_start = time.time()
        oax, oay, oaz, ogx, ogy, ogz = _get_other_data()
        ax, ay, az, gx, gy, gz = get_my_data()
        print(f'Other: {oax:6.4f} {oay:6.4f} {oaz:6.4f} {ogx:6.4f} {ogy:6.4f} {ogz:6.4f}')
        print(f'Mine : {ax:6.4f} {ay:6.4f} {az:6.4f} {gx:6.4f} {gy:6.4f} {gz:6.4f}')
        print('------------------------------')

    #     # data_list = mpu.get_fifo_buffer()
    #     data = mpu.get_all_data()
    #     # if data_list is not None:
    #         # for data in data_list:
    #     pitch_from_acceleration = degrees(atan2(data[0], -data[2]))
    #     pitch_gyro_integration = previous_pitch + data[4] * dt

    #     previous_pitch = alpha * pitch_gyro_integration + (1 - alpha) * pitch_from_acceleration
    #     counter += 1
    #     print(f'0: {data[0]:8.4f}, 1: {data[1]:8.4f}, 2: {data[2]:8.4f}, 3: {data[3]:8.4f}, 4: {data[4]:8.4f}, 5: {data[5]:8.4f}, Pitch: {previous_pitch:6.4f}')
            
        loop_end = time.time()
        loop_duration = loop_end - loop_start

        sleep_time = max(0, dt - loop_duration)
        time.sleep(sleep_time)

    # print(counter)