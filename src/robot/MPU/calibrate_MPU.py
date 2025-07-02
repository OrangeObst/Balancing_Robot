from smbus2 import SMBus
from robot.MPU.MyMpu6050 import MyMPU6050

if __name__ == "__main__":
    bus = SMBus(1)
    mpu = MyMPU6050(bus)

    mpu.calibrate_sensor(5)
    # TODO: Save offsets to settings.ini and grab duration from CLI argument