from smbus2 import SMBus
import numpy as np
import math
import time

'''
    ------------------ MPU REGISTERS ------------------
'''
_MPU6050_DEFAULT_ADDRESS = 0x68  # MPU6050 default i2c address w/ AD0 low
_MPU6050_DEVICE_ID = 0x68  # The correct MPU6050_WHO_AM_I value
_MPU6050_SELF_TEST_X = 0x0D  # Self test factory calibrated values register
_MPU6050_SELF_TEST_Y = 0x0E  # Self test factory calibrated values register
_MPU6050_SELF_TEST_Z = 0x0F  # Self test factory calibrated values register
_MPU6050_SELF_TEST_A = 0x10  # Self test factory calibrated values register
_MPU6050_SMPLRT_DIV = 0x19  # sample rate divisor register
_MPU6050_CONFIG = 0x1A  # General configuration register
_MPU6050_GYRO_CONFIG = 0x1B  # Gyro specfic configuration register
_MPU6050_ACCEL_CONFIG = 0x1C  # Accelerometer specific configration register
_MPU6050_INT_PIN_CONFIG = 0x37  # Interrupt pin configuration register
_MPU6050_ACCEL_OUT_X = 0x3B  # base address for sensor data reads
_MPU6050_ACCEL_OUT_Y = 0x3D
_MPU6050_ACCEL_OUT_Z = 0x3F
_MPU6050_TEMP_OUT = 0x41  # Temperature data high byte register
_MPU6050_GYRO_OUT_X = 0x43  # base address for sensor data reads
_MPU6050_GYRO_OUT_Y = 0x45
_MPU6050_GYRO_OUT_Z = 0x47
_MPU6050_SIG_PATH_RESET = 0x68  # register to reset sensor signal paths
_MPU6050_USER_CTRL = 0x6A  # FIFO and I2C Master control register
_MPU6050_PWR_MGMT_1 = 0x6B  # Primary power/sleep control register
_MPU6050_PWR_MGMT_2 = 0x6C  # Secondary power/sleep control register
_MPU6050_WHO_AM_I = 0x75  # Divice ID register

'''
    ------------------ SCALES ------------------
'''
ACCEL_SCALE = 16384
GYRO_SCALE = 131

STANDARD_GRAVITY = 1 # 9.80665


class MyMPU6050:
    def __init__(self, i2c_bus: SMBus, address: int = _MPU6050_DEFAULT_ADDRESS) -> None:

        self.bus = i2c_bus
        self.address = address

        self.AX_OFFSET = 0.0
        self.AY_OFFSET = 0.0
        self.AZ_OFFSET = 0.0

        self.GX_OFFSET = 0.0
        self.GY_OFFSET = 0.0
        self.GZ_OFFSET = 0.0

        self.GYRO_DRIFT_X = -0.0001191070
        self.GYRO_DRIFT_Y = -0.0000127492
        self.GYRO_DRIFT_Z = 0.00001833274

        self.initialize_mpu()
    

    def initialize_mpu(self):
        # Wake up MPU
        self.bus.write_byte_data(self.address, _MPU6050_PWR_MGMT_1, 1)
        time.sleep(.01)

        # Reset device
        self.bus.write_byte_data(self.address, _MPU6050_SIG_PATH_RESET, 7)
        time.sleep(.01)
        
        # Settings
        self.set_accel_config(0)
        self.set_gyro_config(0)
        self.set_smplrt_div(0)
        self.set_dlpf_cfg(0)


    def get_raw_data(self, addr):
        all_data = self.bus.read_i2c_block_data(self.address, addr, 6)
        raw_x = _convert_to_signed((all_data[0] << 8) | all_data[1])
        raw_y = _convert_to_signed((all_data[2] << 8) | all_data[3])
        raw_z = _convert_to_signed((all_data[4] << 8) | all_data[5])
        return raw_x, raw_y, raw_z

    def get_raw_accel_data(self):
        accel_x, accel_y, accel_z = self.get_raw_data(_MPU6050_ACCEL_OUT_X)
        return accel_x, accel_y, accel_z

    def get_raw_gyro_data(self):
        gyro_x, gyro_y, gyro_z = self.get_raw_data(_MPU6050_GYRO_OUT_X)
        return gyro_x, gyro_y, gyro_z

    def get_raw_data(self):
        all_data = self.bus.read_i2c_block_data(self.address, _MPU6050_ACCEL_OUT_X, 14)
        
        raw_accel_x = _convert_to_signed((all_data[0] << 8) | all_data[1])
        raw_accel_y = _convert_to_signed((all_data[2] << 8) | all_data[3])
        raw_accel_z = _convert_to_signed((all_data[4] << 8) | all_data[5])

        raw_gyro_x = _convert_to_signed((all_data[8] << 8) | all_data[9])
        raw_gyro_y = _convert_to_signed((all_data[10] << 8) | all_data[11])
        raw_gyro_z = _convert_to_signed((all_data[12] << 8) | all_data[13])

        return raw_accel_x, raw_accel_y, raw_accel_z, raw_gyro_x, raw_gyro_y, raw_gyro_z

    def MPU_ReadData(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.get_raw_data()

        accel_x = (accel_x / self.ACCEL_SCALE)- self.AX_OFFSET
        accel_y = (accel_y / self.ACCEL_SCALE)- self.AY_OFFSET
        accel_z = (accel_z / self.ACCEL_SCALE)- self.AZ_OFFSET

        gyro_x = (gyro_x / self.GYRO_SCALE) - self.GX_OFFSET
        gyro_y = (gyro_y / self.GYRO_SCALE) - self.GY_OFFSET
        gyro_z = (gyro_z / self.GYRO_SCALE) - self.GZ_OFFSET

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def set_register(self, address, value):
        if address is None or value is None:
            raise ValueError("Address and value need to be given!")
        self.bus.write_byte_data(self.address, address, value)

    def read_register(self, address):
        if address is None:
            raise ValueError("Address must be given!")
        value = self.bus.read_byte_data(self.address, address)
        return value

    def calibrate_sensor(self, time_dur=5):
        self.reset_mpu()
        self.set_register(_MPU6050_PWR_MGMT_1, 0x01)
        
        self.set_dlpf_cfg(1)
        
        self.AX_OFFSET = 0.0
        self.AY_OFFSET = 0.0
        self.AZ_OFFSET = 0.0
        self.GX_OFFSET = 0.0
        self.GY_OFFSET = 0.0
        self.GZ_OFFSET = 0.0

        counter = 0
        
        timer = time.time()
        while ((time.time() - timer) < time_dur):
            
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.get_raw_data()
            counter += 1
            
            self.AX_OFFSET += accel_x
            self.AY_OFFSET += accel_y
            self.AZ_OFFSET += accel_z
            self.GX_OFFSET += gyro_x
            self.GY_OFFSET += gyro_y
            self.GZ_OFFSET += gyro_z
            
            if (counter % 100) == 0:
                print (f'Counter: {counter}')

        self.AX_OFFSET /= counter
        self.AY_OFFSET /= counter
        self.AZ_OFFSET /= counter
        self.GX_OFFSET /= counter
        self.GY_OFFSET /= counter
        self.GZ_OFFSET /= counter

        # Remove gravity from az readings
        if self.AZ_OFFSET > 0:
            self.AZ_OFFSET -= self.ACCEL_SCALE
        else:
            self.AZ_OFFSET += self.ACCEL_SCALE

        self.AX_OFFSET /= self.ACCEL_SCALE
        self.AY_OFFSET /= self.ACCEL_SCALE
        self.AZ_OFFSET /= self.ACCEL_SCALE
        self.GX_OFFSET /= self.GYRO_SCALE
        self.GY_OFFSET /= self.GYRO_SCALE
        self.GZ_OFFSET /= self.GYRO_SCALE
        
        print("Setting offsets to: ")
        print(f'OFFSET AX {self.AX_OFFSET}, AY {self.AY_OFFSET}, AZ {self.AZ_OFFSET}')
        print(f'OFFSET GX {self.GX_OFFSET}, GY {self.GY_OFFSET}, GZ {self.GZ_OFFSET}')

    def calculate_gyro_drift(self):
        self.GYRO_DRIFT_X = 0.0
        self.GYRO_DRIFT_Y = 0.0
        self.GYRO_DRIFT_Z = 0.0
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.MPU_ReadData()
        last_x = gyro_x
        last_y = gyro_y
        last_z = gyro_z

        counter = 0
        last_time = time.perf_counter()

        timer = time.time()
        while ((time.time() - timer) < 10):
            starting_time = time.perf_counter()
            dt = starting_time - last_time
            last_time = starting_time

            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.MPU_ReadData()
            counter += 1
            
            self.GYRO_DRIFT_X += gyro_x - last_x
            self.GYRO_DRIFT_Y += gyro_y - last_y
            self.GYRO_DRIFT_Z += gyro_z - last_z

            last_x = gyro_x
            last_y = gyro_y
            last_z = gyro_z
            print (f'Counter: {counter}')

        self.GYRO_DRIFT_X = self.GYRO_DRIFT_X/counter
        self.GYRO_DRIFT_Y = self.GYRO_DRIFT_Y/counter
        self.GYRO_DRIFT_Z = self.GYRO_DRIFT_Z/counter
        
        print("Setting drifts to: ")
        print(f'DRIFTS GX {self.GYRO_DRIFT_X}, GY {self.GYRO_DRIFT_Y}, GZ {self.GYRO_DRIFT_Z}')

    def set_smplrt_div(self, value: int = 0):
        if value < 0 or value > 255:
            raise ValueError("SMPLRT_DIV value must be between 0 and 255")
        self.set_register(_MPU6050_SMPLRT_DIV, value)

    def get_smplrt_div(self):
        value = self.read_register(_MPU6050_SMPLRT_DIV) & 0xFF
        return value

    def set_dlpf_cfg(self, value : int = 0):
        if value < 0 or value > 7:
            raise ValueError("DLPF_CFG value must be between 0 and 7")
        self.set_register(_MPU6050_CONFIG, value)
    
    def get_dlpf_cfg(self):
        value = self.read_register(_MPU6050_CONFIG) & 0x07
        return value

    def set_gyro_config(self, value: int = 0):
        if value < 0 or value > 248:
            raise ValueError("Gyro cfg value must be between 8 and 248")
        value = (value & ~0x07)
        range = value & 0x18
        match range:
            case 0x00:
                self.GYRO_SCALE = 131
            case 0x08:
                self.GYRO_SCALE = 65.5
            case 0x10:
                self.GYRO_SCALE = 32.8
            case 0x18:
                self.GYRO_SCALE = 16.4
            case _:
                pass
        # print(f'Gyro scale: {self.GYRO_SCALE}')
        self.set_register(_MPU6050_GYRO_CONFIG, value)

    def get_gyro_config(self):
        value = self.read_register(_MPU6050_GYRO_CONFIG) & 0xFF
        return value

    def set_accel_config(self, value: int = 0):
        if value < 0 or value > 248:
            raise ValueError("Accel cfg value must be between 8 and 248")
        value = (value & ~0x07)
        range = value & 0x18
        match range:
            case 0:
                self.ACCEL_SCALE = 16384
            case 8:
                self.ACCEL_SCALE = 8192
            case 16:
                self.ACCEL_SCALE = 4096
            case 24:
                self.ACCEL_SCALE = 2048
            case _:
                pass
        # print(f'Accel scale: {self.ACCEL_SCALE}')
        self.set_register(_MPU6050_ACCEL_CONFIG, value)

    def get_accel_config(self):
        value = self.read_register(_MPU6050_ACCEL_CONFIG) & 0xFF
        return value

    def set_accel_offset(self, ax_offset=None, ay_offset=None, az_offset=None):
        if ax_offset is not None:
            self.AX_OFFSET = ax_offset
        if ay_offset is not None:
            self.AY_OFFSET = ay_offset
        if ay_offset is not None:
            self.AZ_OFFSET = az_offset
        print(f'Set offsets to x: {self.AX_OFFSET}, y: {self.AY_OFFSET}, z: {self.AZ_OFFSET}')

    def get_accel_offset(self):
        return self.AX_OFFSET, self.AY_OFFSET, self.AZ_OFFSET

    def set_gyro_offset(self, gx_offset=None, gy_offset=None, gz_offset=None):
        if gx_offset is not None:
            self.GX_OFFSET = gx_offset
        if gy_offset is not None:
            self.GY_OFFSET = gy_offset
        if gy_offset is not None:
            self.GZ_OFFSET = gz_offset
        print(f'Set offsets to x: {self.GX_OFFSET}, y: {self.GY_OFFSET}, z: {self.GZ_OFFSET}')

    def get_gyro_offset(self):
        return self.GX_OFFSET, self.GY_OFFSET, self.GZ_OFFSET
    
    def reset_mpu(self):
        self.set_register(_MPU6050_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

def _convert_to_signed(value):
    if value > 32768:
        value -= 65536
    return value

def get_x_rotation(x, y, z):
    radians = math.atan2(x, math.sqrt(y*y + z*z))
    return -math.degrees(radians)

def get_y_rotation(x, y, z):
    radians = math.atan2(y, math.sqrt(x*x + z*z))
    return math.degrees(radians)


if __name__ == "__main__":

    bus = SMBus(1)
    mpu = MyMPU6050(bus)

    print("Calibrating sensor, do not move the system")
    mpu.calibrate_sensor(5)

    mpu.set_dlpf_cfg(6)

    # mpu.calculate_gyro_drift()
    # print(mpu.GYRO_DRIFT_X, mpu.GYRO_DRIFT_Y, mpu.GYRO_DRIFT_Z)
    # timer = time.time()
    # while ((time.time() - timer) < 10):
    #     print(mpu.get_raw_data())