import time
from smbus2 import SMBus

'------------------ MPU REGISTERS ------------------'

_MPU6050_DEFAULT_ADDRESS    = 0x68 # MPU6050 default i2c address w/ AD0 low
_MPU6050_DEVICE_ID          = 0x68 # The correct MPU6050_WHO_AM_I value
_MPU6050_SELF_TEST_X        = 0x0D # Self test factory calibrated values register
_MPU6050_SELF_TEST_Y        = 0x0E # Self test factory calibrated values register
_MPU6050_SELF_TEST_Z        = 0x0F # Self test factory calibrated values register
_MPU6050_SELF_TEST_A        = 0x10 # Self test factory calibrated values register
_MPU6050_SMPLRT_DIV         = 0x19 # sample rate divisor register
_MPU6050_CONFIG             = 0x1A # General configuration register
_MPU6050_GYRO_CONFIG        = 0x1B # Gyro specfic configuration register
_MPU6050_ACCEL_CONFIG       = 0x1C # Accelerometer specific configration register
_MPU6050_FIFO_EN            = 0x23 # Configure FIFO register
_MPU6050_INT_PIN_CONFIG     = 0x37 # Interrupt pin configuration register
_MPU6050_INT_STATUS         = 0x3A # Interrupt Status register
_MPU6050_ACCEL_OUT_X        = 0x3B # base address for sensor data reads
_MPU6050_ACCEL_OUT_Y        = 0x3D
_MPU6050_ACCEL_OUT_Z        = 0x3F
_MPU6050_TEMP_OUT           = 0x41 # Temperature data high byte register
_MPU6050_GYRO_OUT_X         = 0x43 # base address for sensor data reads
_MPU6050_GYRO_OUT_Y         = 0x45
_MPU6050_GYRO_OUT_Z         = 0x47
_MPU6050_SIG_PATH_RESET     = 0x68 # register to reset sensor signal paths
_MPU6050_USER_CTRL          = 0x6A # FIFO and I2C Master control register
_MPU6050_PWR_MGMT_1         = 0x6B # Primary power/sleep control register
_MPU6050_PWR_MGMT_2         = 0x6C # Secondary power/sleep control register
_MPU6050_FIFO_COUNTH        = 0x72 # Read FIFO count high
_MPU6050_FIFO_COUNTL        = 0x73 # Read FIFO count low
_MPU6050_FIFO_R_W           = 0x74 # Read FIFO values
_MPU6050_WHO_AM_I           = 0x75 # Divice ID register


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

    def _read_sensor_data(self, start_addr, length):
        return self.bus.read_i2c_block_data(self.address, start_addr, length)
    
    def _extract_raw_data(self, data, start_index):
        return tuple(_convert_to_signed((data[i] << 8) | data[i + 1]) for i in range(start_index, start_index + 6, 2))

    def get_raw_accel_data(self) -> tuple[float, float, float]:
        data = self._read_sensor_data(_MPU6050_ACCEL_OUT_X, 6)
        return self._extract_raw_data(data, 0)

    def get_raw_gyro_data(self) -> tuple[float, float, float]:
        data = self._read_sensor_data(_MPU6050_GYRO_OUT_X, 6)
        return self._extract_raw_data(data, 0)

    def get_raw_data(self) -> tuple[float, float, float, float, float, float]:
        data = self._read_sensor_data(_MPU6050_ACCEL_OUT_X, 14)
        raw_accel = self._extract_raw_data(data, 0)
        raw_gyro = self._extract_raw_data(data, 8)
        return raw_accel + raw_gyro

    def get_all_data(self) -> tuple[float, float, float, float, float, float]:
        raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = self.get_raw_data()
        ax = (raw_ax - self.AX_OFFSET) / self.ACCEL_SCALE
        ay = (raw_ay - self.AY_OFFSET) / self.ACCEL_SCALE
        az = (raw_az - self.AZ_OFFSET) / self.ACCEL_SCALE
        gx = (raw_gx - self.GX_OFFSET) / self.GYRO_SCALE
        gy = (raw_gy - self.GY_OFFSET) / self.GYRO_SCALE
        gz = (raw_gz - self.GZ_OFFSET) / self.GYRO_SCALE
        return ax, ay, az, gx, gy, gz

    def set_register(self, address, value):
        if address is None or value is None:
            raise ValueError("Address and value need to be given!")
        self.bus.write_byte_data(self.address, address, value)

    def read_register(self, address) -> int:
        if address is None:
            raise ValueError("Address must be given!")
        value = self.bus.read_byte_data(self.address, address)
        return value

    def calibrate_sensor(self, duration=3) -> tuple[float, float, float, float, float, float]:
        print("Calibrating sensor, do not move the system")
        # self.reset_mpu()
        # self.set_register(_MPU6050_PWR_MGMT_1, 0x01)
        
        self.AX_OFFSET = 0.0
        self.AY_OFFSET = 0.0
        self.AZ_OFFSET = 0.0
        self.GX_OFFSET = 0.0
        self.GY_OFFSET = 0.0
        self.GZ_OFFSET = 0.0

        counter = 0
        timer = time.time()
        while ((time.time() - timer) < duration):
            
            ax, ay, az, gx, gy, gz = self.get_raw_data()
            counter += 1
            
            self.AX_OFFSET += ax
            self.AY_OFFSET += ay
            self.AZ_OFFSET += az
            self.GX_OFFSET += gx
            self.GY_OFFSET += gy
            self.GZ_OFFSET += gz
            
            if (counter % 100) == 0:
                print (f'Counter: {counter}')
            time.sleep(0.01)

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
        
        print("Setting offsets to: ")
        print(f'OFFSET AX: {self.AX_OFFSET:.6f}, AY: {self.AY_OFFSET:.6f}, AZ: {self.AZ_OFFSET:.6f}')
        print(f'OFFSET GX: {self.GX_OFFSET:.6f}, GY: {self.GY_OFFSET:.6f}, GZ: {self.GZ_OFFSET:.6f}')
        return self.AX_OFFSET, self.AY_OFFSET, self.AZ_OFFSET, self.GX_OFFSET, self.GY_OFFSET, self.GZ_OFFSET

    def set_smplrt_div(self, bit_mask: int = 0x00):
        if bit_mask < 0 or bit_mask > 255:
            raise ValueError("SMPLRT_DIV value must be between 0 and 255")
        self.set_register(_MPU6050_SMPLRT_DIV, bit_mask)

    def get_smplrt_div(self) -> int:
        bit_mask = self.read_register(_MPU6050_SMPLRT_DIV) & 0xFF
        return bit_mask

    def set_dlpf_cfg(self, bit_mask: int = 0x00):
        '''
        DLPF_CFG  |         Accelerometer        |               Gyroscope
                  |          (Fs = 1kHz)         |
                  |  Bandwidth(Hz)   Delay(ms)   |   Bandwidth(Hz)   Delay(ms)  Fs(kHz)
            0     |      260             0       |         256           0.98      8
            1     |      184             2.0     |         188           1.9       1
            2     |      94              3.0     |          98           2.8       1
            3     |      44              4.9     |          42           4.8       1
            4     |      21              8.5     |          20           8.3       1
            5     |      10             13.8     |          10           13.4      1
            6     |      5               19.0    |           5           18.6      1
            7     |          RESERVED            |              RESERVED           8
        '''
        if bit_mask > 0x6:
            raise ValueError("DLPF_CFG value must be between 0x0 and 0x6")
        self.set_register(_MPU6050_CONFIG, bit_mask)
    
    def get_dlpf_cfg(self) -> int:
        bit_mask = self.read_register(_MPU6050_CONFIG) & 0x07
        return bit_mask

    def set_gyro_config(self, bit_mask: int = 0x00):
        if bit_mask not in [0x00, 0x08, 0x10, 0x18]:
            raise ValueError("Gyro cfg value should be one of 0x00, 0x08, 0x10, or 0x18")

        if bit_mask == 0x00:
            self.GYRO_SCALE = 131
        elif bit_mask == 0x08:
            self.GYRO_SCALE = 65.5
        elif bit_mask == 0x10:
            self.GYRO_SCALE = 32.8
        elif bit_mask == 0x18:
            self.GYRO_SCALE = 16.4
        self.set_register(_MPU6050_GYRO_CONFIG, bit_mask)

    def get_gyro_config(self) -> int:
        bit_mask = self.read_register(_MPU6050_GYRO_CONFIG) & 0xFF
        return bit_mask

    def set_accel_config(self, bit_mask: int = 0x00):
        if bit_mask not in [0x00, 0x08, 0x10, 0x18]:
            raise ValueError("Accel cfg value should be one of 0x00, 0x08, 0x10, or 0x18")

        if bit_mask == 0x00:
            self.ACCEL_SCALE = 16384
        elif bit_mask == 0x08:
            self.ACCEL_SCALE = 8192
        elif bit_mask == 0x10:
            self.ACCEL_SCALE = 4096
        elif bit_mask == 0x18:
            self.ACCEL_SCALE = 2048
        self.set_register(_MPU6050_ACCEL_CONFIG, bit_mask)

    def get_accel_config(self) -> int:
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

    def get_accel_offset(self) -> tuple[float, float, float]:
        return self.AX_OFFSET, self.AY_OFFSET, self.AZ_OFFSET

    def set_gyro_offset(self, gx_offset=None, gy_offset=None, gz_offset=None):
        if gx_offset is not None:
            self.GX_OFFSET = gx_offset
        if gy_offset is not None:
            self.GY_OFFSET = gy_offset
        if gy_offset is not None:
            self.GZ_OFFSET = gz_offset
        print(f'Set offsets to x: {self.GX_OFFSET}, y: {self.GY_OFFSET}, z: {self.GZ_OFFSET}')

    def get_gyro_offset(self) -> tuple[float, float, float]:
        return self.GX_OFFSET, self.GY_OFFSET, self.GZ_OFFSET
    
    def get_all_offsets(self) -> tuple [float, float, float, float, float, float]:
        return self.get_accel_offset() + self.get_gyro_offset()
    
    def enable_fifo_reg(self):
        bit_mask = self.read_register(_MPU6050_USER_CTRL)
        bit_mask = bit_mask | 0x40
        self.set_register(_MPU6050_USER_CTRL, bit_mask)

    def disable_fifo_reg(self):
        bit_mask = self.read_register(_MPU6050_USER_CTRL)
        bit_mask = bit_mask & ~0x40
        self.set_register(_MPU6050_USER_CTRL, bit_mask)

    def reset_fifo(self):
        bit_mask = self.read_register(_MPU6050_USER_CTRL)
        bit_mask = (bit_mask & ~0x40) | 0x04
        self.set_register(_MPU6050_USER_CTRL, bit_mask)
        time.sleep(0.01)
    
    def configure_fifo_reg(self, bits=0x78):
        """
        Enables the First-In-First-Out (FIFO) operation for the MPU6050 sensor.
    
        Parameters:
        bits (int): Bitmask to configure which sensor data is written to the FIFO.
                    Default: 0x78 (Accelerometer and Gyroscope data to FIFO)
                    Note: Consult the MPU6050 datasheet for bitmask configuration details.
    
        Action:
        Sets the MPU6050 FIFO_ENABLE register with the provided bitmask.
        """
        self.set_register(_MPU6050_FIFO_EN, bits)

    def get_fifo_count(self):
        fifo_count_h = self.read_register(_MPU6050_FIFO_COUNTH)
        fifo_count_l = self.read_register(_MPU6050_FIFO_COUNTL)
        fifo_count = (fifo_count_h << 8) | fifo_count_l
        return fifo_count
    
    def get_fifo_oflow_int(self):
        flag = self.read_register(_MPU6050_INT_STATUS) & 0x10
        return flag

    def get_fifo_buffer(self):
        fifo_count = self.get_fifo_count()
        packets = []

        if fifo_count < 12:
            return None
        else:
            while fifo_count >= 12:
                data = bus.read_i2c_block_data(self.address,_MPU6050_FIFO_R_W, 12)

                ax = ((_convert_to_signed((data[0] << 8) | data[1])) / self.ACCEL_SCALE) - self.AX_OFFSET
                ay = ((_convert_to_signed((data[2] << 8) | data[3])) / self.ACCEL_SCALE) - self.AY_OFFSET
                az = ((_convert_to_signed((data[4] << 8) | data[5])) / self.ACCEL_SCALE) - self.AZ_OFFSET
                gx = ((_convert_to_signed((data[6] << 8) | data[7])) / self.GYRO_SCALE) - self.GX_OFFSET
                gy = ((_convert_to_signed((data[8] << 8) | data[9])) / self.GYRO_SCALE) - self.GY_OFFSET
                gz = ((_convert_to_signed((data[10] << 8) | data[11])) / self.GYRO_SCALE) - self.GZ_OFFSET

                fifo_count -= 12
                packets.append([ax, ay, az, gx, gy, gz])
            
            return packets
        
    def optimize_sample_settings(self, dt):
        '''
        This function tries to optimize the sample settings of the MPU6050 based on the desired time step (dt).
        It selects the best DLPF configuration and calculates the sample rate divider to achieve the target sample rate.
        '''
        dlpf_configs = {
            0: (0.0, 0.98),  # Delay in ms for Accel, Gyro respectively
            1: (2.0, 1.9),
            2: (3.0, 2.8),
            3: (4.9, 4.8),
            4: (8.5, 8.3),
            5: (13.8, 13.4),
            6: (19.0, 18.6),
        }

        # Select the DLPF config with the highest delay less than or equal to dt
        best_dlpf = max([k for k, v in dlpf_configs.items() if max(v) <= dt * 1000])
        print(f"Selected DLPF: {best_dlpf} (Delay <= {dt*1000:.2f} ms)")
        self.set_dlpf_cfg(best_dlpf)

        # Sample Rate Divider Calculation (MPU6050 default sample rate: 8 kHz)
        gyro_output_rate = 8000 if best_dlpf == 0 else 1000
        sample_rate_hz = 1 / dt
        sample_rate_divider = int((gyro_output_rate / sample_rate_hz) - 1)
        print(f"Sample Rate Divider: {sample_rate_divider} (Target Sample Rate: {sample_rate_hz:.2f} Hz)")

        # Set Sample Rate Divider
        self.set_smplrt_div(sample_rate_divider)
    
    def reset_mpu(self):
        self.set_register(_MPU6050_PWR_MGMT_1, 0x80)
        time.sleep(0.1)


def _convert_to_signed(value):
    return value if value < 0x8000 else value - 0x10000


if __name__ == "__main__":
    dt = 0.02
    previous_pitch = 0.0
    alpha = 0.98
    timer = time.time()
    counter = 0

    bus = SMBus(1)
    mpu = MyMPU6050(bus)

    mpu.calibrate_sensor(5)

    # mpu.reset_fifo()
    # mpu.enable_fifo_reg()
    # mpu.configure_fifo_reg()
    # mpu.set_dlpf_cfg(0x2
    # mpu.optimize_sample_settings(dt)
    # mpu.calculate_gyro_drift()
    # print(mpu.GYRO_DRIFT_X, mpu.GYRO_DRIFT_Y, mpu.GYRO_DRIFT_Z)

    # mpu.set_accel_offset(0.059397, -0.019336, 0.104918)
    # mpu.set_gyro_offset(0.180059, 0.101374, 0.241004)



    # from math import atan2, sqrt, degrees
    
    # while ((time.time() - timer) < 5):
    # while True:
        # loop_start = time.time()
    #     # data_list = mpu.get_fifo_buffer()
        # data = mpu.get_all_data()
        # print(mpu.get_raw_data())
    #     # if data_list is not None:
    #         # for data in data_list:
    #     pitch_from_acceleration = degrees(atan2(data[0], -data[2]))
    #     pitch_gyro_integration = previous_pitch + data[4] * dt

    #     previous_pitch = alpha * pitch_gyro_integration + (1 - alpha) * pitch_from_acceleration
    #     counter += 1
        # print(f'0: {data[0]:8.4f}, 1: {data[1]:8.4f}, 2: {data[2]:8.4f}, 3: {data[3]:8.4f}, 4: {data[4]:8.4f}, 5: {data[5]:8.4f}, Pitch: {previous_pitch:6.4f}')
        
        # loop_end = time.time()
        # loop_duration = loop_end - loop_start

        # sleep_time = max(0, dt - loop_duration)
        # time.sleep(0.1)

    # print(counter)