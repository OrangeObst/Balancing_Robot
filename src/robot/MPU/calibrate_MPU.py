import argparse
from configparser import ConfigParser
import os
from smbus2 import SMBus
from MyMpu6050 import MyMPU6050

def store_values_in_ini(ax, ay, az, gx, gy, gz, config_file):
    config = ConfigParser()
    config.read(config_file)
    
    if 'MPU' not in config:
        config['MPU'] = {}
    
    config['MPU']['AX_OFFSET'] = str(ax)
    config['MPU']['AY_OFFSET'] = str(ay)
    config['MPU']['AZ_OFFSET'] = str(az)
    config['MPU']['GX_OFFSET'] = str(gx)
    config['MPU']['GY_OFFSET'] = str(gy)
    config['MPU']['GZ_OFFSET'] = str(gz)
    
    with open(config_file, 'w') as configfile:
        config.write(configfile)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Calibrate the MPU6050 sensor.')
    parser.add_argument('calibration_value', type=int, help='Integer value for calibration')
    args = parser.parse_args()

    bus = SMBus(1)
    mpu = MyMPU6050(bus)
    
    ax, ay, az, gx, gy, gz = mpu.calibrate_sensor(args.calibration_value)
    
    script_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    config_file_path = os.path.join(script_dir, 'settings.ini')
    
    store_values_in_ini(ax, ay, az, gx, gy, gz, config_file_path)