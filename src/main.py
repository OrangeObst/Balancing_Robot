from robot.motors.motor_controller import MotorController
from robot.robot import BalancingRobot
from util.data_collector import DataCollector
from robot.MPU.MyMpu6050 import MyMPU6050
from robot.pid_controller import PidController
from robot.motors.stepper_motor import Stepper
from smbus2 import SMBus
from configparser import ConfigParser
from threading import Event
import signal
import os

config = ConfigParser()
script_dir = os.path.dirname(os.path.abspath(__file__))
config_file_path = os.path.join(script_dir, 'settings.ini')
config.read(config_file_path)


# Position PID constants
USE_POS_PID = config.getboolean('Position_PID', 'use_pos_pid')                      # De-/activate position PID controller

# Time settings
DELAY = config.getfloat('Time', 'delay')                                            # Updatetime delay
TIMER = config.getfloat('Time', 'timer')                                            # Runtime in seconds

# Motor settings
USE_MOTORS = config.getboolean('Motor', 'use_motors')                               # De-/activate motors
MICROSTEPS = config.getfloat('Motor', 'microsteps')                                 # Stepper motor HAT microstep setting
USE_PROCESSED_MOTORS = config.getboolean('Motor', 'use_processed_motors')           # Use processed motors (MultiprocessingStepper)

# MPU settings
SAMPLE_TIME = config.getfloat('MPU', 'sample_time')                                 # MPUaverager sample time => DELAY / SAMPLE_TIME
CALIBRATE = config.getboolean('MPU', 'calibrate')                                   # True: MPU calibrates before every start, False: uses hardcoded offset

# Data logging
LOG_DATA = config.getboolean('Logging', 'log_data')                                 # De-/activate data logging


if __name__ == "__main__":
    # ----- MPU -----
    bus = SMBus(1)
    mpu = MyMPU6050(bus)
    ax_offset = config.getfloat('MPU', 'ax_offset')
    ay_offset = config.getfloat('MPU', 'ay_offset')
    az_offset = config.getfloat('MPU', 'az_offset')
    gx_offset = config.getfloat('MPU', 'gx_offset')
    gy_offset = config.getfloat('MPU', 'gy_offset')
    gz_offset = config.getfloat('MPU', 'gz_offset')
    mpu.set_accel_offset(ax_offset, ay_offset, az_offset)
    mpu.set_gyro_offset(gx_offset, gy_offset, gz_offset)
    sample_time = DELAY
    mpu.optimize_sample_settings(sample_time)

    # ----- PID -----
    min_velocity = -100
    max_velocity = 100
    angle_setpoint = 0.0
    ap = config.getfloat('Angle_PID', 'ap')
    ai = config.getfloat('Angle_PID', 'ai')
    ad = config.getfloat('Angle_PID', 'ad')
    derivative_fiilter = config.getfloat('Angle_PID', 'derivative_filter')
    position_setpoint = 0.0
    min_angle = -25.0
    max_angle = 25.0
    pp = config.getfloat('Position_PID', 'pp')
    pi = config.getfloat('Position_PID', 'pi')
    pd = config.getfloat('Position_PID', 'pd')
    delay = DELAY

    angle_pid = PidController(ap, ai, ad, min_velocity, max_velocity, setpoint=angle_setpoint, alpha=0.5, deadband=0.4)
    pos_pid = PidController(pp, pi, pd, min_angle, max_angle, position_setpoint)
    
    # ----- Motor -----
    spr = 200 * MICROSTEPS
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), microsteps=8)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), microsteps=8, invert_direction=True)
    motor_controller = MotorController(left_motor, right_motor, use_motors=USE_MOTORS, processed=USE_PROCESSED_MOTORS)
    
    # ----- Logging -----
    data_collector = DataCollector()
    
    stop_event = Event()

    # ----- Robot -----
    robot = BalancingRobot(
        motor_controller = motor_controller,
        mpu = mpu,
        angle_pid = angle_pid,
        pos_pid = pos_pid,
        data_collector = data_collector,
        stop_event = stop_event
    )

    def _shutdown(signum, frame):
        print('Signal received, shutting down')
        try:
            if robot.running:
                robot.stop()
            robot.shutdown()
        except Exception as e:
            print('Error during robot shutdown', e)
        stop_event.set()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    stop_event.wait()