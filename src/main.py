from robot.robot import BalancingRobot
from util.data_collector import DataCollector
from util.plot_graphs import Plotter
from robot.MPU.MyMpu6050 import MyMPU6050
from robot.pid_controller import PID_Controller
from robot.stepper_motor import Stepper
from robot.threaded_motors import ThreadedStepper
# from codetiming import Timer
from time import time
from smbus2 import SMBus
from configparser import ConfigParser

config = ConfigParser()
config.read('/home/newPi/Desktop/Balancing_Robot/src/settings.ini')

# Angle PID constants
AP = config.getfloat('Angle_PID', 'AP')                 # 15
AI = config.getfloat('Angle_PID', 'AI')                 # 0.01
AD = config.getfloat('Angle_PID', 'AD')                 # 0.15

# Position PID constants
PP = config.getfloat('Position_PID', 'PP')              # 0.0005
PI = config.getfloat('Position_PID', 'PI')              # 0.0
PD = config.getfloat('Position_PID', 'PD')              # 0.0006
USE_POS_PID = config.getboolean('Position_PID', 'USE_POS_PID')                      # De-/activate position PID controller
MAX_TARGET_ANGLE = config.getfloat('Position_PID', 'MAX_TARGET_ANGLE')              # Max output for position PID controller
FILTER_TARGET_ANGLE = config.getboolean('Position_PID', 'FILTER_TARGET_ANGLE')      # De-/activate filtering for target angle, could reduce instability
                                                                                    # Has to be false if USE_POS_PID is False

# Speed PID constants
SP = config.getfloat('Speed_PID', 'SP')
SI = config.getfloat('Speed_PID', 'SI')
SD = config.getfloat('Speed_PID', 'SD')
USE_SPEED_PID = config.getboolean('Speed_PID', 'USE_SPEED_PID')

# Time settings
DELAY = config.getfloat('Time', 'DELAY')                                            # Updatetime delay
TIMER = config.getfloat('Time', 'TIMER')                                            # Runtime in seconds

# Motor settings
USE_MOTORS = config.getboolean('Motor', 'USE_MOTORS')                               # De-/activate motors
USE_THREADED_MOTORS = config.getboolean('Motor', 'USE_THREADED_MOTORS')             # Threaded motors
MICROSTEPS = config.getfloat('Motor', 'MICROSTEPS')                                 # Stepper motor HAT microstep setting

# MPU settings
COMPLEMENTARY_ALPHA = config.getfloat('MPU', 'COMPLEMENTARY_ALPHA')                 # Complementary filter for the accelerometer and gyroscope (MPU6050)
FILTER_ACCEL_ANGLE = config.getboolean('MPU', 'FILTER_ACCEL_ANGLE')                 # De-/activate filtering for acceleration angle, increases reaction time
AVERAGE_MPU_VALUES = config.getboolean('MPU', 'AVERAGE_MPU_VALUES')                 # De-/activate averaging for MPU samples over SAMPLE_TIME
SAMPLE_TIME = config.getfloat('MPU', 'SAMPLE_TIME')                                 # MPUaverager sample time => DELAY / SAMPLE_TIME
CALIBRATE = config.getboolean('MPU', 'CALIBRATE')                                   # True: MPU calibrates before every start, False: uses hardcoded offset

# Data logging
LOG_DATA = config.getboolean('Logging', 'LOG_DATA')                                 # De-/activate data logging


if __name__ == "__main__":

    # ----- MPU -----
    bus = SMBus(1)
    mpu = MyMPU6050(bus)
    if CALIBRATE:
        mpu.calibrate_sensor(2)
    else:
        mpu.set_accel_offset(0.081267, -0.019716, 0.101106)
        mpu.set_gyro_offset(0.112204, 0.124888, 0.493542)

    sample_time = SAMPLE_TIME if AVERAGE_MPU_VALUES else DELAY
    mpu.optimize_sample_settings(sample_time)

    # ----- PID -----
    min_velocity = -100
    max_velocity = 100
    angle_setpoint = 0.0
    ap = AP
    ai = AI
    ad = AD
    position_setpoint = 0.0
    min_angle = -25.0
    max_angle = 25.0
    pp = PP
    pi = PI
    pd = PD
    speed_setpoint = 0.0
    sp = SP
    si = SI
    sd = SD
    delay = DELAY

    angle_pid = PID_Controller(ap, ai, ad, min_velocity, max_velocity, setpoint=angle_setpoint, alpha=0.5, deadband=0.4)
    pos_pid = PID_Controller(pp, pi, pd, min_angle, max_angle, position_setpoint)
    speed_pid = PID_Controller(sp, si, sd, min_angle, max_angle, speed_setpoint)
    
    # ----- Motor -----
    spr = 200 * MICROSTEPS
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), microsteps=8)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), microsteps=8, invert_direction=True)
    if USE_MOTORS:
        left_motor.start()
        right_motor.start()

    # ----- Logging -----
    data_collector = DataCollector()
    # data_collector.log_decorator_enabled = LOG_DATA
    
    # ----- Robot -----
    robot = BalancingRobot(
        left_motor = left_motor,
        right_motor = right_motor,
        mpu = mpu,
        pid1 = pos_pid,
        pid2 = angle_pid,
        pid3 = speed_pid,
        data_collector = data_collector
    )


    timer = time() + TIMER
    try:
        while time() < timer:
            robot.loop()
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        robot.shutdown()
        
        print("Exiting ...")
        print(f'Counter: {robot.counter}')


    if LOG_DATA:
        angle_pid_const = [
            ap,
            ai,
            ad
        ]
        pos_pid_const = [
            pp,
            pi,
            pd
        ]
        speed_pid_const = [
            sp,
            si,
            sd
        ]

        # collected_data = data_collector.get_data()
        collected_data = data_collector.get_collected_data()

        plotter = Plotter(angle_pid_const, pos_pid_const, speed_pid_const)
        plotter.plot_measurements('Angles [°]', {'Robot angle': collected_data['angle'], 'Target angle': collected_data['pos_pid_terms']['output']}, TIMER, 'Steps', {'Steps': collected_data['avg_steps']})
        plotter.plot_measurements('Angles [°]', {'Robot angle': collected_data['angle']}, TIMER, 'Speed', {'Speed': collected_data['angle_pid_terms']['output']}, name='Angle_to_Speed')        
        plotter.plot_measurements('Angles [°]', {'Accel angle': collected_data['accel_angle'], 'Gyro angle': collected_data['gyro_angle']}, TIMER, name='Accel_Gyro_angles')
        plotter.plot_measurements('Accel', {'ax': collected_data['ax'], 'ay': collected_data['ay'], 'az': collected_data['az']}, TIMER, name="Accel_Data")
        plotter.plot_measurements('Gyro', {'gx': collected_data['gx'], 'gy': collected_data['gy'], 'gz': collected_data['gz']}, TIMER, name="Gyro_Data")
        plotter.plot_measurements('PD values', {'P terms': collected_data['angle_pid_terms']['p_terms'], 'D terms': collected_data['angle_pid_terms']['d_terms'], 'Output': collected_data['angle_pid_terms']['output']}, TIMER, name='PD graph')
        plotter.subplot_p_i_d_values('Angle', collected_data['angle_pid_terms'], TIMER, 100, unified_y_limit=False, name='Angle_PID_Terms')
        if USE_POS_PID:
            plotter.subplot_p_i_d_values('Position', collected_data['pos_pid_terms'], TIMER, 100, unified_y_limit=False, name='Position_PID_Terms')
        if USE_SPEED_PID:
            plotter.subplot_p_i_d_values('Speed', collected_data['speed_pid_terms'], TIMER, 100, unified_y_limit=False, name='Speed_PID_Terms')
        
        plotter.plot_angles([[collected_data['angle'],'Robot angle'], [collected_data['pos_pid_terms']['output'],'Target angle']], TIMER,  name='Angle_to_target_angle')
        # plotter.plot_angles([[collected_data['accel_angles'],'Winkel aus Beschleunigungsdaten'], [collected_data['f_accel_angles'], 'Gefilterter Winkel'], [collected_data['gyro_angles'],'Winkel aus Gyroskopdaten']])
