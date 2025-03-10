import RPi.GPIO as GPIO
from math import degrees, atan2, sqrt
from smbus2 import SMBus
from robot.mpu6050 import MyMPU6050
from robot.pid_controller import PID_Controller
from robot.stepper_motor import Stepper
from robot.robot import BalancingRobot
# from robot.threaded_motors import Stepper
# from robot.mpu6050_copy import MyMPU6050
from util.data_collector import DataCollector
from util import timed_task, plot_graphs
from util.lowpassfilter import LowPassFilter
# from codetiming import Timer
from time import time

# Angle PID
AP = 17                  # 8
AI = 0.01                # 0.2
AD = 0.00               # 0.08
# Position PID
PP = 0.0005               # 0.0005
PI = 0.0                # 0.0
PD = 0.0006                # 0.0006

ALPHA = 0.98            # Komplementärfilter 
DELAY = 0.01            
TIMER = 10
MICROSTEPS = 8
MAX_TARGET_ANGLE = 5
USE_MOTORS = True
USE_POS_PID = False
FILTER_TARGET_ANGLE = False  # Has to be false if USE_POS_PID is False
FILTER_ACCEL_ANGLE = True
REMOTE = False
AVERAGED = False
CALIBRATE = False
LOG_DATA = True
WRITE_TO_CSV = False

if __name__ == "__main__":

    # ----- MPU -----
    bus = SMBus(1)
    # mpu = MyMPU6050(0x68)
    mpu = MyMPU6050(bus)
    if CALIBRATE:
        mpu.calibrate_sensor(2)
    else:
        mpu.set_accel_offset(0.059397, -0.019336, 0.104918) # 0.074998, -0.025541, 0.101678
        mpu.set_gyro_offset(0.180059, 0.101374, 0.241004) # 0.169651, -0.024273, -0.038918

    # mpu.optimize_sample_settings(DELAY)

    # ----- PID -----
    MAX_VELOCITY = 100
    angle_setpoint = 0.0
    ap = AP
    ai = AI
    ad = AD
    position_setpoint = 0.0
    MAX_TARGET_ANGLE = 25.0
    pid_alpha = 0.5
    pp = PP
    pi = PI
    pd = PD
    delay = DELAY

    pos_pid = PID_Controller(pp, pi, pd, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE, position_setpoint, pid_alpha)
    angle_pid = PID_Controller(ap, ai, ad, -MAX_VELOCITY, MAX_VELOCITY, angle_setpoint, pid_alpha)

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
        if USE_MOTORS:
            print("Stopping Motors ...")
            left_motor.shutdown()
            right_motor.shutdown()
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

        collected_data = data_collector.get_collected_data()
        # DataCollector.print_to_txt_file()
        # DataCollector.print_averages()


        plotter = plot_graphs.Plotter(angle_pid_const, pos_pid_const, TIMER)
        plotter.plot_measurements('Angles [°]', {'Robot angle': collected_data['angle'], 'Target angle': collected_data['pos_pid_terms']['output']}, 'Steps', {'Steps': collected_data['steps']})
        plotter.plot_measurements('Angles [°]', {'Robot angle': collected_data['angle']}, 'Speed', {'Speed': collected_data['angle_pid_terms']['output']}, name='Angle_to_Speed')        
        plotter.plot_measurements('Accel', {'ax': collected_data['ax'], 'ay': collected_data['ay'], 'az': collected_data['az']}, name="Accel_Data")
        plotter.plot_measurements('Gyro', {'gx': collected_data['gx'], 'gy': collected_data['gy'], 'gz': collected_data['gz']}, name="Gyro_Data")

        plotter.subplot_p_i_d_values('Angle', collected_data['angle_pid_terms'], 100, 'PID_Terms')
        plotter.subplot_p_i_d_values('Position', collected_data['pos_pid_terms'], 100, 'PID_Terms')
        
        plotter.plot_angles([[collected_data['angle'],'Robot angle'], [collected_data['pos_pid_terms']['output'],'Target angle']], name='Angle_to_target_angle')
        # plotter.plot_angles([[collected_data['accel_angles'],'Winkel aus Beschleunigungsdaten'], [collected_data['f_accel_angles'], 'Gefilterter Winkel'], [collected_data['gyro_angles'],'Winkel aus Gyroskopdaten']])
