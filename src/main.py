import time
import RPi.GPIO as GPIO
import numpy as np
from math import degrees, atan2, sqrt
from smbus2 import SMBus
from util.data_collector import DataCollector
from util import timed_task, plot_graphs
from robot.mpu6050 import MyMPU6050
from util.lowpassfilter import LowPassFilter
from robot.pid_controller import PID_Controller
from robot.stepper_motor import Stepper
from codetiming import Timer

ALPHA = 0.98            # 0.98
DELAY = 0.01            # 0.01
# Angle PID
AP = 11                  # 8
AI = 0.4                # 0.2
AD = 0.08               # 0.08
# Position PID
PP = 0.005               # 0.005
PI = 0.0                # 0.0
PD = 0.0                # 0.6
TIMER = 10
MICROSTEPS = 4
MAX_TARGET_ANGLE = 5
USE_MOTORS = True
REMOTE = False
AVERAGED = False
CALIBRATE = False
LOG_DATA = True
WRITE_TO_CSV = False

        
class BalancingRobot:
    def __init__(self, 
                 left_motor: Stepper, 
                 right_motor: Stepper, 
                 mpu: MyMPU6050, 
                 pid1: PID_Controller, 
                 pid2: PID_Controller,
                 data_collector: DataCollector 
                ):
        
        # Hardware
        self.left_motor = left_motor                # Stepper Motor left
        self.right_motor = right_motor              # Stepper Motor right
        self.mpu = mpu                              # MPU6050

        # PIDs
        self.pos_pid = pid1                         # Unfiltered Position PID
        self.angle_pid = pid2                       # Unfiltered Angle PID

        # Angles and speeds
        self.angle = 0.0
        self.previous_pitch = 0.0
        self.data = [0, 0, 0, 0, 0, 0]
        self.collected_data = []
        self.target_velocity = 0.0

        # Filters
        lpf_alpha = 0.2                             # Low pass filter alpha
        self.lpf_angle = LowPassFilter(lpf_alpha)   # LPF for angle
        self.lpf_target_angle = LowPassFilter(lpf_alpha)
        self.alpha = ALPHA                          # Complementary filter alpha

        # Timed Tasks for static execution times
        # MPU6050 data update frequency depends on dlpf
        # self.collect_data_task = timed_task.TimedTask(delay=DELAY, run=self.accumulate_sensor_data)
        # self.update_angle_task = timed_task.TimedTask(delay=DELAY, run=self.update_angle_handler)
        self.control_loop_task = timed_task.TimedTask(delay=DELAY, run=self.control_loop_handler)

        # Logging data
        if LOG_DATA:
            self.data_collector = data_collector
            self.starting_time = time.time()


    # @Timer(name="sensor", text="Collect data: {milliseconds:.6f}ms")
    def accumulate_sensor_data(self, now, dt):
        tmp_data = self.mpu.get_all_data()
        self.collected_data.append(tmp_data)
        if len(self.collected_data) > 3:
            self.collected_data.pop(0)

    # @Timer(name="average", text="average angle: {milliseconds:.6f}ms")
    def get_average_readings(self):
        if len(self.collected_data) > 0:
            avg_data = np.mean(self.collected_data, axis=0)
            return avg_data
        else:
            return [0, 0, 0, 0, 0, 0]


    # # @Timer(name="angle", text="Update angle: {milliseconds:.6f}ms")
    # def update_angle_handler(self, now, dt):
    # @Timer(name="pid", text="Update PID: {milliseconds:.6f}ms")
    def control_loop_handler(self, now, dt):
        if AVERAGED:
            self.data = self.get_average_readings()
        else:
            self.data = self.mpu.get_all_data()

        # Calculate pitch from accelerometer and gyroscope
        # pitch_from_acceleration = degrees(atan2(self.data[0], max(1e-6,sqrt(self.data[1]**2 + self.data[2]**2))))
        pitch_from_acceleration = degrees(atan2(self.data[0], -self.data[2]))
        pitch_gyro_integration = self.previous_pitch + self.data[4] * dt

        # Filter volatile acceleration angle
        f_accel_angle = self.lpf_angle.filter(pitch_from_acceleration)

        # Apply complementary filter
        pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration

        self.previous_pitch = pitch
        self.angle = pitch

        if LOG_DATA:
            self.data_collector.log_angle_data(pitch, pitch_gyro_integration, pitch_from_acceleration, f_accel_angle)
            ms_since_start = (now - self.starting_time) * 1000
            self.data_collector.log_data('timestamped_angles', [ms_since_start, float(self.angle)])
            print(f'0: {self.data[0]:7.4f}, 1: {self.data[1]:7.4f}, 2: {self.data[2]:7.4f}, 3: {self.data[3]:7.4f}, 4: {self.data[4]:7.4f}, 5: {self.data[5]:7.4f}, Pitch: {pitch}')

        # Position PID
        # avg_steps = ((robot.left_motor.get_position() + robot.right_motor.get_position())/2) / MICROSTEPS 
        # revolutions = avg_steps / 200 / MICROSTEPS      # steps / 200 steps per revolution / Microstepping => Actual revolution
        # tar_angle, pp, pi, pd = self.pos_pid.update(avg_steps, dt)

        # Angle PID
        # filtered_target_angle = self.lpf_target_angle.filter(-tar_angle)
        # filtered_target_angle = max(-MAX_TARGET_ANGLE, min(MAX_TARGET_ANGLE, filtered_target_angle))
        # self.angle_pid.set_setpoint(filtered_target_angle)

        speed, ap, ai, ad = self.angle_pid.update(self.angle, dt)
        self.target_velocity = -speed                   # Negative angle => Positive speed. Invert to drive in the right direction
        
        try:
            ms_since_start = (now - self.starting_time) * 1000
            # print(f'Ms: {ms_since_start:5.2f} | Angle: {self.angle:7.4f} | Speed: {self.target_velocity:7.4f} | Revolutions: {revolutions:7.4f} | target angle: {tar_angle:7.4f} | target angle: {filtered_target_angle:7.4f} | P: {ap:7.4f} I: {ai:7.4f} D: {ad:7.4f} | dt: {dt:7.4f}')
            # print(f'Ms: {ms_since_start:5.2f} | Steps: {avg_steps:7.4f} | Target angle: {tar_angle:7.4f} | Angle: {self.angle:7.4f} | Speed: {self.target_velocity:7.4f}')
        except Exception as e:
            print(f"oops: {e}")

        self.left_motor.set_velocity(self.target_velocity)
        self.right_motor.set_velocity(self.target_velocity)
        
        if LOG_DATA:
            # self.data_collector.log_pid_data('pos', pp, pi, pd, tar_angle)
            self.data_collector.log_pid_data('angle', ap, ai, ad, self.target_velocity)
            # self.data_collector.log_data('steps', avg_steps)
            # self.data_collector.log_data('target_angles', filtered_target_angle)



    # @Timer(name="Main loop", text="Main loop: {milliseconds:.6f}ms")
    def loop(self):
        # self.collect_data_task.loop()
        # self.update_angle_task.loop()
        self.control_loop_task.loop()

        if USE_MOTORS:
            self.left_motor.loop()
            self.right_motor.loop()


if __name__ == "__main__":

    # ----- MPU -----
    bus = SMBus(1)
    mpu = MyMPU6050(bus)
    if CALIBRATE:
        mpu.calibrate_sensor(2)
    else:
        mpu.set_accel_offset(0.074998, -0.025541, 0.101678) # 0.045213, -0.020453, 0.103890
        mpu.set_gyro_offset(0.169651, -0.024273, -0.038918) # 0.260446, -0.063810, 0.010697
    mpu.set_dlpf_cfg(4)

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
    pid_alpha = 0.5
    pp = PP
    pi = PI
    pd = PD
    delay = DELAY

    pos_pid = PID_Controller(pp, pi, pd, min_angle, max_angle, position_setpoint, pid_alpha)
    angle_pid = PID_Controller(ap, ai, ad, min_velocity, max_velocity, angle_setpoint, pid_alpha)

    # ----- Motor -----
    stepresolution = 200 * MICROSTEPS
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=stepresolution)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), invert_direction=True, steps=stepresolution)
    if USE_MOTORS:
        left_motor.start()
        right_motor.start()

    # ----- Logging -----
    data_collector = DataCollector()
    
    # ----- Robot -----
    robot = BalancingRobot(
        left_motor = left_motor,
        right_motor = right_motor,
        mpu = mpu,
        pid1 = pos_pid,
        pid2 = angle_pid,
        data_collector = data_collector
    )


    timer = time.time() + TIMER
    try:
        while time.time() < timer:
            robot.loop()
    except KeyboardInterrupt:
        print("Interrupted")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if USE_MOTORS:
            print("Stopping Motors ...")
            left_motor.stop()
            right_motor.stop()
        print("Cleaning up GPIO ...")
        GPIO.cleanup()
        print("Exiting ...")


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

        collected_data = data_collector.get_all_collected_data()
        robot.data_collector.print_averages()

        if WRITE_TO_CSV:
            data_collector.write_timestamped_angles_to_csv('/home/newPi/Desktop/Balance_Bot/Stepper_Bot/Messungen')

        plotter = plot_graphs.Plotter(angle_pid_const, pos_pid_const, TIMER)
        # plotter.plot_measurements('Angles [°]', {'Robot angle': collected_data['angles'], 'Target angle': collected_data['pos_pid_terms']['output']}, 'Steps', {'Steps': collected_data['steps']})
        plotter.plot_measurements('Angles [°]', {'Robot angle': collected_data['angles']}, 'Speed', {'Speed': collected_data['angle_pid_terms']['output']}, name='Angle_to_Speed')        
        
        plotter.subplot_p_i_d_values('Angle', collected_data['angle_pid_terms'], 100, 'PID_Terms')
        # plotter.subplot_p_i_d_values('Position', collected_data['pos_pid_terms'], 100, 'PID_Terms')
        
        # plotter.plot_angles([[collected_data['angles'],'Robot angle'], [collected_data['pos_pid_terms']['output'],'Target angle']], name='Angle_to_target_angle')
        plotter.plot_angles([[collected_data['accel_angles'],'Winkel aus Beschleunigungsdaten'], [collected_data['f_accel_angles'], 'Gefilterter Winkel'], [collected_data['gyro_angles'],'Winkel aus Gyroskopdaten']])



        # plotter.plot_angles([[robot.aoutput,"Angle PID"], [robot.foutput, "Filtered angle PID"]], ylim=50, name="Comparison")
        # plotter.stackplot_pid_values(collected_data['angle_pid_terms'], "Angle_PID_Stackplot")
        # plotter.stackplot_pid_values(collected_data['pos_pid_terms'], "Pos_PID_Stackplot")