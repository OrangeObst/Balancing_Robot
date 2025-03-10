from math import degrees, atan2, sqrt
from util.data_collector import DataCollector
from util import timed_task
from robot.mpu6050 import MyMPU6050
# from robot.mpu6050_copy import MyMPU6050
from util.lowpassfilter import LowPassFilter
from robot.pid_controller import PID_Controller
from robot.stepper_motor import Stepper
# from robot.threaded_motors import Stepper
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
        self.left_motor = left_motor                        # Stepper Motor left
        self.right_motor = right_motor                      # Stepper Motor right
        self.mpu = mpu                                      # MPU6050

        # PIDs
        self.pos_pid = pid1                                 # Unfiltered Position PID
        self.angle_pid = pid2                               # Unfiltered Angle PID

        # Angles
        self.previous_angle = 0.0

        # Filters
        lpf_alpha = 0.2                                     # Low pass filter alpha
        self.lpf_accel_angle = LowPassFilter(lpf_alpha)     # LPF for accel angle
        self.lpf_target_angle = LowPassFilter(lpf_alpha)    # LPF for target angle
        self.alpha = ALPHA                                  # Complementary filter alpha

        # Timed Tasks for static execution times
        # MPU6050 data update frequency depends on dlpf
        # self.update_angle_task = timed_task.TimedTask(delay=DELAY, run=self.update_angle_handler)
        self.control_loop_task = timed_task.TimedTask(delay=DELAY, run=self.control_loop_handler)

        self.counter = 0
        # Logging data
        self.data_collector = data_collector
        self.starting_time = time()


    # @Timer(name="Control loop", text="Control loop: {milliseconds:.6f}ms")
    def control_loop_handler(self, now, dt):
        """Main control loop handler"""
        # data = self._get_all_data()
        data = self.mpu.get_all_data()
        angle, accel_angle, gyro_angle = self._calculate_angle(data, dt)
        avg_steps = self._calculate_average_steps()
        if USE_POS_PID:
            target_angle, pp, pi, pd = self.pos_pid.update(avg_steps, dt)
            if FILTER_TARGET_ANGLE:
                filtered_target_angle = self._filter_target_angle(target_angle)
        else:
            target_angle, pp, pi, pd = 0.0, 0.0, 0.0, 0.0
        angle_pid_input = filtered_target_angle if FILTER_TARGET_ANGLE else target_angle
        speed, ap, ai, ad = self._update_angle_pid(angle_pid_input, angle, dt)
        self._apply_motor_controls(speed)
        self.counter += 1

        if LOG_DATA:
            self._log_data(data, now, angle, accel_angle, gyro_angle, pp, pi, pd, target_angle, ap, ai, ad, speed, avg_steps)
            # print(f'0: {data[0]:7.4f}, 1: {data[1]:7.4f}, 2: {data[2]:7.4f}, 3: {data[3]:7.4f}, 4: {data[4]:7.4f}, 5: {data[5]:7.4f}, Angle: {angle}')
            print(f'Angle: {angle:6.4f} | Speed: {speed:6.4f} | T_angle: {target_angle:6.4f}')
            # print(f'gx: {data[3]:6.4f} | gy: {data[4]:6.4f} | gz: {data[5]:6.4f} | Angle: {angle:6.4f}')
    
    
    def _calculate_angle(self, data, dt):
        """Calculate angle from accelerometer and gyroscope data"""
        accel_angle = self._acceleration_angle(data)
        gyro_angle = self._gyro_angle_integration(data, dt)
        angle = self._complementary_filter(accel_angle, gyro_angle)
        return angle, accel_angle, gyro_angle


    def _acceleration_angle(self, data):
        """Calculate angle from accelerometer data"""
        # accel_angle = degrees(atan2(data[0], max(1e-6,sqrt(data[1]**2 + data[2]**2))))
        accel_angle = degrees(atan2(data[0], -data[2]))
        return accel_angle


    def _gyro_angle_integration(self, data, dt):
        """Calculate angle from gyroscope integration"""
        gyro_angle = self.previous_angle + data[4] * dt
        return gyro_angle


    def _complementary_filter(self, accel_angle, gyro_angle):
        """Apply low pass filter to accel_angle and complementary filter to merge acceleration and gyroscope angles"""
        if FILTER_ACCEL_ANGLE:
            accel_angle = self.lpf_accel_angle.filter(accel_angle)
            if LOG_DATA:
                self.data_collector.log_data('f_accel_angle', accel_angle)
        angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        self.previous_angle = angle
        return angle


    def _calculate_average_steps(self):
        """Calculate average steps from motor positions"""
        steps = ((self.left_motor.get_position() + self.right_motor.get_position()) / 2) / MICROSTEPS
        return steps


    def _update_angle_pid(self, target_angle, angle, dt):
        """Update Angle PID controller"""
        self.angle_pid.set_setpoint(target_angle)
        speed, ap, ai, ad = self.angle_pid.update(angle, dt)
        negated_speed = -speed  # Negative angle => Positive speed. Invert to drive in the right direction
        return negated_speed, ap, ai, ad


    def _filter_target_angle(self, target_angle):
        """Filter target angle to ensure it's within valid bounds"""
        filtered_angle = self.lpf_target_angle.filter(target_angle)
        if LOG_DATA:
            self.data_collector.log_data('filtered_target_angles', filtered_angle)
        return max(-MAX_TARGET_ANGLE, min(MAX_TARGET_ANGLE, filtered_angle))


    def _apply_motor_controls(self, speed):
        """Set motor velocities based on the calculated target velocity"""
        self.left_motor.set_velocity(speed)
        self.right_motor.set_velocity(speed)


    def _log_data(self, data, timestamp, angle, accel_angle, gyro_angle, pp, pi, pd, target_angle, ap, ai, ad, speed, steps):
        self.data_collector.log_data('ax', data[0])
        self.data_collector.log_data('ay', data[1])
        self.data_collector.log_data('az', data[2])
        self.data_collector.log_data('gx', data[3])
        self.data_collector.log_data('gy', data[4])
        self.data_collector.log_data('gz', data[5])
        ms_since_start = (timestamp - self.starting_time) * 1000
        self.data_collector.log_data('timestamped_angles', [ms_since_start, float(angle)])
        self.data_collector.log_data('angle', angle)
        self.data_collector.log_data('accel_angle', accel_angle)
        self.data_collector.log_data('gyro_angle', gyro_angle)
        self.data_collector.log_data('steps', steps)
        self.data_collector.log_pid_data('pos', pp, pi, pd, target_angle)
        self.data_collector.log_pid_data('angle', ap, ai, ad, speed)

    def _get_all_data(self):
        accel_data = mpu.get_accel_data()
        # print(accel_data['x'])
        # print(accel_data['y'])
        # print(accel_data['z'])
        gyro_data = mpu.get_gyro_data()
        # print(gyro_data['x'])
        # print(gyro_data['y'])
        # print(gyro_data['z'])
        return accel_data['x'], accel_data['y'], accel_data['y'], gyro_data['x'], gyro_data['y'], gyro_data['z']

    # @Timer(name="Main loop", text="Main loop: {milliseconds:.6f}ms")
    def loop(self):
        # self.collect_data_task.loop()
        # self.update_angle_task.loop()
        self.control_loop_task.loop()

        if USE_MOTORS:
            self.left_motor.loop()
            self.right_motor.loop()


if __name__ == "__main__":
    pass