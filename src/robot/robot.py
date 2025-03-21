from configparser import ConfigParser
from math import degrees, atan2, sqrt
from queue import Queue
from util import timed_task
from util.lowpassfilter import LowPassFilter
from robot.MpuDataAverager import MpuDataAverager
from robot.threaded_motors import ThreadedStepper
from time import time

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

        
class BalancingRobot:
    def __init__(self, 
                 left_motor,                # : Stepper
                 right_motor,               # : Stepper
                 mpu,                       # : MyMPU6050 
                 pid1,                      # : PID_Controller 
                 pid2,                      # : PID_Controller
                 data_collector             # : DataCollector
                ):
        
        # Hardware
        self.left_motor = left_motor                        # Stepper Motor left
        self.right_motor = right_motor                      # Stepper Motor right
        if USE_THREADED_MOTORS:
            self.left_motor = ThreadedStepper(self.left_motor)
            self.right_motor = ThreadedStepper(self.right_motor)
            self.left_motor.start()
            self.right_motor.start()
        self.mpu = mpu                                      # MPU6050

        if AVERAGE_MPU_VALUES:
            self.mpudata_queue = Queue()
            self.mpudata_averager = MpuDataAverager(self.mpu, self.mpudata_queue, SAMPLE_TIME)
            self.mpudata_averager.start()

        # PIDs
        self.pos_pid = pid1                                 # Unfiltered Position PID
        self.angle_pid = pid2                               # Unfiltered Angle PID

        # Filters
        lpf_alpha = 0.4                                     # Low pass filter alpha
        self.lpf_accel_angle = LowPassFilter(lpf_alpha)     # LPF for accel angle
        self.lpf_target_angle = LowPassFilter(lpf_alpha)    # LPF for target angle
        self.alpha = COMPLEMENTARY_ALPHA                                  # Complementary filter alpha

        # Timed Tasks for static execution times
        # MPU6050 data update frequency depends on dlpf
        # self.update_angle_task = timed_task.TimedTask(delay=DELAY, run=self.update_angle_handler)
        self.control_loop_task = timed_task.TimedTask(delay=DELAY, run=self.control_loop_handler)

        self.previous_angle = 0.0
        self.previous_speed = 0.0
        self.counter = 0

        # Logging data
        self.data_collector = data_collector
        self.starting_time = time()


    # @Timer(name="Control loop", text="Control loop: {milliseconds:.6f}ms")
    def control_loop_handler(self, now, dt):
        """Main control loop handler"""
        if AVERAGE_MPU_VALUES:
            while self.mpudata_queue.empty():
                pass
            data = self.mpudata_queue.get_nowait()
        else:
            data = self.mpu.get_all_data()
    
        angle, accel_angle, gyro_angle = self._calculate_angle(data, dt)
        avg_steps = self._calculate_average_steps()
        if USE_POS_PID:
            target_angle, pp, pi, pd = self.pos_pid.update(avg_steps, dt)
            if FILTER_TARGET_ANGLE:
                filtered_target_angle = self._filter_target_angle(target_angle)
        else:
            target_angle, pp, pi, pd = 0.0, 0.0, 0.0, 0.0
        angle_pid_setpoint = filtered_target_angle if FILTER_TARGET_ANGLE else target_angle

        speed, ap, ai, ad = self._update_angle_pid(angle_pid_setpoint, angle, dt)
        self._apply_motor_controls(speed)
        self.counter += 1
        print(f'Angle: {angle:7.4f} | Speed: {speed:7.4f} | T_angle: {target_angle:7.4f}')

        if LOG_DATA:
            self._log_data(data, now, angle, accel_angle, gyro_angle, pp, pi, pd, target_angle, ap, ai, ad, speed, avg_steps)
            # print(f'0: {data[0]:7.4f}, 1: {data[1]:7.4f}, 2: {data[2]:7.4f}, 3: {data[3]:7.4f}, 4: {data[4]:7.4f}, 5: {data[5]:7.4f}, Angle: {angle}')
            # print(f'Angle: {angle:7.4f} | Speed: {speed:7.4f} | T_angle: {target_angle:7.4f}')
            # print(f'gx: {data[3]:6.4f} | gy: {data[4]:6.4f} | gz: {data[5]:6.4f} | Angle: {angle:6.4f}')
    
    
    def _calculate_angle(self, data, dt):
        """Calculate angle from accelerometer and gyroscope data"""
        accel_angle = self._acceleration_angle(data)
        gyro_angle = self._gyro_angle_integration(data, dt)
        angle = self._complementary_filter(accel_angle, gyro_angle)
        return angle, accel_angle, gyro_angle


    def _acceleration_angle(self, data):
        """Calculate angle from accelerometer data"""
        accel_angle = degrees(atan2(data[0], max(1e-6,sqrt(data[1]**2 + data[2]**2))))
        # accel_angle = degrees(atan2(data[0], -data[2]))
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


    def _update_angle_pid(self, target_angle: float, angle: float, dt: float) -> tuple[float, float, float, float]:
        """
        Updates the Angle PID controller.

        Args:
        - target_angle (float): The desired angle.
        - angle (float): The current angle.
        - dt (float): Time difference.

        Returns:
        - A tuple containing speed (negated if necessary for directional alignment), 
        proportional, integral, and derivative terms.
        """
        self.angle_pid.set_setpoint(target_angle)
        speed, p_term, i_term, d_term = self.angle_pid.update(angle, dt)
        
        # Negate speed to get positive angle -> positive speed
        negated_speed = -speed  
        return negated_speed, p_term, i_term, d_term


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


    def shutdown(self):
        self.left_motor.shutdown()
        self.right_motor.shutdown()


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


    # @Timer(name="Main loop", text="Main loop: {milliseconds:.6f}ms")
    def loop(self):
        self.control_loop_task.loop()

        if USE_MOTORS and not USE_THREADED_MOTORS:
            self.left_motor.loop()
            self.right_motor.loop()


if __name__ == "__main__":
    pass