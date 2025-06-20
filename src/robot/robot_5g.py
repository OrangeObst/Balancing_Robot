import multiprocessing
import numpy as np
import time
from configparser import ConfigParser
from math import degrees, atan2, sqrt
from multiprocessing import Process
from util.lowpassfilter import LowPassFilter
from util.udp_client import UdpClient
from util.timed_task import TimedTask

config = ConfigParser()
config.read('/home/newPi/Desktop/Balancing_Robot/src/settings.ini')

# Angle PID constants
AP = config.getfloat('Angle_PID', 'AP')
AI = config.getfloat('Angle_PID', 'AI')
AD = config.getfloat('Angle_PID', 'AD')

# Position PID constants
PP = config.getfloat('Position_PID', 'PP')
PI = config.getfloat('Position_PID', 'PI')
PD = config.getfloat('Position_PID', 'PD')
USE_POS_PID = config.getboolean('Position_PID', 'USE_POS_PID')                      # De-/activate position PID controller
MAX_TARGET_ANGLE = config.getfloat('Position_PID', 'MAX_TARGET_ANGLE')              # Max output for position PID controller
FILTER_TARGET_ANGLE = config.getboolean('Position_PID', 'FILTER_TARGET_ANGLE')      # De-/activate filtering for target angle, could reduce instability

# Speed PID constants
SP = config.getfloat('Speed_PID', 'SP')
SI = config.getfloat('Speed_PID', 'SI')
SD = config.getfloat('Speed_PID', 'SD')
USE_SPEED_PID = config.getboolean('Speed_PID', 'USE_SPEED_PID')

# Time settings
DELAY = config.getfloat('Time', 'DELAY')                                            # Updatetime delay

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

# Communication
BROKER = config.get('Communication', 'BROKER')
PORT = config.getint('Communication', 'PORT')
USE_5G = config.getboolean('Communication', 'USE_5G')

class BalancingRobot5G:
    def __init__(self, 
                 left_motor,                # : Stepper
                 right_motor,               # : Stepper
                 mpu,                       # : MyMPU6050 
                 pid1,                      # : PID_Controller 
                 pid2,                      # : PID_Controller
                 pid3,                      # : PID_Controller
                 data_collector             # : DataCollector
                ):
        
        # Hardware
        self.left_motor = left_motor                        # Stepper Motor left
        self.right_motor = right_motor                      # Stepper Motor right
        self.mpu = mpu                                      # MPU6050

        # PIDs
        self.pos_pid = pid1                                 # Position PID
        self.angle_pid = pid2                               # Angle PID
        self.speed_pid = pid3                               # Speed PID

        # Filters
        lpf_alpha = 0.8                                     # Low pass filter alpha
        self.lpf_accel_angle = LowPassFilter(lpf_alpha)     # LPF for accel angle
        self.lpf_target_angle = LowPassFilter(lpf_alpha)    # LPF for target angle
        self.alpha = COMPLEMENTARY_ALPHA                    # Complementary filter alpha

        self.previous_angle = 0.0
        self.speed = 0.0
        self.average_speed = 0.0

        # Start-up
        self.is_stable = False
        self.within_angle_count = 0
        self.stable_angle_threshold = 1     # Degrees
        self.stable_angle_duration = 0.3    # Seconds
        self.last_angle_stable_time = 0.0

        # MPU data
        self.collected_data = []

        # Communication
        self.client = UdpClient(BROKER, PORT)
        self.last_received_message = 0.0

        # Logging data
        self.data_collector = data_collector
        self.starting_time = time.time()
        self.counter = 0                        # Only to see if the algo doesnt run into time issues 

        # Timed Tasks
        self.main_calculation_task = TimedTask(delay=DELAY, run=self._calculate_angle)

        # Manager
        self.manager = multiprocessing.Manager()
        self.run_processes = self.manager.Value('run_process', True)
        self.stable_angle = self.manager.Value('stable_angle', False)
        self.thread_lock = self.manager.Lock()
        self.messages = self.manager.list()

        # Processes
        self.sending_process = Process(target=self._send_data_handler, args=[self.run_processes, self.client])
        self.receiving_process = Process(target=self._receive_data_handler, args=[self.run_processes, self.client, self.messages])
        # self.motor_control_process = Process(target=self.)
        # self.calculation_process = Process(target=self._data_handler, args=[self.run_processes, self.messages])

    def _send_data_handler(self, run_process, client):
        while run_process.value:
            data = self.mpu.get_all_data()
            client.send(data)
            time.sleep(DELAY)

    def _receive_data_handler(self, run_process, client, messages):
        while run_process.value:
            data = client.receive_messages()
            if data == 'done':
                run_process.value = False
                break
            if data:
                messages.append(data)

    def stable_startup(self):
        # data = self._get_mpu_data()

        while not self.is_stable:
            self.main_calculation_task.loop()
    
    # def _xyz
    #     angle, accel_angle, gyro_angle = self._calculate_angle(data, dt)
    #     if not self.stable_angle.value:
    #         if abs(angle) <= self.stable_angle_threshold:
    #             if self.within_angle_count == 0:
    #                 self.last_angle_stable_time = now
    #             self.within_angle_count += 1
    #             if (now - self.last_angle_stable_time) >= self.stable_angle_duration:
    #                 self.startup_angle_stable = True
    #                 print("Startup angle stability achieved. Starting motor control..")
    #         else:
    #             self.within_angle_count = 0  # Reset if angle exceeds threshold
    #         print(f'Angle: {angle:7.4f} | Angle counter: {self.within_angle_count}')
    
    def _data_handler(self):
        # TODO: Not sure how to make the process wait for the first few values
        # Possible the same way I did with the startup_angle
        # while run_process.value:
            # time.sleep(0.02)
        data = list(self.messages)
        self.messages[:] = []
        print(f'data: {data}')

    def _control_loop_handler(self, now, dt):
        """Main control loop handler"""
        # Average MPU value to simulate remote calculation possibility
        data = list(self.messages)

        print(f'data: {data} | counter: {self.counter}')
        self.counter += 1

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
        steps = ((self.left_motor.get_position() + self.right_motor.get_position()) / 2) # / MICROSTEPS
        return steps

    def _calculate_average_speed(self):
        # Calculates and returns average steps per second
        self.average_speed = 0.5 * self.average_speed + 0.5 * self.speed    # avg_speed in %
        return (self.average_speed / 100) * 3000                            # avg_speed in steps per second

    def _update_angle_pid(self, target_angle: float, angle: float, dt: float) -> tuple[float, float, float, float]:
        self.angle_pid.set_setpoint(target_angle)
        speed, p_term, i_term, d_term = self.angle_pid.update(angle, dt)
        
        # Negate speed to get positive angle -> positive speed
        speed = -speed  
        return speed, p_term, i_term, d_term

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


    def start(self):
        self.sending_process.start()
        self.receiving_process.start()
        # self.stable_startup
        self.calculation_process.start()

    def shutdown(self):
        print("Shutdown called")
        self.run_processes.value = False
        self.left_motor.shutdown()
        self.right_motor.shutdown()
        self.sending_process.join()
        self.receiving_process.join()
        self.calculation_process.join()

    def _log_data(self, data, timestamp, angle, accel_angle, gyro_angle, target_angle, pp, pi, pd, pos_pid_output, ap, ai, ad, angle_pid_output, sp, si, sd, speed_pid_output, avg_steps, avg_sps):
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
        self.data_collector.log_data('target_angle', target_angle)
        self.data_collector.log_data('avg_steps', avg_steps)
        self.data_collector.log_data('avg_sps', avg_sps)
        self.data_collector.log_pid_data('pos', pp, pi, pd, pos_pid_output)
        self.data_collector.log_pid_data('angle', ap, ai, ad, angle_pid_output)
        self.data_collector.log_pid_data('speed', sp, si, sd, speed_pid_output)


if __name__ == "__main__":
    pass