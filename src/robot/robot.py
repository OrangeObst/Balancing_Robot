from configparser import ConfigParser
from math import degrees, atan2, sqrt
import numpy as np
from robot.processed_motors import MultiprocessingStepper
from util.timed_task import TimedTask
from util.lowpassfilter import LowPassFilter
from util.websocket import WebSocketServer
from time import time
from util.udp_client import UdpClient
import os

config = ConfigParser()
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
config_file_path = os.path.join(script_dir, 'settings.ini')
config.read(config_file_path)

# Position PID constants
USE_POS_PID = config.getboolean('Position_PID', 'USE_POS_PID')                      # De-/activate position PID controller
MAX_TARGET_ANGLE = config.getfloat('Position_PID', 'MAX_TARGET_ANGLE')              # Max output for position PID controller
FILTER_TARGET_ANGLE = config.getboolean('Position_PID', 'FILTER_TARGET_ANGLE')      # De-/activate filtering for target angle, could reduce instability

# Speed PID constants
USE_SPEED_PID = config.getboolean('Speed_PID', 'USE_SPEED_PID')                     # De-/activate speed PID controller

# Time settings
DELAY = config.getfloat('Time', 'DELAY')                                            # Updatetime delay

# Motor settings
USE_MOTORS = config.getboolean('Motor', 'USE_MOTORS')                               # De-/activate motors
USE_PROCESSED_MOTORS = config.getboolean('Motor', 'USE_PROCESSED_MOTORS')           # De-/activate multiprocessed motors
MICROSTEPS = config.getfloat('Motor', 'MICROSTEPS')                                 # Stepper motor HAT microstep setting

# MPU settings
COMPLEMENTARY_ALPHA = config.getfloat('MPU', 'COMPLEMENTARY_ALPHA')                 # Complementary filter for the accelerometer and gyroscope (MPU6050)
FILTER_ACCEL_ANGLE = config.getboolean('MPU', 'FILTER_ACCEL_ANGLE')                 # De-/activate filtering for acceleration angle, increases reaction time
AVERAGE_MPU_VALUES = config.getboolean('MPU', 'AVERAGE_MPU_VALUES')                 # De-/activate averaging for MPU samples over SAMPLE_TIME
SAMPLE_TIME = config.getfloat('MPU', 'SAMPLE_TIME')                                 # MPUaverager sample time => DELAY / SAMPLE_TIME

# Data logging
LOG_DATA = config.getboolean('Logging', 'LOG_DATA')                                 # De-/activate data logging

# Communication
BROKER = config.get('Communication', 'BROKER')
PORT = config.getint('Communication', 'PORT')

class BalancingRobot:
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
        if USE_PROCESSED_MOTORS:
            self.process_motors = MultiprocessingStepper(self.left_motor, self.right_motor)
            self.process_motors.start()
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

        # Timed Tasks for static execution times
        self.control_loop_task = TimedTask(delay=DELAY, run=self._control_loop_handler)
        self.collect_data_task = TimedTask(delay=SAMPLE_TIME, run=self._accumulate_sensor_data)

        self.previous_angle = 0.0
        self.speed = 0.0
        self.average_speed = 0.0
        self.counter = 0                        # Only to see if the algo doesnt run into time issues 

        # Start-up
        self.startup_angle_stable = False
        self.within_angle_count = 0
        self.stable_angle_threshold = 1     # Degrees
        self.stable_angle_duration = 0.3    # Seconds
        self.last_angle_stable_time = 0.0

        # MPU data
        self.collected_data = []

        # Communication
        self.udp_client = UdpClient(BROKER, PORT)

        # Server
        self.server = WebSocketServer()
        self.server.start()

        # Logging data
        self.data_collector = data_collector
        self.starting_time = time()


    def _control_loop_handler(self, now, dt):
        """Main control loop handler"""
        # Average MPU value to simulate remote calculation possibility
        if AVERAGE_MPU_VALUES:
            data = self._get_average_readings()
        else:
            data = self.mpu.get_all_data()
    
        angle, accel_angle, gyro_angle = self._calculate_angle(data, dt)
        
        timestamp = time()
        self.udp_client.send(timestamp)
        
        # Disable motors at start-up until a good angle has been kept for a duration
        # Might be necessary to prevent PID imbalance at start
        if not self.startup_angle_stable:
            if abs(angle) <= self.stable_angle_threshold:
                if self.within_angle_count == 0:
                    self.last_angle_stable_time = now
                self.within_angle_count += 1
                if (now - self.last_angle_stable_time) >= self.stable_angle_duration:
                    self.startup_angle_stable = True
                    print("Startup angle stability achieved. Starting motor control..")
            else:
                self.within_angle_count = 0  # Reset if angle exceeds threshold
            print(f'Angle: {angle:7.4f} | Angle counter: {self.within_angle_count}')
        else:
            avg_steps = self._calculate_average_steps()
            if USE_POS_PID:
                pos_output, pp, pi, pd = self.pos_pid.update(avg_steps, dt)
                target_angle = pos_output
                if FILTER_TARGET_ANGLE:
                    filtered_target_angle = self._filter_target_angle(pos_output)
            else:
                pos_output, pp, pi, pd = 0.0, 0.0, 0.0, 0.0
                target_angle = pos_output

            # TODO: continue speed PID calibration
            if USE_SPEED_PID:
                avg_steps_per_second = self._calculate_average_speed()
                # self.speed_pid.set_setpoint(avg_steps_per_second)             # average speed in steps per second
                self.speed_pid.set_setpoint(self.average_speed)                 # average speed between -100 and 100
                speed_output, sp, si, sd = self.speed_pid.update(-(avg_steps/1000), dt)
                target_angle = speed_output
            else:
                avg_steps_per_second, speed_output, sp, si, sd = 0.0, 0.0, 0.0, 0.0, 0.0

            angle_pid_setpoint = filtered_target_angle if FILTER_TARGET_ANGLE else target_angle

            self.speed, ap, ai, ad = self._update_angle_pid(angle_pid_setpoint, angle, dt)
            self._apply_motor_controls(self.speed)
            self.counter += 1
            # print(f'Angle: {angle:7.4f} | Speed: {self.speed:7.4f} | dt: {dt:7.4f}')
            self.server.emit_data({'angle': angle, 'accel_angle': accel_angle, 'gyro_angle': gyro_angle})

            if LOG_DATA:
                self._log_data(data, now, angle, accel_angle, gyro_angle, target_angle, pp, pi, pd, pos_output, ap, ai, ad, self.speed, speed_output, sp, si, sd, avg_steps, avg_steps_per_second)
                # print(f'0: {data[0]:7.4f}, 1: {data[1]:7.4f}, 2: {data[2]:7.4f}, 3: {data[3]:7.4f}, 4: {data[4]:7.4f}, 5: {data[5]:7.4f}, Angle: {angle}')
                # print(f'gx: {data[3]:6.4f} | gy: {data[4]:6.4f} | gz: {data[5]:6.4f} | Angle: {angle:6.4f}')

    def _accumulate_sensor_data(self, now, dt):
        tmp_data = self.mpu.get_all_data()
        self.collected_data.append(tmp_data)
        if len(self.collected_data) > 3:
            self.collected_data.pop(0)

    def _get_average_readings(self):
        if len(self.collected_data) > 0:
            avg_data = np.mean(self.collected_data, axis=0)
            return avg_data
        else:
            return [0, 0, 0, 0, 0, 0]

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
        if USE_PROCESSED_MOTORS:
            left_motor_steps, right_motor_steps = self.process_motors.get_steps()
        else:
            left_motor_steps, right_motor_steps = self.left_motor.get_position(), self.right_motor.get_position()
        steps = ((left_motor_steps + right_motor_steps) / 2) # / MICROSTEPS
        return steps

    def _calculate_average_speed(self):
        # Calculates and returns average steps per second
        self.average_speed = 0.5 * self.average_speed + 0.5 * self.speed    # avg_speed in %
        return (self.average_speed / 100) * 3000                            # avg_speed in steps per second

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
        speed = -speed  
        return speed, p_term, i_term, d_term

    def _filter_target_angle(self, target_angle):
        """Filter target angle and ensure it's within valid bounds"""
        filtered_angle = self.lpf_target_angle.filter(target_angle)
        if LOG_DATA:
            self.data_collector.log_data('filtered_target_angles', filtered_angle)
        return max(-MAX_TARGET_ANGLE, min(MAX_TARGET_ANGLE, filtered_angle))

    def _apply_motor_controls(self, speed):
        """Set motor velocities based on the calculated target velocity"""
        if USE_PROCESSED_MOTORS:
            self.process_motors.set_velocity(speed, speed)
        else:
            self.left_motor.set_velocity(speed)
            self.right_motor.set_velocity(speed)

    def shutdown(self):
        if USE_PROCESSED_MOTORS:
            self.process_motors.shutdown()
        else:
            self.left_motor.shutdown()
            self.right_motor.shutdown()
        self.server.stop()

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

        # ms_since_start = (timestamp - self.starting_time) * 1000
        # data_entry = {
        #     'timestamp': ms_since_start,
        #     'ax': data[0],
        #     'ay': data[1],
        #     'az': data[2],
        #     'gx': data[3],
        #     'gy': data[4],
        #     'gz': data[5],
        #     'angle': angle,
        #     'accel_angle': accel_angle,
        #     'gyro_angle': gyro_angle,
        #     'target_angle': target_angle,
        #     'avg_steps': avg_steps,
        #     'avg_steps_per_second': avg_sps,
        #     'pos_p': pp,
        #     'pos_i': pi,
        #     'pos_d': pd,
        #     'pos_output': pos_output,
        #     'angle_p': ap,
        #     'angle_i': ai,
        #     'angle_d': ad,
        #     'angle_o': ao,
        #     'speed_p': sp,
        #     'speed_i': si,
        #     'speed_d': sd,
        #     'speed_output': speed_output
        # }

        # self.data_collector.log_data(data_entry)


    # @Timer(name="Main loop", text="Main loop: {milliseconds:.6f}ms")
    def loop(self):
        if AVERAGE_MPU_VALUES:
            self.collect_data_task.loop()
        self.control_loop_task.loop()

        if USE_MOTORS and not USE_PROCESSED_MOTORS:
            self.left_motor.loop()
            self.right_motor.loop()


if __name__ == "__main__":
    pass