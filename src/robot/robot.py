from robot.processed_motors import MultiprocessingStepper
from util.timed_task import TimedTask
from util.lowpassfilter import LowPassFilter
from util.websocket import WebSocketServer
from util.udp_client import UdpClient
from configparser import ConfigParser
from math import degrees, atan2, sqrt
from time import time
import os

# === Configuration Loading ===
config = ConfigParser()
config.read(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'settings.ini'))

# === Config Constants ===
USE_POS_PID = config.getboolean('Position_PID', 'USE_POS_PID')
MAX_TARGET_ANGLE = config.getfloat('Position_PID', 'MAX_TARGET_ANGLE')
FILTER_TARGET_ANGLE = config.getboolean('Position_PID', 'FILTER_TARGET_ANGLE')
USE_SPEED_PID = config.getboolean('Speed_PID', 'USE_SPEED_PID')
DELAY = config.getfloat('Time', 'DELAY')
USE_MOTORS = config.getboolean('Motor', 'USE_MOTORS')
USE_PROCESSED_MOTORS = config.getboolean('Motor', 'USE_PROCESSED_MOTORS')
MICROSTEPS = config.getfloat('Motor', 'MICROSTEPS')
COMPLEMENTARY_ALPHA = config.getfloat('MPU', 'COMPLEMENTARY_ALPHA')
FILTER_ACCEL_ANGLE = config.getboolean('MPU', 'FILTER_ACCEL_ANGLE')
LOG_DATA = config.getboolean('Logging', 'LOG_DATA')
BROKER = config.get('Communication', 'BROKER')
PORT = config.getint('Communication', 'PORT')


class BalancingRobot:
    def __init__(self, left_motor, right_motor, mpu, angle_pid, pos_pid, speed_pid, data_collector):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.mpu = mpu

        self.angle_pid = angle_pid
        self.pos_pid = pos_pid
        self.speed_pid = speed_pid

        self.data_collector = data_collector
        self.data_to_collect = {}

        self.previous_angle = 0.0
        self.speed = 0.0
        self.average_speed = 0.0
        self.counter = 0

        self._setup_filters()
        self._setup_motors()
        self._setup_comm()
        self._setup_startup_state()

        self.control_loop_task = TimedTask(delay=DELAY, run=self._control_loop)

    def _setup_filters(self):
        alpha = 0.8
        self.lpf_accel_angle = LowPassFilter(alpha)
        self.lpf_target_angle = LowPassFilter(alpha)
        self.alpha = COMPLEMENTARY_ALPHA

    def _setup_motors(self):
        if USE_MOTORS:
            self.left_motor.start()
            self.right_motor.start()

        if USE_PROCESSED_MOTORS:
            self.process_motors = MultiprocessingStepper(self.left_motor, self.right_motor)
            self.process_motors.start()

    # TODO: There might be an issue with the constants callback due to race conditions or timing issues.
    def _set_pid_constants(self, constants):
        if USE_POS_PID:
            self.pos_pid.set_parameters(constants['Pp'], constants['Pi'], constants['Pd'])
        elif USE_SPEED_PID:
            self.speed_pid.set_parameters(constants['Sp'], constants['Si'], constants['Sd'])
        self.angle_pid.set_parameters(constants['Ap'], constants['Ai'], constants['Ad'])

    def _get_pid_constants(self):
        return {
            'Ap': self.angle_pid.kp,
            'Ai': self.angle_pid.ki,
            'Ad': self.angle_pid.kd,
            'Pp': self.pos_pid.kp if USE_POS_PID else None,
            'Pi': self.pos_pid.ki if USE_POS_PID else None,
            'Pd': self.pos_pid.kd if USE_POS_PID else None,
            'Sp': self.speed_pid.kp if USE_SPEED_PID else None,
            'Si': self.speed_pid.ki if USE_SPEED_PID else None,
            'Sd': self.speed_pid.kd if USE_SPEED_PID else None
        }

    def _setup_comm(self):
        self.udp_client = UdpClient(BROKER, PORT)
        self.server = WebSocketServer(
            constants_callback=self._set_pid_constants, 
            constants_provider=self._get_pid_constants)
        self.server.start()

    def _setup_startup_state(self):
        self.startup_angle_stable = False
        self.within_angle_count = 0
        self.stable_angle_threshold = 1
        self.stable_angle_duration = 0.3
        self.last_angle_stable_time = 0.0
        self.starting_time = time()

    def loop(self):
        self.control_loop_task.loop()
        if USE_MOTORS and not USE_PROCESSED_MOTORS:
            self.left_motor.loop()
            self.right_motor.loop()

    def _control_loop(self, now, dt):
        data = self.mpu.get_all_data()
        angle = self._calculate_angle(data, dt)
        if self._check_startup_stability(angle, now):
            speed = self._run_control_logic(angle, dt)
            self._apply_motor_speed(speed)
            self._update_average_speed(speed)
        self.data_collector.emit(self.server.emit_data)
        self.data_collector.snapshot()
        self.counter += 1

    def _check_startup_stability(self, angle, now):
        if self.startup_angle_stable:
            return True
        if abs(angle) <= self.stable_angle_threshold:
            if self.within_angle_count == 0:
                self.last_angle_stable_time = now
            self.within_angle_count += 1
            if now - self.last_angle_stable_time >= self.stable_angle_duration:
                print("Startup angle stability achieved. Starting motor control..")
                self.startup_angle_stable = True
        else:
            self.within_angle_count = 0
        print(f'Angle: {angle:.4f} | Angle counter: {self.within_angle_count}')
        return self.startup_angle_stable

    def _run_control_logic(self, angle, dt):
        steps = self._get_avg_motor_steps()
        target_angle = self._get_target_angle(steps, dt)
        speed = self._update_angle_pid(target_angle, angle, dt)
        return speed

    def _calculate_angle(self, data, dt):
        accel = degrees(atan2(data[0], max(1e-6, sqrt(data[1]**2 + data[2]**2))))
        gyro = self.previous_angle + data[4] * dt
        if FILTER_ACCEL_ANGLE:
            accel = self.lpf_accel_angle.filter(accel)
        angle = self.alpha * gyro + (1 - self.alpha) * accel
        self.previous_angle = angle
        self.data_collector.collect(angle=angle, accel_angle=accel, gyro_angle=gyro)
        return angle

    def _get_avg_motor_steps(self):
        if USE_PROCESSED_MOTORS:
            left, right = self.process_motors.get_steps()
        else:
            left = self.left_motor.get_position()
            right = self.right_motor.get_position()
        avg = (left + right) / 2    # / MICROSTEPS
        self.data_collector.collect(avg_steps=avg)
        return avg
    
    def _update_average_speed(self, speed):
        # Update average speed based on the current speed and return speed in steps per second
        self.average_speed = 0.5 * self.average_speed + 0.5 * speed         # avg_speed in %
    
    def _get_average_speed_in_steps_per_second(self):
        return (self.average_speed / 100) * 3000                            # avg_speed in steps per second

    def _get_target_angle(self, steps, dt):
        if USE_POS_PID:
            self.pos_pid.set_setpoint(self.average_speed)
            output, pp, pi, pd = self.pos_pid.update(-steps/1000, dt)
            target = self.lpf_target_angle.filter(output) if FILTER_TARGET_ANGLE else output   
            target = max(-MAX_TARGET_ANGLE, min(MAX_TARGET_ANGLE, target))
            self.data_collector.collect(pos_pid_output=output, pp=pp, pi=pi, pd=pd, filtered_target_angle=target)
        elif USE_SPEED_PID:
            self.speed_pid.set_setpoint(self.average_speed)
            output, sp, si, sd = self.speed_pid.update(-(steps / 1000), dt)
            target = output
            self.data_collector.collect(speed_pid_output=output, sp=sp, si=si, sd=sd)
        else:
            target = 0.0
        target = -target                # Hard to explain
        self.data_collector.collect(target_angle=target)
        return target

    def _update_angle_pid(self, target, angle, dt):
        self.angle_pid.set_setpoint(target)
        output, ap, ai, ad = self.angle_pid.update(angle, dt)
        speed = -output                 # Invert output for motor control: Positive angle -> positive speed
        self.data_collector.collect(angle_pid_output=output, ap=ap, ai=ai, ad=ad)
        return speed

    def _apply_motor_speed(self, speed):
        if USE_MOTORS:
            self.left_motor.set_velocity(speed)
            self.right_motor.set_velocity(speed)
        elif USE_PROCESSED_MOTORS:
            self.process_motors.set_velocity(speed, speed)

    def shutdown(self):
        if USE_PROCESSED_MOTORS:
            self.process_motors.shutdown()
        else:
            self.left_motor.shutdown()
            self.right_motor.shutdown()
        self.server.stop()
