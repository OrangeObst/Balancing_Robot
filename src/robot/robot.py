import os
from time import time
from math import degrees, atan2, sqrt
from configparser import ConfigParser
from network.udp_client import UdpClient
from network.websocket import WebsocketClient
from util.timed_task import TimedTask
from util.lowpassfilter import LowPassFilter
from multiprocessing import Value

# === Configuration Loading ===
config = ConfigParser()
config.read(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'settings.ini'))

# === Config Constants ===
USE_POS_PID = config.getboolean('Position_PID', 'use_pos_pid')
FILTER_TARGET_ANGLE = config.getboolean('Position_PID', 'filter_target_angle')
DELAY = config.getfloat('Time', 'delay')
COMPLEMENTARY_ALPHA = config.getfloat('MPU', 'complementary_alpha')
FILTER_ACCEL_ANGLE = config.getboolean('MPU', 'filter_accel_angle')
BROKER = config.get('Communication', 'broker')
PORT = config.getint('Communication', 'port')


class BalancingRobot:
    def __init__(self, motor_controller, mpu, angle_pid, pos_pid, data_collector, stop_event=None):
        self.name = 'IAV_SBR_5B_0328'
        self.motor_controller = motor_controller
        self.mpu = mpu

        self.angle_pid = angle_pid
        self.pos_pid = pos_pid
        self.use_pos_pid = USE_POS_PID

        self.data_collector = data_collector

        self._stop_event = stop_event

        self.running = Value('b', False)
        self.previous_angle = 0.0
        self.average_speed = 0.0
        self.counter = 0

        self._setup_comm()

        self.control_loop_task = TimedTask(delay=DELAY, run=self._control_loop)

    # Setup low-pass filters for angle and target angle
    def _setup_filters(self):
        alpha = 0.8
        self.lpf_accel_angle = LowPassFilter(alpha)
        self.lpf_target_angle = LowPassFilter(alpha)
        self.alpha = COMPLEMENTARY_ALPHA

    # Get current PID constants as a dictionary
    def _get_pid_constants(self):
        angle_constants = self.angle_pid.get_constants()
        position_constants = self.pos_pid.get_constants()
        constants = {
            'ap': angle_constants['kp'],
            'ai': angle_constants['ki'],
            'ad': angle_constants['kd'],
            'pp': position_constants['kp'],
            'pi': position_constants['ki'],
            'pd': position_constants['kd'],
        }
        return constants

    # Set PID constants from a dictionary
    def set_pid_constants(self, constants):
        self.angle_pid.set_constants(constants['ap'], constants['ai'], constants['ad'])
        if self.use_pos_pid:
            self.pos_pid.set_constants(constants['pp'], constants['pi'], constants['pd'])

    # Send robot-specific data to the server
    def send_robot_specific_data(self):
        data = {
            'name': self.name,
            'pid_constants': self._get_pid_constants(),
            'use_pos_pid': self.use_pos_pid,
            'running': self.running.value,
        }
        self.client.emit('robot_data', data)
    
    # Calibrate the MPU sensor for a given duration
    def calibrate_mpu(self, duration=3):
        self.mpu.calibrate_sensor(duration)

    # Save current settings to the configuration file
    def save_settings(self):
        global config
        mpu_offsets = self.mpu.get_all_offsets()
        pid_constants = self._get_pid_constants()
        config['MPU']['ax_offset'] = str(mpu_offsets[0])
        config['MPU']['ay_offset'] = str(mpu_offsets[1])
        config['MPU']['az_offset'] = str(mpu_offsets[2])
        config['MPU']['gx_offset'] = str(mpu_offsets[3])
        config['MPU']['gy_offset'] = str(mpu_offsets[4])
        config['MPU']['gz_offset'] = str(mpu_offsets[5])
        config['Angle_PID']['ap'] = str(pid_constants['ap'])
        config['Angle_PID']['ai'] = str(pid_constants['ai'])
        config['Angle_PID']['ad'] = str(pid_constants['ad'])
        config['Position_PID']['pp'] = str(pid_constants['pp'])
        config['Position_PID']['pi'] = str(pid_constants['pi'])
        config['Position_PID']['pd'] = str(pid_constants['pd'])
        with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'settings.ini'), 'w') as settingsfile:
            config.write(settingsfile)

    # Toggle position PID controller
    def switch_pos_pid(self):
        self.use_pos_pid = not self.use_pos_pid
        self.client.emit('pos_pid_status', self.use_pos_pid)
        if self.use_pos_pid:
            self.send_pid_constants()
    
    # Sets the enable pins to HIGH
    def start_processed_motors(self):
        self.motor_controller.start()

    # Sets the enable pins to LOW
    def stop_processed_motors(self):
        self.motor_controller.stop()

    # Activates the multiprocessed motor control
    def activate_processed_motors(self):
        self.motor_controller.activate_processed_motors()

    # Deactivates the multiprocessed motor control
    def deactivate_processed_motors(self):
        self.motor_controller.deactivate_processed_motors()
        self._reset_pids()

    # TODO: Change robot status to send all relevant data at once
    # Enables motors, resets filters and starts the robot control loop
    def start(self):
        if not self.running.value:
            self.start_processed_motors()
            self._setup_filters()
            self._setup_startup_state()
            self._reset_pids()
            self.running.value = True
            self.client.emit('robot_status', self.running.value)
            self.loop()

    # TODO: Motors turn strangely slow after stopping and starting again
    # Disables motors and stops the robot control loop
    def stop(self):
        self.motor_controller.stop()
        self.running.value = False
        print(f'Counter: {self.counter}')
        self.client.emit('robot_status', self.running.value)

    # Reset both PID controllers to prevent issues at restart
    def _reset_pids(self):
        self.angle_pid.reset_controller()
        self.pos_pid.reset_controller()

    # Setup websocket and UDP communication with callback methods
    def _setup_comm(self):
        self.udp_client = UdpClient(BROKER, PORT)
        self.client = WebsocketClient(
            set_pid_constants=self.set_pid_constants,
            send_robot_specific_data=self.send_robot_specific_data,
            calibrate_mpu=self.calibrate_mpu,
            start_robot=self.start,
            stop_robot=self.stop,
            shutdown_robot=self.shutdown,
            save_settings=self.save_settings,
            switch_pos_pid=self.switch_pos_pid,
            start_motors=self.start_processed_motors,
            stop_motors=self.stop_processed_motors,
            activate_motors=self.activate_processed_motors,
            deactivate_motors=self.deactivate_processed_motors,
            server_url=f'http://127.0.0.1:5000'            # server_url=f'http://{BROKER}:{PORT}'
        )
        self.client.connect()
        self.client.register_robot({'name': self.name})

    # Set variables to initial state
    def _setup_startup_state(self):
        self.previous_angle = 0.0
        self.average_speed = 0.0
        self.counter = 0
        self.startup_angle_stable = False
        self.within_angle_count = 0
        self.stable_angle_threshold = 1
        self.stable_angle_duration = 0.5
        self.last_angle_stable_time = 0.0
        self.starting_time = time()

    def loop(self):
        while self.running.value:
            self.control_loop_task.loop()       # Tasks that run at fixed intervals
            self.motor_controller.loop()

    # Main control loop that runs at fixed intervals
    def _control_loop(self, now, dt):
        data = self.mpu.get_all_data()
        angle = self._calculate_angle(data, dt)
        if self._check_startup_stability(angle, now):
            speed = self._run_control_logic(angle, dt)
            self._apply_motor_speed(speed)
            self._update_average_speed(speed)
        self.client.emit('data', self.data_collector.get_latest())
        # self.udp_client.send(self.data_collector.get_latest())
        self.data_collector.snapshot()
        self.counter += 1

    # Check if the robot's angle is stable enough to start balancing
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

    # Run the main control logic
    def _run_control_logic(self, angle, dt):
        steps = self._get_avg_motor_steps()
        target_angle = self._get_target_angle(steps, dt)
        speed = self._update_angle_pid(target_angle, angle, dt)
        return speed

    # Calculate the current angle using a complementary filter
    def _calculate_angle(self, data, dt):
        accel = degrees(atan2(data[0], max(1e-6, sqrt(data[1]**2 + data[2]**2))))
        gyro = self.previous_angle + data[4] * dt
        if FILTER_ACCEL_ANGLE:
            accel = self.lpf_accel_angle.filter(accel)
        angle = self.alpha * gyro + (1 - self.alpha) * accel
        self.previous_angle = angle
        self.data_collector.collect(angle=angle, accel_angle=accel, gyro_angle=gyro)
        return angle

    # Get average motor steps from both motors
    def _get_avg_motor_steps(self):
        left, right = self.motor_controller.get_steps()
        avg = (left + right) / 2
        self.data_collector.collect(avg_steps=avg)
        return avg
    
    # Update average speed based on current speed
    def _update_average_speed(self, speed):
        self.average_speed = 0.5 * self.average_speed + 0.5 * speed         # avg_speed in %
    
    # Convert average speed percentage to steps per second
    def _get_average_speed_in_steps_per_second(self):
        return (self.average_speed / 100) * 3000                            # avg_speed in steps per second

    # Uses position PID to get target angle based on motor steps
    def _get_target_angle(self, steps, dt):
        if self.use_pos_pid:
            self.pos_pid.set_setpoint(self.average_speed)
            output, p_pterm, p_iterm, p_dterm = self.pos_pid.update(-steps/1000, dt)
            target = self.lpf_target_angle.filter(output) if FILTER_TARGET_ANGLE else output   
            self.data_collector.collect(pos_pid_output=output, p_pterm=p_pterm, p_iterm=p_iterm, p_dterm=p_dterm, filtered_target_angle=target)
        else:
            target = 0.0
        # target = -target
        self.data_collector.collect(target_angle=target)
        return target

    # Update angle PID and return speed output
    def _update_angle_pid(self, target, angle, dt):
        self.angle_pid.set_setpoint(target)
        output, a_pterm, a_iterm, a_dterm = self.angle_pid.update(angle, dt)
        speed = -output                 # Invert output for motor control: Positive angle -> positive speed
        self.data_collector.collect(speed=output, a_pterm=a_pterm, a_iterm=a_iterm, a_dterm=a_dterm)
        return speed

    # Apply calculated speed to both motors
    def _apply_motor_speed(self, speed):
        self.motor_controller.set_velocity(speed, speed)

    # Shutdown the robot safely
    def shutdown(self):
        self.motor_controller.shutdown()
        try:
            if self._stop_event is not None:
                self._stop_event.set()
        except Exception as e:
            print(f"Error during robot shutdown: {e}")
