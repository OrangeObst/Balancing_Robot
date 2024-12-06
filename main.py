import time
import RPi.GPIO as GPIO
import json
import paho.mqtt.client as mqtt

import numpy as np
from scipy.interpolate import interp1d
from multiprocessing import Lock
from math import degrees, atan2, sqrt
from smbus2 import SMBus
from util import timed_task, plot_graphs, mpu6050, pid_controller, pid_filtered, mqtt_connector
from stepper_motor import Stepper
from codetiming import Timer

SPEED_FACTOR = 0.5      # 0.5   
ALPHA = 0.98            # 0.98
BALANCE_POINT = 0.0     # 0.0
DELAY = 0.01            # 0.01
FACTOR = 1              # 1
AP = 8                  # 8   * FACTOR
AI = 0.2                # 0.2   * FACTOR
AD = 0.08               # 0.08 * FACTOR
AN = 10                 # 10    derivative filter smaller => more filtering
PP = 0.11               # 0.11
PI = 0.0                # 0.0
PD = 0.6                # 0.6
PN = 100                # 100
TIMER = 10
USE_MOTORS = False
REMOTE = False
CALIBRATE = False


class LowPassFilter:
        def __init__(self, alpha):
            self.alpha = alpha
            self.y = 0
        
        def filter(self, x):
            self.y = self.alpha * x + (1 - self.alpha) * self.y
            return self.y

class MovingAverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.data = []

    def update(self, new_value):
        self.data.append(new_value)
        if len(self.data) > self.window_size:
            self.data.pop(0)
        return sum(self.data) / len(self.data)
        
class BalancingRobot:
    def __init__(self, left_motor: Stepper, right_motor: Stepper, mpu: mpu6050, min_velocity, max_velocity, angle_setpoint=0.0):
        # Hardware
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.mpu = mpu
        self.data = [0, 0, 0, 0, 0, 0]
        self.collected_data = [0, 0, 0, 0, 0, 0]
        self.data_buffer = []
        self.counter = 0
        self.lock = Lock()

        # Angles
        self.angle = 0.0
        self.previous_pitch = angle_setpoint

        # Velocities
        self.target_velocity = 0.0
        self.avg_speed = 0.0

        # Low pass filter
        lpf_alpha = 0.2
        self.lpf_x = LowPassFilter(lpf_alpha)
        self.lpf_y = LowPassFilter(lpf_alpha)
        self.lpf_z = LowPassFilter(lpf_alpha)

        # Moving average filter
        self.maf = MovingAverageFilter(window_size=3)

        self.alpha = ALPHA   # complementary filter

        # PIDs
        self.velocity_setpoint = 0.0
        self.min_velocity = min_velocity
        self.max_velocity = max_velocity
        self.angle_setpoint = angle_setpoint
        self.ap = AP
        self.ai = AI
        self.ad = AD
        self.an = AN
        self.position_setpoint = 0.0
        self.min_angle = -25.0
        self.max_angle = 25.0
        self.pp = PP
        self.pi = PI
        self.pd = PD
        self.pn = PN
        self.delay = DELAY

        self.pos_pid = pid_controller.PID_Controller(self.pp, self.pi, self.pd, self.min_angle, self.max_angle, self.position_setpoint)
        self.pos_pid_filtered = pid_filtered.PID_Filtered(self.pp, self.pi, self.pd, self.pn, self.delay, self.min_angle, self.max_angle, self.position_setpoint)
        self.angle_pid = pid_controller.PID_Controller(self.ap, self.ai, self.ad, self.min_velocity, self.max_velocity, self.angle_setpoint)
        self.angle_pid_filtered = pid_filtered.PID_Filtered(self.ap, self.ai, self.ad, self.an, self.delay, self.min_velocity, self.max_velocity, self.angle_setpoint)
        # MPU6050 data update frequency depends on dlpf
        self.collect_data_task = timed_task.TimedTask(delay=0.003, run=self.accumulate_sensor_data)
        self.update_angle_task = timed_task.TimedTask(delay=self.delay, run=self.update_angle_handler)
        self.control_loop_task = timed_task.TimedTask(delay=self.delay, run=self.control_loop_handler)

        # MqttConnector
        if REMOTE:
            self.send_data_task = timed_task.TimedTask(delay=0.003, run=self.send_data_to_remote)

            def on_message(client, userdata, msg):
                message = msg.payload.decode()
                message = json.loads(message)
                delay = time.time() - message[0]
                with self.lock:
                    self.data_buffer.append([message, delay])
                # print(f"Received message: {message} with delay: {delay:.4f}")

            self.mqtt_client = mqtt_connector.MqttConnector(on_message)
            self.mqtt_client.connect_client()

        # extras
        self.apterms = []
        self.aiterms = []
        self.adterms = []
        self.fapterms = []
        self.faiterms = []
        self.fadterms = []
        self.faoutput = []
        self.fpterms = []
        self.fiterms = []
        self.fdterms = []
        self.foutput = []
        self.ppterms = []
        self.piterms = []
        self.pdterms = []
        self.angles = []
        self.aoutput = []
        self.gyro_angles = []
        self.accel_angles = []
        self.accelerations = []
        self.angle_setpoints = []
        self.target_angles = []
        self.filtered_data = []


    def send_data_to_remote(self, now, dt):
        data = mpu.MPU_ReadData()
        payload = json.dumps([now, data])
        self.mqtt_client.publish_data(payload)

    def synchronize_and_interpolate(self, interval=3):
        # timestamps = [ts for ts,_ in self.data_buffer[0]]
        # imu_data = np.array([dp for _,dp in self.data_buffer[0]])
        timestamps = []
        imu_data = []
        delays = []
        for data_set, dt in self.data_buffer:
            timestamps.append(data_set[0])
            imu_data.append(data_set[1])
            delays.append(dt)
        
        with self.lock:
            self.data_buffer.clear()

        # Interpolation
        interp_func = interp1d(timestamps, imu_data, axis=0, fill_value="extrapolate")
        new_timestamps = np.arange(timestamps[0], timestamps[-1], interval)
        # test interval = timestamps[-1] - timestamps[0]
        interpolated_data = interp_func(new_timestamps)
        dt = np.average(delays)
        self.average_data(interpolated_data, len(interpolated_data))
    
    def average_data(self, interpolated_data, window_size=3):
        averaged_data = np.array([
            np.convolve(interpolated_data[:, i], np.ones(window_size) / window_size, mode='valid')
            for i in range(interpolated_data.shape[1])
        ]).T
        self.data = averaged_data[0]

    def accumulate_sensor_data(self, now, dt):
        tmp_data = self.mpu.MPU_ReadData()
        self.add_data_to_buffer(tmp_data)

    def add_data_to_buffer(self, tmp_data):
        self.collected_data = [
            self.collected_data[0] + tmp_data[0], # self.lpf_x.filter(tmp_data[0])
            self.collected_data[1] + tmp_data[1], # self.lpf_y.filter(tmp_data[1])
            self.collected_data[2] + tmp_data[2], # self.lpf_z.filter(tmp_data[2])
            self.collected_data[3] + tmp_data[3],
            self.collected_data[4] + tmp_data[4],
            self.collected_data[5] + tmp_data[5]
        ]
        self.counter += 1

    def get_average_readings(self):
        self.data = [
            self.collected_data[0] / self.counter,
            self.collected_data[1] / self.counter,
            self.collected_data[2] / self.counter,
            self.collected_data[3] / self.counter,
            self.collected_data[4] / self.counter,
            self.collected_data[5] / self.counter
        ]
        # print(self.data)
        self.collected_data.clear()
        self.collected_data = [0, 0, 0, 0, 0, 0]
        self.counter = 0

    # @Timer(name="Update angle", text="Update angle: {milliseconds:.6f}ms")
    def update_angle_handler(self, now, dt):
        if not REMOTE:
            # self.data = self.mpu.MPU_ReadData()
            self.get_average_readings()
        # elif len(self.data_buffer)>3:
        #     self.synchronize_and_interpolate()

        if self.data:
        # if len(self.data_buffer) > 3:
            # self.synchronize_and_interpolate()
            
            # Calculate pitch from accelerometer and gyroscope
            pitch_from_acceleration = degrees(atan2(self.data[0], sqrt(self.data[1]**2 + self.data[2]**2)))
            pitch_gyro_integration = self.previous_pitch + self.data[4] * dt

            self.accel_angles.append(pitch_from_acceleration)
            self.gyro_angles.append(pitch_gyro_integration)

            # Filter volatile acceleration angle
            # pitch_from_acceleration = self.lpf_angle.filter(pitch_from_acceleration)
            pitch_from_acceleration = self.maf.update(pitch_from_acceleration)
            self.filtered_data.append(pitch_from_acceleration)

            # Apply complementary filter
            pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration

            self.previous_pitch = pitch
            self.angle = pitch - BALANCE_POINT
            print(f'Angle: {self.angle:.5}')
            
            self.angles.append(self.angle)


    # @Timer(name="Update PID", text="Update PID: {milliseconds:.6f}ms")
    def control_loop_handler(self, now, dt):

        # Position PID
        avg_steps = (robot.left_motor.get_position() + robot.right_motor.get_position())/2
        self.avg_speed = (1-SPEED_FACTOR) * self.avg_speed + SPEED_FACTOR * self.target_velocity
        
        self.pos_pid.set_setpoint(self.avg_speed)
        tar_angle, pp, pi, pd = self.pos_pid.update(-(avg_steps/3000), dt)
        self.pos_pid_filtered.set_setpoint(self.avg_speed)
        tar_angle_f, ppf, pif, pdf = self.pos_pid_filtered.update(-(avg_steps/3000))
        # TODO: check if update should be - or + avg_steps / 3000
        self.angle_pid.set_setpoint(tar_angle)
        self.angle_pid_filtered.set_setpoint(tar_angle)
        # print(f'avg_steps: {avg_steps:.5f}, avg_speed: {self.avg_speed:.5f}, tar_angle: {tar_angle:.5f}')

        # Angle PID
        s, ap, ai, ad = self.angle_pid.update(self.angle, dt)
        fs, fap, fai, fad = self.angle_pid_filtered.update(self.angle)
        self.target_velocity = fs

        self.left_motor.set_velocity(-self.target_velocity)
        self.right_motor.set_velocity(-self.target_velocity)
        
        self.apterms.append(ap)
        self.aiterms.append(ai)
        self.adterms.append(ad)
        self.aoutput.append(s)
        self.fapterms.append(fap)
        self.faiterms.append(fai)
        self.fadterms.append(fad)
        self.faoutput.append(fs)
        self.ppterms.append(pp)
        self.piterms.append(pi)
        self.pdterms.append(pd)
        self.fpterms.append(ppf)
        self.fiterms.append(pif)
        self.fdterms.append(pdf)
        self.foutput.append(tar_angle_f)
        self.target_angles.append(tar_angle)


    # @Timer(name="Main loop", text="Main loop: {milliseconds:.6f}ms")
    def loop(self):
        if REMOTE:
            self.send_data_task.loop()

        self.collect_data_task.loop()
        self.update_angle_task.loop()
        self.control_loop_task.loop()

        if USE_MOTORS:
            self.left_motor.loop()
            self.right_motor.loop()


if __name__ == "__main__":

    bus = SMBus(1)
    mpu = mpu6050.MyMPU6050(bus)
    if CALIBRATE:
        mpu.calibrate_sensor(2)
    else:
        mpu.set_accel_offset(0.062707, -0.031156, 0.105614) # 0.070627, -0.039342, 0.097467
        mpu.set_gyro_offset(0.184877, -0.599061, -0.786623)   # 0.474419, 0.449830, 0.787613
    mpu.set_dlpf_cfg(2)

    # steps = 200 / microstep-setting 
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=800)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), invert_direction=True, steps=800)
    if USE_MOTORS:
        left_motor.start()
        right_motor.start()

    robot = BalancingRobot(
        left_motor=left_motor,
        right_motor=right_motor,
        mpu=mpu,
        min_velocity=-100,
        max_velocity=100,
        angle_setpoint=BALANCE_POINT)


    timer = time.time() + TIMER
    try:
        while time.time() < timer:
            robot.loop()
            
    except KeyboardInterrupt:
        print("Interrupted")
        pass
    
    if USE_MOTORS:
        left_motor.stop()
        right_motor.stop()
    if REMOTE:
        robot.mqtt_client.disconnect_client()
    
    GPIO.cleanup()

    angle_pid = [
        robot.ap,
        robot.ai,
        robot.ad
    ]
    pos_pid = [
        robot.pp,
        robot.pi,
        robot.pd
    ]
    collected_data = {
        'angles': robot.angles, 
        'accel_angles': robot.accel_angles, 
        'gyro_angles': robot.gyro_angles, 
        'apterms': robot.apterms, 
        'aiterms': robot.aiterms, 
        'adterms': robot.adterms,
        'aoutput': robot.aoutput, 
        'target_angles': robot.target_angles,
        'ppterms': robot.ppterms,
        'piterms': robot.piterms,
        'pdterms': robot.pdterms,
        'fpterms': robot.fpterms,
        'fiterms': robot.fiterms,
        'fdterms': robot.fdterms,
        'foutput': robot.foutput
    }

    plotter = plot_graphs.Plotter(angle_pid, pos_pid, collected_data, TIMER)
    plotter.print_averages()
    plotter.plot_angle_speed(robot.angles, robot.aoutput)
    plotter.plot_angles([[robot.accel_angles,"Accelerometer angle"], [robot.filtered_data, "Filtered angle"], [robot.gyro_angles,"Gyroscope angle"]])
    # plotter.stackplot_pid_values(robot.apterms, robot.aiterms, robot.adterms, 100, "Angle_PID_stackplot")
    # plotter.stackplot_pid_values(robot.ppterms, robot.piterms, robot.pdterms, 20, "Pos_PID_stackplot")
    plotter.subplot_p_i_d_values(angle_pid, robot.apterms, robot.aiterms, robot.adterms, 10, "Angle_PID_subplot")
    plotter.subplot_p_i_d_values(angle_pid, robot.fapterms, robot.faiterms, robot.fadterms, 10, "Angle_PID_filtered")
    plotter.subplot_p_i_d_values(pos_pid, robot.ppterms, robot.piterms, robot.pdterms, 5, "Pos_PID_subplot")
    plotter.subplot_p_i_d_values(pos_pid, robot.fpterms, robot.fiterms, robot.fdterms, 5, "Pos_PID_filtered")
    plotter.plot_angles([[robot.angles,"Robot angle"], [robot.target_angles,"Target angle"]], name="Angle_to_target_angle")
    
    # plotter.plot_angles([[robot.aoutput,"Angle PID"], [robot.foutput, "Filtered angle PID"]], ylim=50, name="Comparison")