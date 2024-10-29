import time
import RPi.GPIO as GPIO
from math import degrees, atan2

from smbus2 import SMBus
from util import timed_task, plot_graphs, mpu6050, pid_controller
from stepper_motor import Stepper

PITCH_BIAS = 0


class BalancingRobot:
    def __init__(self, left_motor: Stepper, right_motor: Stepper, mpu: mpu6050, min_velocity, max_velocity, angle_bias=0.0):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.mpu = mpu

        self.angle_bias = angle_bias
        self.angle = 0.0
        self.previous_pitch = 0.0

        self.u = 5.0
        self.velocity = 0.0
        self.target_velocity = 0.0
        self.setpoint = 0.0
        self.v = 0.0

        self.alpha = 0.96
        self.delay = 0.01

        self.min_velocity = min_velocity
        self.max_velocity = max_velocity

        self.pid = pid_controller.PID_Controller(5, 0.05, 0.4, self.min_velocity, self.max_velocity, self.setpoint)
        self.sensor_read_task = timed_task.TimedTask(delay=self.delay/2, run=self.update_angle_handler)
        self.control_loop_task = timed_task.TimedTask(delay=self.delay/2, run=self.control_loop_handler)
        self.update_velocity_task = timed_task.TimedTask(delay=self.delay/10, run=self.update_velocity_handler)

        # extras
        self.pterms = []
        self.iterms = []
        self.dterms = []
        self.angles = []
        self.velocities = []

    def update_angle_handler(self, now, dt):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.MPU_ReadData()

        # Calculate pitch from accelerometer and gyroscope
        pitch_from_acceleration = degrees(atan2(-accel_x, -accel_z))
        pitch_gyro_integration = self.previous_pitch + gyro_y * dt
        # print(f'Pitch from acc: {pitch_from_acceleration}, Pitch from gyro: {pitch_gyro_integration}')
        
        # Apply complementary filter
        pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration

        self.previous_pitch = pitch
        self.angle = pitch - self.angle_bias
        print(f'Angle: {self.angle}')
        self.angles.append(self.angle)

    def control_loop_handler(self, now, dt):
        u,p,i,d = self.pid.update(self.angle, dt)
        self.target_velocity = u
        self.v = (self.target_velocity - self.velocity) / 5
        print(f"Angle: {self.angle:.3f}; Target_velocity: {self.target_velocity}; Step_Delay: {self.left_motor.step_delay}")
        self.pterms.append(p)
        self.iterms.append(i)
        self.dterms.append(d)

    def update_velocity_handler(self, now, dt):
        self.velocity += self.v
        if self.velocity > self.max_velocity:
            self.velocity = self.max_velocity
        if self.velocity < self.min_velocity:
            self.velocity = self.min_velocity

        self.velocities.append(self.velocity)
        print(f'Current velocity: {self.velocity}')
        self.left_motor.set_velocity(self.velocity)
        self.right_motor.set_velocity(-self.velocity)

    def loop(self):
        self.sensor_read_task.loop()
        self.control_loop_task.loop()
        self.update_velocity_task.loop()
        self.left_motor.loop()
        self.right_motor.loop()


if __name__ == "__main__":

    bus = SMBus(1)
    mpu = mpu6050.MyMPU6050(bus)
    mpu.set_dlpf_cfg(2)
    mpu.set_smplrt_div(4)
    
    # steps = 200 / stepsetting 
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=400)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), steps=400)
    # left_motor.start()
    # right_motor.start()

    robot = BalancingRobot(
        left_motor=left_motor,
        right_motor=right_motor,
        mpu=mpu,
        min_velocity=-100,
        max_velocity=100,
        angle_bias=PITCH_BIAS)

    timer = time.time() + 10
    try:
        while time.time() < timer:
            # robot.loop()
            robot.sensor_read_task.loop()
    except KeyboardInterrupt:
        print("Interrupted")
        pass
    
    # left_motor.stop()
    # right_motor.stop()
    GPIO.cleanup()
    
    # plotter = plot_graphs.Plotter()
    # plotter.plot_angle_speed(robot.angles, robot.velocities, robot.setpoint)
    # plotter.stackplot_pid_values(robot.pterms, robot.iterms, robot.dterms)