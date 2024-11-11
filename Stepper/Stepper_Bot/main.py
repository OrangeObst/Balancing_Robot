import time
import RPi.GPIO as GPIO
import json
import paho.mqtt.client as mqtt

from math import degrees, atan2, sqrt
from smbus2 import SMBus
from util import timed_task, plot_graphs, mpu6050, pid_controller
from stepper_motor import Stepper
from codetiming import Timer

ALPHA = 0.98
BALANCE_POINT = -0.09
DELAY = 0.05
FACTOR = 1
AP = 9           #9   * FACTOR
AI = 0           #    * FACTOR # 3
AD = 0.3         #0.3 * FACTOR
SP = 0.005
SI = 0.003
SD = 0.0
ACC_STEPS = 5
USE_MOTORS = True
TIMER = 10
REMOTE = False

# Best results so far alpha=0.96, balancep=0.0, delay=0.005, factor=1, KP=9, KI=2, KD=0.3, acc_step=5

class LowPassFilter:
        def __init__(self, alpha):
            self.alpha = alpha
            self.y = 0
        
        def filter(self, x):
            self.y = self.alpha * x + (1 - self.alpha) * self.y
            return self.y
        
class BalancingRobot:
    def __init__(self, left_motor: Stepper, right_motor: Stepper, mpu: mpu6050, min_velocity, max_velocity, angle_setpoint=0.0):
        # Hardware
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.mpu = mpu

        # Angles
        self.angle_setpoint = angle_setpoint
        self.angle = 0.0
        self.previous_pitch = self.angle_setpoint

        # Velocities
        self.velocity_setpoint = 0.0
        self.a = 0.0
        self.acceleration_steps = ACC_STEPS
        self.velocity = 0.0
        self.target_velocity = 0.0

        self.alpha = ALPHA   # complementary filter

        self.min_velocity = min_velocity
        self.max_velocity = max_velocity
        self.min_angle = -5.0
        self.max_angle = 5.0
        
        self.sp = SP
        self.si = SI
        self.sd = SD
        self.ap = AP
        self.ai = AI
        self.ad = AD
        self.delay = DELAY
        self.speed_pid = pid_controller.PID_Controller(self.sp, self.si, self.sd, self.min_angle, self.max_angle, self.velocity_setpoint)
        self.angle_pid = pid_controller.PID_Controller(self.ap, self.ai, self.ad, self.min_velocity, self.max_velocity, self.angle_setpoint)
        self.sensor_read_task = timed_task.TimedTask(delay=self.delay, run=self.update_angle_handler)
        self.control_loop_task = timed_task.TimedTask(delay=self.delay, run=self.control_loop_handler)
        self.update_velocity_task = timed_task.TimedTask(delay=self.delay/self.acceleration_steps, run=self.update_velocity_handler)

        # MqttConnector
        if REMOTE:
            self.send_data_task = timed_task.TimedTask(delay=self.delay, run=self.send_data_to_remote)
            self.broker = "10.224.64.29"
            self.port = 1883
            self.send_topic = "mpu6050/data"
            self.receive_topic = "mpu6050/data"
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            self.data = []

            def on_connect(client, userdata, flags, rc, *args, **kwargs):
                print(f"Connected with result code {rc}")
                client.subscribe(self.receive_topic)

            def on_publish(client, userdata, mid, *args, **kwargs):
                print(f"Message {mid} published.")

            def on_message(client, userdata, msg):
                message = msg.payload.decode()
                message = json.loads(message)
                self.data = message
                print(f"Received message: {message}")

            self.client.on_connect = on_connect
            self.client.on_publish = on_publish
            self.client.on_message = on_message
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()

        # extras
        self.apterms = []
        self.aiterms = []
        self.adterms = []
        self.spterms = []
        self.siterms = []
        self.sdterms = []
        self.angles = []
        self.velocities = []
        self.gyro_angles = []
        self.accel_angles = []
        self.accelerations = []
        self.angle_setpoints = []


    def send_data_to_remote(self, now, dt):
        data = mpu.MPU_ReadData()
        payload = json.dumps(data)
        self.client.publish(self.send_topic, payload)

    # @Timer(name="Update angle", text="Update angle: {milliseconds:.6f}ms")
    def update_angle_handler(self, now, dt):
        # accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.mpu.MPU_ReadData()
        self.data = self.mpu.MPU_ReadData()
        if self.data:
            # accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.data
            # Calculate pitch from accelerometer and gyroscope
            # pitch_from_acceleration = degrees(atan2(accel_x, -accel_z))
            pitch_from_acceleration = degrees(atan2(self.data[0], sqrt(self.data[1]**2 + self.data[2]**2)))
            pitch_gyro_integration = self.previous_pitch + self.data[4] * dt

            # Apply complementary filter
            pitch = self.alpha * pitch_gyro_integration + (1 - self.alpha) * pitch_from_acceleration

            self.previous_pitch = pitch
            self.angle = pitch - BALANCE_POINT
            print(f'Angle: {self.angle:.5}')
            
            self.angles.append(self.angle)
            self.accel_angles.append(pitch_from_acceleration - BALANCE_POINT)
            self.gyro_angles.append(pitch_gyro_integration - BALANCE_POINT)

    # @Timer(name="Update PID", text="Update PID: {milliseconds:.6f}ms")
    def control_loop_handler(self, now, dt):
        angle, sp, si, sd = self.speed_pid.update(self.velocity, dt)
        # print(f'test: {angle}, {sp}, {si}, {sd}')
        self.angle_pid.set_setpoint(-angle)

        s, ap, ai, ad = self.angle_pid.update(self.angle, dt)
        self.target_velocity = s
        self.a = (self.target_velocity - self.velocity) / self.acceleration_steps
        # print(f"Angle: {self.angle:.4f}; Target_velocity: {self.target_velocity:.4f}; Step_Delay: {self.left_motor.step_delay:.4f}")
        
        self.apterms.append(ap)
        self.aiterms.append(ai)
        self.adterms.append(ad)
        self.spterms.append(sp)
        self.siterms.append(si)
        self.sdterms.append(sd)
        self.velocities.append(self.target_velocity)

    # @Timer(name="Update velocity", text="Update velocity: {milliseconds:.6f}ms")
    def update_velocity_handler(self, now, dt):
        self.velocity += self.a
        if self.velocity > self.max_velocity:
            self.velocity = self.max_velocity
        if self.velocity < self.min_velocity:
            self.velocity = self.min_velocity

        self.accelerations.append(self.velocity)
        # print(f'Current velocity: {self.velocity:.4f}')
        self.left_motor.set_velocity(self.velocity)
        self.right_motor.set_velocity(-self.velocity)

    # @Timer(name="Main loop", text="Main loop: {milliseconds:.6f}ms")
    def loop(self):
        if REMOTE:
            self.send_data_task.loop()

        self.sensor_read_task.loop()
        self.control_loop_task.loop()
        self.update_velocity_task.loop()
        if USE_MOTORS:
            self.left_motor.loop()
            self.right_motor.loop()


if __name__ == "__main__":

    bus = SMBus(1)
    mpu = mpu6050.MyMPU6050(bus)
    # mpu.calibrate_sensor(2)
    mpu.set_dlpf_cfg(6)
    mpu.set_accel_offset(0.059984, -0.039342, 0.097467) # 0.070627, -0.039342, 0.097467
    mpu.set_gyro_offset(0.552760, 0.509830, 0.787613)   # 0.474419, 0.449830, 0.787613

    # steps = 200 / microstep-setting 
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=800)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), steps=800)
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

    GPIO.cleanup()

    angle_pid = [
        robot.ap,
        robot.ai,
        robot.ad
    ]
    speed_pid = [
        robot.sp,
        robot.si,
        robot.sd
    ]
    collected_data = {
        'angles': robot.angles, 
        'accel_angles': robot.accel_angles, 
        'gyro_angles': robot.gyro_angles, 
        'velocities': robot.velocities, 
        'accelerations': robot.accelerations, 
        'apterms': robot.apterms, 
        'aiterms': robot.aiterms, 
        'adterms': robot.adterms,
        'spterms': robot.spterms,
        'siterms': robot.siterms,
        'sdterms': robot.sdterms
    }

    plotter = plot_graphs.Plotter(angle_pid, speed_pid, collected_data, TIMER)
    plotter.print_averages()
    plotter.plot_angle_speed(robot.angles, robot.velocities, robot.accelerations)
    plotter.stackplot_pid_values(robot.apterms, robot.aiterms, robot.adterms, 100, "Angle_PID_stackplot")
    plotter.stackplot_pid_values(robot.spterms, robot.siterms, robot.sdterms, 2, "Speed_PID_stackplot")
    plotter.plot_angles(robot.accel_angles, robot.gyro_angles)
    plotter.subplot_p_i_d_values(angle_pid, robot.apterms, robot.aiterms, robot.adterms, 10, "Angle_PID_subplot")
    plotter.subplot_p_i_d_values(speed_pid, robot.spterms, robot.siterms, robot.sdterms, 2, "Speed_PID_subplot")