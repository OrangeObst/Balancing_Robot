import socket
import json
import threading
import time
from math import degrees, atan2, sqrt

# TODO: read up on kalman filter. run kalman on every value or on averages?
# ping / rtt info: https://github.com/ChuanyuXue/udp-latency?tab=readme-ov-file
# https://github.com/bestvibes/IEEE1588-PTP/blob/dev/slave/slave.py

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 17002))

sensor_buffer = []
buffer_lock = threading.Lock()
last_addr = None

class SimpleKalmanFilter:
    def __init__(self, q_angle=0.001, q_gyro=0.003, r_angle=0.03):
        self.q_angle = q_angle
        self.q_gyro = q_gyro
        self.r_angle = r_angle
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, accel_angle, gyro_rate, dt):
        # Predict
        rate = gyro_rate - self.bias
        self.angle += dt * rate

        # Update error covariance matrix
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[1][0] - self.P[0][1] + self.q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.q_gyro * dt

        # Innovation
        S = self.P[0][0] + self.r_angle
        K = [self.P[0][0] / S, self.P[1][0] / S]

        y = accel_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Update error covariance matrix
        p00_temp = self.P[0][0]
        p01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * p00_temp
        self.P[0][1] -= K[0] * p01_temp
        self.P[1][0] -= K[1] * p00_temp
        self.P[1][1] -= K[1] * p01_temp

        return self.angle

kalman = SimpleKalmanFilter()

def get_accel_angle(accel_x, accel_y, accel_z):
    # Calculate tilt angle from accelerometer (in degrees)
    return degrees(atan2(accel_x, max(1e-6, sqrt(accel_y**2 + accel_z**2))))

def handle_sensor_data(msg, addr):
    global last_addr
    with buffer_lock:
        sensor_buffer.append(msg)
        sensor_buffer.sort(key=lambda d: d.get('timestamp', 0))
        sensor_buffer[:] = sensor_buffer[-4:]
    last_addr = addr

handlers = {
    "sensor_data": handle_sensor_data,
}

def control_loop():
    last_timestamp = None
    while True:
        time.sleep(0.02)  # 20ms
        with buffer_lock:
            if not sensor_buffer:
                continue
            batch = sensor_buffer[-4:]
            sensor_buffer.clear()

        # Use the latest sensor value for prediction
        latest = batch[-1]
        accel_angle = get_accel_angle(latest['accel_x'], latest['accel_y'], latest['accel_z'])
        # Use gyro_y for tilt around the robot's axis (adjust if needed)
        gyro_rate = latest['gyro_y']
        timestamp = latest['timestamp']
        if last_timestamp is None:
            dt = 0.02
        else:
            dt = (timestamp - last_timestamp) / 1000.0
        last_timestamp = timestamp

        filtered_angle = kalman.update(accel_angle, gyro_rate, dt)
        # Predict angle 20ms ahead
        predicted_angle = filtered_angle + gyro_rate * 0.02

        motor_speed = compute_control(predicted_angle)
        if last_addr:
            response = json.dumps({"motor_speed": motor_speed}).encode()
            sock.sendto(response, last_addr)

def compute_control(predicted_angle):
    # Replace with your PID logic
    return 0

threading.Thread(target=control_loop, daemon=True).start()

while True:
    data, addr = sock.recvfrom(4096)
    msg = json.loads(data.decode())
    msg_type = msg.get("type")
    if msg_type in handlers:
        handlers[msg_type](msg, addr)