from smbus2 import SMBus
from mpu6050 import MyMPU6050
from codetiming import Timer
from multiprocessing import Process, Pipe, Manager
import paho.mqtt.client as mqtt
import json
import time
import sys

# MPU6050 setup
bus = SMBus(1)
mpu = MyMPU6050(bus)

mpu.set_dlpf_cfg(2)
mpu.set_smplrt_div(4)

# MQTT setup
broker = "10.224.64.29"
port = 1883
topic = "mpu6050/data"

def on_connect(client, userdata, flags, rc, *args, **kwargs):
    print(f"Connected with result code {rc}")
    client.subscribe(topic)

# @Timer(name="on_publish", text="on_publish: {milliseconds:.6f}ms")
def on_publish(client, userdata, mid, *args, **kwargs):
    print(f"Message {mid} published.")

# @Timer(name="on_message", text="on_message: {milliseconds:.6f}ms")
def on_message(client, userdata, msg):
    global message_counter
    message = msg.payload.decode()
    message = json.loads(message)
    latency = time.time() - float(message[1])
    print(f"Received message nr. {message_counter}: {message[0]} with latency {latency}")
    # print(f"Round-trip latency: {latency:.6f} seconds")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.on_connect = on_connect
client.on_publish = on_publish

client.connect(broker, port, 60)
client.loop_start()


message_counter = 0

try:
    end_time = time.time() + 1800
    counter = 0
    while(time.time() < end_time):
        start_time = time.time()
        loop_time = start_time + 0.002
        
        data = mpu.MPU_ReadData()

        payload = json.dumps([data, start_time, counter])
        counter += 1
        result = client.publish(topic, payload, qos=0)
        message_counter += 1
        
        remaining_time = loop_time - time.time()
        if remaining_time > 0:
            time.sleep(remaining_time)
        

    data = 'Done'
    payload = json.dumps([data, start_time, counter])
    client.publish(topic, payload)
    print("Should be done")
    print(f"Haha {counter}")

except KeyboardInterrupt:
    print("Exiting...")
finally:
    client.loop_stop()
    client.disconnect()