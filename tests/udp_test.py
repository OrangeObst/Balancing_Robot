import sys
import os
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.util.udp_client import UdpClient

BROKER = "10.224.64.29"
PORT = 17002

udp_client = UdpClient(BROKER, PORT)

udp_client.start_sending()
udp_client.start_receiving()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopping the client...")
    udp_client.close()
