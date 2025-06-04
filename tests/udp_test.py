import queue
import sys
import os
import time
import threading

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.util.udp_client import UdpClient

import time
import statistics

class UdpTest:
    def __init__(self, server_host, server_port, test_count=10, delay=0.1):
        self.client = UdpClient(server_host, server_port)
        self.test_count = test_count
        self.latencies = []
        self.start_time = None
        self.end_time = None
        self.receiving_thread = None
        self.receiving_done = False
        self.delay = delay

    def start_test(self):
        self.start_time = time.time()
        self.receiving_thread = threading.Thread(target=self.receive_messages)
        self.receiving_thread.start()
        for _ in range(self.test_count):
            message = time.time()
            self.client.send(message)
            time.sleep(self.delay)
        self.client.send('done')

    def receive_messages(self):
        while not self.receiving_done:
            data = self.client.receive_messages()
            if data == 'done':
                self.receiving_done = True
                break
            if data:
                latency = time.time() - data
                self.latencies.append(latency)

    def calculate_stats(self):
        if self.receiving_done:
            if self.latencies:
                self.min_latency = min(self.latencies)
                self.max_latency = max(self.latencies)
                self.avg_latency = sum(self.latencies) / len(self.latencies)
                self.jitter = statistics.stdev(self.latencies)
                self.packet_loss_rate = (self.test_count - len(self.latencies)) / self.test_count
            else:
                self.min_latency = self.max_latency = self.avg_latency = self.jitter = self.packet_loss_rate = "N/A"
        else:
            print("Receiving thread has not completed yet.")

    def print_stats(self):
        print(f"Minimum latency: {self.min_latency:.4f} seconds")
        print(f"Maximum latency: {self.max_latency:.4f} seconds")
        print(f"Average latency: {self.avg_latency:.4f} seconds")
        print(f"Jitter: {self.jitter:.4f} seconds")
        print(f"Packet loss rate: {self.packet_loss_rate * 100:.2f}%")

    def close(self):
        self.client.close()
        if self.receiving_thread:
            self.receiving_thread.join()

BROKER = "10.224.64.29"
PORT = 17002
TEST_COUNT = 1000
DELAY = 0.005

udp_test = UdpTest(BROKER, PORT, TEST_COUNT, DELAY)
udp_test.start_test()
while not udp_test.receiving_done:
    time.sleep(0.1)  # Wait for the receiving thread to complete
udp_test.calculate_stats()
udp_test.print_stats()
udp_test.close()
