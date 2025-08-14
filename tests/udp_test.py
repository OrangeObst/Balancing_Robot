import sys
import os
import time
import threading
import time
import statistics
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from network.udp_client import UdpClient

BROKER = "10.224.64.29"
PORT = 17002
TEST_COUNT = 1000
DELAY = 0.005

class UdpTest:
    def __init__(self, server_host, server_port, messages=10, delay=0.1):
        self.client = UdpClient(server_host, server_port)
        self.messages = messages
        self.counter = 0
        self.latencies = []
        self.start_time = None
        self.end_time = None
        self.receiving_thread = None
        self.run_thread = True
        self.delay = delay

    def start_test(self):
        self.start_time = time.time()
        self.receiving_thread = threading.Thread(target=self.receive_messages)
        self.receiving_thread.start()
        for _ in range(self.messages):
            message = time.time()
            self.client.send(message)
            self.counter += 1
            time.sleep(self.delay)
        self.client.send('done')

    def receive_messages(self):
        while self.run_thread:
            data = self.client.receive_messages()
            if data == 'done':
                self.run_thread = False
                break
            if data is not None:
                latency = time.time() - data
                self.latencies.append(latency)

    def calculate_stats(self):
        if self.latencies:
            self.min_latency = min(self.latencies)
            self.max_latency = max(self.latencies)
            self.avg_latency = sum(self.latencies) / len(self.latencies)
            self.jitter = statistics.stdev(self.latencies)
            self.packet_loss_rate = (self.counter - len(self.latencies)) / self.counter
        else:
            self.min_latency = self.max_latency = self.avg_latency = self.jitter = self.packet_loss_rate = "N/A"

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

    def kill_threads(self):
        self.run_thread = False


if __name__ == "__main__":
    udp_test = UdpTest(BROKER, PORT, TEST_COUNT, DELAY)
    try:
        udp_test.start_test()
        udp_test.receiving_thread.join()
        # while not udp_test.run_thread:
        #     time.sleep(0.1)  # Wait for the receiving thread to complete
    except KeyboardInterrupt:
        print("KeyboardInterrupt has been called")
    finally:
        udp_test.kill_threads()
        udp_test.close()

    udp_test.calculate_stats()
    udp_test.print_stats()
