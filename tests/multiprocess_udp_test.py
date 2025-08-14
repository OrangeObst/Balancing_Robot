import sys
import os
import time
import multiprocessing
import statistics

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from network.udp_client import UdpClient

BROKER = "10.224.64.29"
PORT = 17002
TEST_COUNT = 1000
DELAY = 0.005

class UdpTest:
    def __init__(self, server_host, server_port, messages=10, delay=0.1):
        self.server_host = server_host
        self.server_port = server_port
        self.messages = messages
        self.counter = 0
        self.start_time = None
        self.end_time = None
        self.receiving_process = None
        self.delay = delay

        self.run_process = multiprocessing.Value('b', True)
        self.manager = multiprocessing.Manager()
        self.latencies = self.manager.list()

    def start_test(self):
        self.start_time = time.time()
        self.client = UdpClient(self.server_host, self.server_port)
        self.receiving_process = multiprocessing.Process(target=self.receive_messages, args=[self.client, self.latencies, self.run_process])
        self.receiving_process.start()
        for _ in range(self.messages):
            message = time.time()
            self.client.send(message)
            self.counter += 1
            time.sleep(self.delay)
        self.client.send('done')

    def receive_messages(self, client, latencies, run_process):
        while run_process.value:
            data = client.receive_messages()
            if data == 'done':
                self.run_process.value = False
                break
            if data is not None:
                latency = time.time() - data
                latencies.append(latency)

    def calculate_stats(self):
        latencies = list(self.latencies)
        if len(latencies) > 0:
            self.min_latency = min(latencies)
            self.max_latency = max(latencies)
            self.avg_latency = sum(latencies) / len(latencies)
            self.jitter = statistics.stdev(latencies)
            self.packet_loss_rate = (self.counter - len(latencies)) / self.counter
        else:
            self.min_latency = self.max_latency = self.avg_latency = self.jitter = self.packet_loss_rate = "N/A"

    def print_stats(self):
        print(f"Minimum latency: {self.min_latency:.4f} seconds")
        print(f"Maximum latency: {self.max_latency:.4f} seconds")
        print(f"Average latency: {self.avg_latency:.4f} seconds")
        print(f"Jitter: {self.jitter:.4f} seconds")
        print(f"Packet loss rate: {self.packet_loss_rate * 100:.4f}%")

    def close(self):
        if self.receiving_process:
            self.receiving_process.join()
        self.client.close()

    def kill_process(self):
        self.run_process.value = False


if __name__ == "__main__":
    udp_test = UdpTest(BROKER, PORT, TEST_COUNT, DELAY)
    try:
        udp_test.start_test()
        udp_test.receiving_process.join()
    except KeyboardInterrupt:
        print("KeyboardInterrupt has been called")
    finally:
        udp_test.kill_process()
        udp_test.close()

    udp_test.calculate_stats()
    udp_test.print_stats()
