import time
import multiprocessing
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from network.udp_client import UdpClient
from src.util.timed_task import TimedTask

BROKER = '10.224.64.29'
PORT = 17002

class MultiprocessingTest:
    def __init__(self):
        self.client = UdpClient(BROKER, PORT)
        self.manager = multiprocessing.Manager()
        self.message_list = self.manager.list()
        self.run_process = multiprocessing.Value('b', True)
        self.sending_process = multiprocessing.Process(target=self._send_data_handler, args=[self.client, self.run_process])
        self.receiving_process = multiprocessing.Process(target=self._receive_data_handler, args=[self.client, self.message_list, self.run_process])
        self.calculation_process = multiprocessing.Process(target=self._data_handler, args=[self.message_list, self.run_process],)
        self.control_task = TimedTask(delay=0.02, run=self._control_loop_handler)


    def start_processes(self):
        self.sending_process.start()
        self.receiving_process.start()
        self.calculation_process.start()
    
    def _send_data_handler(self, client, run_process):
        while run_process.value:
            data = time.time()
            client.send(data)
            time.sleep(0.1)
        client.send('done')

    def _receive_data_handler(self, client, messages, run_process):
        while run_process.value:
            data = client.receive_messages()
            if data == 'done':
                run_process.value = False
                break
            if data:
                messages.append(data)

    def _data_handler(self, messages, run_process):
        while run_process.value:
            time.sleep(0.02)
            data = list(messages)
            messages[:] = []
            print(f'data: {data}')
            

    def _kill_threads(self):
        self.run_process.value = False

    def _control_loop_handler(self, now, dt):
        data = self._get_mpu_data()
        print(f'data: {data}')


    def _get_mpu_data(self):
        data = list(self.message_list)
        return data


if __name__ == "__main__":
    test = MultiprocessingTest()
    test.start_processes()
    end_time = time.time() + 5
    while time.time() < end_time:
        pass
    test._kill_threads()
    test.sending_process.join()
    test.receiving_process.join()
    