import threading
from queue import Queue
import time

class MpuDataAverager(threading.Thread):
    def __init__(self, mpu, queue, sample_rate=0.003):
        super().__init__()
        self.mpu = mpu
        self.queue = queue
        self.sample_rate = sample_rate
        self.num_samples = int(0.01 / sample_rate)
        self.samples = []

    def run(self):
        summed_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        while True:
            
            data = self.mpu.get_all_data()
            self.samples.append(data)
            if len(self.samples) >= self.num_samples:
                for values in self.samples:
                    summed_values[0] += values[0]
                    summed_values[1] += values[1]
                    summed_values[2] += values[2]
                    summed_values[3] += values[3]
                    summed_values[4] += values[4]
                    summed_values[5] += values[5]

                averaged_data = self._calculate_averaged_data(summed_values)
                self.queue.put(averaged_data)

                self.samples = []
                summed_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            time.sleep(self.sample_rate)

    def _calculate_averaged_data(self, samples):
        return [value / self.num_samples for value in samples]
