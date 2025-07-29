import csv
import time
import numpy as np
from pathlib import Path

class DataCollector:
    def __init__(self):
        self._data = {}
        self._history = []

    def collect(self, **kwargs):
        self._data.update(kwargs)

    def snapshot(self):
        self._history.append(self._data.copy())
        self._data.clear()

    def get_latest(self):
        return self._history[-1] if self._history else {}

    def get_all(self):
        return self._history

    def clear(self):
        self._data.clear()
        self._history.clear()

    def emit(self, emitter):
        if self._data:
            emitter(self._data.copy())
            self._data.clear()

    def log(self, logger):
        if self._history:
            for entry in self._history:
                logger(entry)
            self._history.clear()


    def print_averages(self):
        print("----- Average values -----")
        for key, data in self.data.items():
            if isinstance(data, dict):
                print(f'{key:18}:')
                for pid_key, pid_data in data.items():
                    try:
                        avg = np.mean(pid_data)
                        print(f'  {pid_key:15}: {avg:.5f}')
                    except Exception as e:
                        print(f'oops: {e}')
                        print(pid_key, pid_data)
            elif "timestamped" in key:
                pass
            else:
                try:
                    avg = np.mean(data, axis=0)
                    print(f'{key:18}: {avg:.5f}')
                except Exception as e:
                    print(f'oops: {e}')
                    print(key, data)
    
    def write_data_to_csv(self, destination_folder=None):
        if destination_folder is None:
            project_root = Path(__file__).resolve().parents[2]  # project/src/util -> go up 2
            destination_folder = project_root / "Measurements"
        else:
            destination_folder = Path(destination_folder)

        destination_folder.mkdir(parents=True, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = destination_folder / f"log_data_{timestamp}.csv"

        if not self._history:
            print("No data to write.")
            return

        # Aggregate all keys for consistent header
        all_keys = sorted({key for entry in self._history for key in entry})

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=all_keys)
            writer.writeheader()
            for entry in self._history:
                writer.writerow(entry)

        print(f"Data written to {filename}")


if __name__ == "__main__":
    collector = DataCollector()

