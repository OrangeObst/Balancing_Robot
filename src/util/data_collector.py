import csv
import os

import numpy as np


class DataCollector:
    def __init__(self):
        self.data = {
            'angles': [],
            'gyro_angles': [],
            'accel_angles': [],
            'f_accel_angles': [],
            'angle_pid_terms': {
                'p_terms': [],
                'i_terms': [],
                'd_terms': [],
                'output': []
            },
            'pos_pid_terms': {
                'p_terms': [],
                'i_terms': [],
                'd_terms': [],
                'output': []
            },
            'timestamped_angles': [],
            'steps': [],
            'target_angles': []
        }

        self.pid_key_map = {
            'angle': 'angle_pid_terms',
            'pos': 'pos_pid_terms'
        }


    def update_filename(self, filename):
        self.data['filename'] = filename


    def log_data(self, key, value):
        self.data.setdefault(key, []).append(value)


    def log_angle_data(self, angle, gyro_angle, accel_angle, f_accel_angle):
        self.data['angles'].append(angle)
        self.data['gyro_angles'].append(gyro_angle)
        self.data['accel_angles'].append(accel_angle)
        self.data['f_accel_angles'].append(f_accel_angle)


    def log_pid_data(self, pid, p_terms, i_terms, d_terms, output):
        if pid not in self.pid_key_map:
            raise ValueError(f"Invalid pid: {pid}. Must be one of: {list(self.pid_key_map.keys())}")
        
        pid_key = self.pid_key_map[pid]
        self.data[pid_key]['p_terms'].append(p_terms)
        self.data[pid_key]['i_terms'].append(i_terms)
        self.data[pid_key]['d_terms'].append(d_terms)
        self.data[pid_key]['output'].append(output)


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
            


    def get_all_collected_data(self):
        return self.data
    

    def get_next_log_file_name(self, destination_folder='/home/newPi/Desktop/'):
        # Ensure the destination folder exists
        os.makedirs(destination_folder, exist_ok=True)
        
        base_name = "log_data_"
        extension = ".csv"
        i = 1
        while os.path.exists(os.path.join(destination_folder, f"{base_name}{i}{extension}")):
            i += 1
        return os.path.join(destination_folder, f"{base_name}{i}{extension}")
    
    def write_timestamped_angles_to_csv(self, destination_folder='/home/newPi/Desktop/'):
        filename = self.get_next_log_file_name(destination_folder)
        with open(filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Milliseconds since start', 'Angle'])
            for log in self.data['timestamped_angles']:
                csv_writer.writerow(log)


if __name__ == "__main__":
    collector = DataCollector()
    print(collector.get_next_log_file_name('/home/newPi/Desktop/Balance_Bot/Stepper_Bot/Messungen'))
