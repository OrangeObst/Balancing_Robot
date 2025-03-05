from functools import wraps
from collections import defaultdict
import os
import json

class DataCollector:
    _collected_data = defaultdict(list)

    def _collect_data_decorator(func):
        """Decorator to collect data on the decorated function's execution."""
        @wraps(func)
        def wrapper(*args, **kwargs):
            result = func(*args, **kwargs)
            # data_to_collect = {
            #     'args': args,
            #     'kwargs': kwargs,
            #     # Add more metrics as needed (e.g., time.perf_counter() for timing)
            # }
            DataCollector._collected_data[func.__name__].append(result)
            return func(*args, **kwargs)
        return wrapper

    def _store_data(func_name, args, kwargs, result):
        """
        Generalized to store multiple result values.
        
        :param func_name: Name of the function.
        :param args: Positional arguments.
        :param kwargs: Keyword arguments.
        :param result: Dictionary with variable names as keys and their values.
        """
        # Store each result value under its variable name
        for key, value in result.items():
            if key not in DataCollector._collected_data:
                DataCollector._collected_data[key] = []
            DataCollector._collected_data[key].append(value)


    def get_collected_data(var_name=None):
        if var_name:
            return DataCollector._collected_data[var_name]
        return dict(DataCollector._collected_data)


    def clear_collected_data(var_name=None):
        if var_name:
            DataCollector._collected_data[var_name].clear()
        else:
            DataCollector._collected_data.clear()

    def _get_next_log_file_name(destination_folder='/home/newPi/Desktop/Balancing_Robot/graphs/', extension=".txt"):
        base_name = "log_data_"
        extension = extension
        i = 1
        while os.path.exists(os.path.join(destination_folder, f"{base_name}{i}{extension}")):
            i += 1
        return os.path.join(destination_folder, f"{base_name}{i}{extension}")

    def print_to_txt_file(destination_folder='/home/newPi/Desktop/Balancing_Robot/graphs/'):
        filename = DataCollector._get_next_log_file_name(destination_folder, extension=".txt")
        with open(filename, 'w') as txtfile:
            for key, values in DataCollector._collected_data.items():
                txtfile.write(f"{key}:\n")
                txtfile.write(f"  ")
                for value in values:
                    txtfile.write(f"{value}, ")
                txtfile.write("\n")


    def print_averages():
        print(DataCollector._collected_data)
    #     print("----- Average values -----")
    #     for key, data in DataCollector.collected_data.items():
    #         if isinstance(data, dict):
    #             print(f'{key:18}:')
    #             for pid_key, pid_data in data.items():
    #                 try:
    #                     avg = np.mean(pid_data)
    #                     print(f'  {pid_key:15}: {avg:.5f}')
    #                 except Exception as e:
    #                     print(f'oops: {e}')
    #                     print(pid_key, pid_data)
    #         elif "timestamped" in key:
    #             pass
    #         else:
    #             try:
    #                 avg = np.mean(data, axis=0)
    #                 print(f'{key:18}: {avg:.5f}')
    #             except Exception as e:
    #                 print(f'oops: {e}')
    #                 print(key, data)