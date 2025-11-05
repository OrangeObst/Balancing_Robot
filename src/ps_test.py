import psutil
import time

print(psutil.cpu_count())
print(psutil.cpu_freq())
print(psutil.cpu_percent())
print(psutil.cpu_stats())
print(psutil.cpu_times())
print(psutil.cpu_times_percent())
print(psutil.virtual_memory())

print(psutil.virtual_memory().available * 100 / psutil.virtual_memory().total)

cpu_usage = psutil.cpu_percent(interval=1, percpu=True)
print(f"CPU Usage: {cpu_usage}%")

network = psutil.net_io_counters(pernic=True)
print(f"Bytes Sent: {network}, Bytes Received: {network}")

test_list = [1, 2, 3, 4, 5]
print(f"Original List: {test_list}")
print(f'Shortened List: {test_list[-3:]}')