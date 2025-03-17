import sys
import os
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.util.plot_graphs import Plotter

def generate_dynamic_values(num_points, timer, frequency=1):
    """Generates dynamic values for p_terms, i_terms, d_terms, and output with a given frequency."""
    time_values = np.linspace(0, timer, num_points)
    p_terms = np.sin(frequency * time_values)
    i_terms = np.cos(frequency * time_values)
    d_terms = np.sin(frequency * time_values)
    output = np.arctan2(np.sin(frequency * time_values), np.cos(frequency * time_values))
    return time_values, p_terms, i_terms, d_terms, output

# Generate dynamic values with increased frequency
num_points = 1000
timer = 10
frequency = 50  # Increase this value to squash the wave together more
ms, p_terms, i_terms, d_terms, output = generate_dynamic_values(num_points, timer, frequency)

# Angle PID
AP = 17
AI = 0.01
AD = 0.00
# Position PID
PP = 0.0005
PI = 0.0
PD = 0.0006

angle_pid_const = [AP, AI, AD]
pos_pid_const = [PP, PI, PD]

plotter = Plotter(angle_pid_const, pos_pid_const)
plotter.subplot_p_i_d_values('Angle', {'p_terms': p_terms, 'i_terms': i_terms, 'd_terms': d_terms, 'output': output}, timer, 100, 'PID_Terms')