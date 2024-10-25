import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self):
        pass

    def stackplot_pid_values(self, p_values, i_values, d_values):
        fig, plts = plt.subplots(figsize = (10, 10))

        # Need size of smallest dataset to be able to plot properly
        smaller_value = min(len(p_values), len(i_values), len(d_values))
        time_values = np.linspace(0, 10, smaller_value)

        plts.set_xlim(0, 10)
        plts.set_ylim(-100, 100)
        
        color_map = ["#0000FF", "#00FF00", "#FF0000"]
        y = np.vstack([p_values, i_values, d_values])
        plts.stackplot(time_values, y, colors = color_map)

        plt.savefig('stackplot.png')

    def subplot_p_i_d_values(self, p_values, i_values, d_values):
        fig, plts = plt.subplots(3, figsize = (10, 10))

        # Need size of smallest dataset to be able to plot properly
        smaller_value = min(len(p_values), len(d_values))
        time_values = np.linspace(0, 10, smaller_value)

        plts[0].set_xlim(0, 10)
        plts[0].set_ylim(-100, 100)
        plts[1].set_xlim(0, 10)
        plts[1].set_ylim(-100, 100)
        plts[2].set_xlim(0, 10)
        plts[2].set_ylim(-100, 100)

        p = np.vstack([p_values])
        i = np.vstack([i_values])
        d = np.vstack([d_values])

        plts[0].stackplot(time_values, p)
        plts[0].set_title('Proportional')
        plts[1].stackplot(time_values, i)
        plts[1].set_title('Integral')
        plts[2].stackplot(time_values, d)
        plts[2].set_title('Derivativ')

        plt.savefig('subplot.png')

    def plot_angle_speed(self, angles, speeds, setpoint):
        fig, left_ax = plt.subplots(figsize = (10, 10))
        right_ax = left_ax.twinx()

        time_values = np.linspace(0, 10, len(angles))
        p1, = left_ax.plot(time_values, angles, "b-")
        time_values = np.linspace(0, 10, len(speeds))
        p2, = right_ax.plot(time_values, speeds, "r-")
        left_ax.axhline(y=setpoint, color='k', linestyle='--', label=f'Balancing point at {setpoint}°')

        left_ax.set_xlim(0, 10)
        left_ax.set_ylim(-11, 11)
        right_ax.set_ylim(-110, 110)

        left_ax.set_xlabel('Time (s)')
        left_ax.set_ylabel('Angle (°)')
        right_ax.set_ylabel('Speed')

        left_ax.yaxis.label.set_color(p1.get_color())
        right_ax.yaxis.label.set_color(p2.get_color())

        plt.savefig('speedplot.png')