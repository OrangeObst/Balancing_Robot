import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self, apid, ppid, timer=10):
        self.pid_constants = {
            'Angle': {
                'P': apid[0],
                'I': apid[1],
                'D': apid[2],
            },
            'Position': {
                'P': ppid[0],
                'I': ppid[1],
                'D': ppid[2]
            }
        }
        self.title = f"(AP={apid[0]}, AI={apid[1]}, AD={apid[2]}, PP={ppid[0]}, PI={ppid[1]}, PD={ppid[2]})"
        self.timer = timer
        self.color_cycle = [
            '#0077BB', '#44A0E6', '#0099CC',
            '#FF69B4', '#FFFF00', '#FFA07A',
            '#8B0A0A', '#6495ED', '#4B0082',
            '#32CD32', '#008000',
            '#964B00', '#666666', '#C5CAE9'
        ]

    def stackplot_pid_values(self, p_terms, i_terms, d_terms, name="PID_stackplot"):
        fig, plts = plt.subplots(figsize = (10, 10))

        # Need size of smallest dataset to be able to plot properly
        smaller_value = min(len(p_terms), len(i_terms), len(d_terms))
        time_values = np.linspace(0, self.timer, smaller_value)

        plts.set_xlim(0, self.timer)
        
        plts.set_alpha(0.5)
        color_map = ["#0000FF", "#00FF00", "#FF0000"]
        y = np.vstack([p_terms, i_terms, d_terms])
        plts.stackplot(time_values, y, colors = color_map)
        
        plt.title(f'Stackplot PID values {self.title}')
        plt.savefig(f'graphs/{name}')


    def subplot_p_i_d_values(self, pid_type, pid_terms, data_range_percentile=99, name="PID_subplot"):
        """
        Parameters:
        - pid_type: Type of PID (e.g., 'Angle', 'Position')
        - pid_terms: Dictionary containing PID terms (pterms, iterms, dterms)
        - data_range_percentile: Percentile to determine Y-axis limits (symmetrical around zero)
        - name: Name for the saved plot file
        """
        fig, plts = plt.subplots(2, 2, figsize=(12, 10))
        plts = [plts] if not isinstance(plts, np.ndarray) else plts  # Handle both cases for plts
        plot_indices = [(0, 0), (0, 1), (1, 0), (1, 1)]

        # Calculate symmetrical y-limits based on the data range percentile
        all_values = [val for _, values in pid_terms.items() for val in values]
        lower_lim = np.percentile(all_values, (100 - data_range_percentile) / 2)
        upper_lim = np.percentile(all_values, 100 - (100 - data_range_percentile) / 2)
        y_lim = max(abs(lower_lim), abs(upper_lim))

        for (i, j), (label, values) in zip(plot_indices, pid_terms.items()):
            plts[i, j].set_xlim(0, self.timer)
            plts[i, j].set_ylim(-y_lim, y_lim)
            x = np.linspace(0, self.timer, len(values))
            plts[i, j].plot(x, values)
            plts[i, j].grid()
            
            # Zero line for better visibility
            plts[i, j].axhline(0, color='black', lw=0.5, linestyle='--')
            
            if label in ['p_terms', 'i_terms', 'd_terms']:
                pid_constant_label = label[:1].upper()
                pid_constant_value = self.pid_constants[pid_type][pid_constant_label]
                plts[i, j].set_title(f'{pid_type} {label.capitalize()} ({pid_constant_label} = {pid_constant_value})')
            else:
                plts[i, j].set_title(f'{pid_type} {label.capitalize()}')
        
        plt.tight_layout()  
        plt.savefig(f'graphs/{pid_type}_{name}.png')


    def plot_measurements(
        self,
        left_axis_key: str,
        left_axis_values: dict[list[float]],
        right_axis_key: str,
        right_axis_values: dict[list[float]],
        name: str = "Measurement_Plot"
    ):
        """
        Plot multiple series on the left axis and multiple series on the right axis.
        :param left_axis_key: Label for the left axis
        :param left_axis_values: List of lists of values for the left axis
        :param right_axis_key: Label for the right axis
        :param right_axis_values: List of lists of values for the right axis
        :param name: Output plot file name (default: "Measurement_Plot")
        """

        fig, left_ax = plt.subplots(figsize=(6.4, 4.8))
        right_ax = left_ax.twinx()

        max_len = max(
            (max(len(values) for values in left_axis_values.values()) if left_axis_values else 1,
            max(len(values) for values in right_axis_values.values()) if right_axis_values else 1),
            default=1
        )
        max_value_left = max(max(abs(x) for x in sublist) for sublist in left_axis_values.values())
        max_value_right = max(max(abs(x) for x in sublist) for sublist in right_axis_values.values())

        time_values = np.linspace(0, self.timer, max_len)

        colors = plt.cm.tab20(range(len(left_axis_values)+len(right_axis_values)))
        counter = 0
        for key, values in left_axis_values.items():
            plot_values = values[:max_len]
            time_values_plot = time_values[:len(plot_values)]
            left_ax.plot(time_values_plot, plot_values, label=f"{key}", color=colors[counter])
            counter += 1

        for key, values in right_axis_values.items():
            plot_values = values[:max_len]
            time_values_plot = time_values[:len(plot_values)]
            right_ax.plot(time_values_plot, plot_values, label=f"{key}", color=colors[counter])
            counter += 1

        left_ax.set_xlim(0, self.timer)
        left_ax.set_ylim(-max_value_left, max_value_left)
        left_ax.set_xlabel('Time [s]')
        left_ax.set_ylabel(left_axis_key)
        right_ax.set_ylabel(right_axis_key)
        right_ax.set_ylim(-max_value_right, max_value_right)

        fig.legend(handles=left_ax.lines + right_ax.lines, 
        labels=[f"{key}" for key, values in left_axis_values.items()] + [f"{key}" for key, values in right_axis_values.items()], 
        loc='upper right', ncol=2)

        left_ax.axhline(y = 0.0, linestyle = '--')
        # plt.title(f'Measurements {self.title}')
        plt.savefig(f'graphs/{name}')


    def plot_angles(self, data_sets, colors=None, name="Angles"):
        fig, left_ax = plt.subplots(figsize = (6.4, 4.8))

        if colors is None:
            colors = ["#a2a2a2", "#4e4e4e", "#22e032"]

        for i, [data, label] in enumerate(data_sets):
            time_values = np.linspace(0, self.timer, len(data))
            color = colors[i % len(colors)]
            left_ax.plot(time_values, data, color=color, label=label)

        # avg_angle = np.mean(self.collected_data['angles'], axis=0)
        left_ax.axhline(y=0, color='k', linestyle='--')
        left_ax.set_xlim(0, self.timer)

        left_ax.set_xlabel('Zeit [s]')
        left_ax.set_ylabel('Winkel [°]')

        plt.legend()
        # plt.title(f'Accel- and Gyroangles {self.title}')
        plt.savefig(f'graphs/{name}')


    def print_averages(self, collected_data):
        print("----- Average values -----")
        for key, data in collected_data.items():
            if "terms" in key:
                avg_pterms, avg_iterms, avg_dterms = np.mean(data, axis=0)
                print(f'{key:18}: P: {avg_pterms:6.5f}, I: {avg_iterms:6.5f}, D: {avg_dterms:6.5f}')
            else:
                avg = np.mean(data, axis=0)
                print(f'{key:18}: {avg:.5f}')


if __name__ == "__main__":
    angle_pid_const = [
        10,
        5,
        1
    ]
    pos_pid_const = [
        1,
        5,
        10
    ]
    plotter = Plotter(angle_pid_const, pos_pid_const, 10)
    left_values = [[1,2,3,4,5],'1',[5,4,3,2,1],'2']
    right_values = [[100,200,300,400,500], 'steps']
    print(left_values + right_values)
    # plotter.plot_measurements(left_values, right_values)