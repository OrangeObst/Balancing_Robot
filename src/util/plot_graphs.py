import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self):
        pass

    def stackplot_pid_values(self, p_terms, i_terms, d_terms, timer, name="PID_stackplot"):
        """
        Create a stack plot of PID values.

        :param p_terms: List of P terms
        :param i_terms: List of I terms
        :param d_terms: List of D terms
        :param timer: Duration for the x-axis
        :param name: Name for the saved plot file
        """
        try:
            _, plts = plt.subplots(figsize=(10, 10))
            smaller_value = min(len(p_terms), len(i_terms), len(d_terms))
            time_values = np.linspace(0, timer, smaller_value)

            plts.set_xlim(0, timer)
            plts.set_alpha(0.5)
            color_map = ["#0000FF", "#00FF00", "#FF0000"]
            y = np.vstack([p_terms, i_terms, d_terms])
            plts.stackplot(time_values, y, colors=color_map)

            plt.title(f'Stackplot PID values {self.title}')
            plt.savefig(f'graphs/{name}')
        except Exception as e:
            print(f"Error in stackplot_pid_values: {e}")

    def subplot_p_i_d_values(self, pid_type, pid_terms, timer, data_range_percentile=99, 
                          unified_y_limit=True, name="PID_subplot"):
        """
        Create subplots for P, I, and D values.

        :param pid_type: Type of PID (e.g., 'Angle', 'Position')
        :param pid_terms: Dictionary containing PID terms (pterms, iterms, dterms)
        :param timer: Duration for the x-axis
        :param data_range_percentile: Percentile to determine Y-axis limits (symmetrical around zero)
        :param unified_y_limit: Boolean to control y-limit behavior
            - True: Use a global max for all subplots
            - False: Use individual local max for each subplot
        :param name: Name for the saved plot file
        """
        # try:
        _, plts = plt.subplots(2, 2, figsize=(12, 10))
        plts = [plts] if not isinstance(plts, np.ndarray) else plts
        plot_indices = [(0, 0), (0, 1), (1, 0), (1, 1)]

        if unified_y_limit:
            # Calculate global max/min for all subplots
            all_values = [val for _, values in pid_terms.items() for val in values]
            lower_lim = np.percentile(all_values, (100 - data_range_percentile) / 2)
            upper_lim = np.percentile(all_values, 100 - (100 - data_range_percentile) / 2)
            global_y_lim = max(abs(lower_lim), abs(upper_lim))

            # Apply global y-limit to all subplots
            for (i, j), (label, values) in zip(plot_indices, pid_terms.items()):
                plts[i, j].set_xlim(0, timer)
                plts[i, j].set_ylim(-global_y_lim, global_y_lim)

        else:
            # Calculate and apply individual local max/min for each subplot
            for (i, j), (label, values) in zip(plot_indices, pid_terms.items()):
                plts[i, j].set_xlim(0, timer)
                lower_lim = np.percentile(values, (100 - data_range_percentile) / 2)
                upper_lim = np.percentile(values, 100 - (100 - data_range_percentile) / 2)
                local_y_lim = max(abs(lower_lim), abs(upper_lim))
                if local_y_lim > 0:
                    plts[i, j].set_ylim(-local_y_lim, local_y_lim)

        for (i, j), (label, values) in zip(plot_indices, pid_terms.items()):
            x = np.linspace(0, timer, len(values))
            plts[i, j].plot(x, values)
            plts[i, j].grid(alpha=0.5)
            plts[i, j].axhline(0, color='black', lw=0.5, linestyle='--')
            pid_constant_label = label[:1].upper()
            pid_constant_value = self.pid_constants[pid_type].get(pid_constant_label, '')
            if pid_constant_label!= 'O':
                plts[i, j].set_title(f'{pid_type} {label.capitalize()} ({pid_constant_label} = {pid_constant_value})')
            else:
                plts[i, j].set_title(f'{pid_type} {label.capitalize()} (O = P+I-D)')

        plt.tight_layout()
        plt.savefig(f'graphs/{name}.png')
        # except Exception as e:
        #     print(f"Error in subplot_p_i_d_values: {e}")


    def plot_measurements(self, left_axis_key, left_axis_values, right_axis_key=None, right_axis_values=None, timer=10, name="Measurement_Plot"):
        """
        Plot multiple series on the left axis and optionally on the right axis.

        :param left_axis_key: Label for the left axis
        :param left_axis_values: Dictionary of lists of values for the left axis
        :param timer: Duration for the x-axis
        :param right_axis_key: [Optional] Label for the right axis
        :param right_axis_values: [Optional] Dictionary of lists of values for the right axis
        :param name: Output plot file name (default: "Measurement_Plot")
        """
        try:
            fig, left_ax = plt.subplots(figsize=(6.4, 4.8))

            if right_axis_values is not None:
                right_ax = left_ax.twinx()
                axes_values_list = [left_axis_values, right_axis_values]
                axes_keys_list = [left_axis_key, right_axis_key]
            else:
                axes_values_list = [left_axis_values]
                axes_keys_list = [left_axis_key]

            max_len = max((max(len(values) for values in axis_values.values()) if axis_values else 1 for axis_values in axes_values_list), default=1)
            max_values = [max(max(abs(x) for x in sublist) for sublist in axis_values.values()) if axis_values else 0 for axis_values in axes_values_list]

            time_values = np.linspace(0, timer, max_len)
            colors = plt.cm.tab20(range(sum(len(axis_values) for axis_values in axes_values_list)))
            counter = 0

            for ax, (axis_key, axis_values) in enumerate(zip(axes_keys_list, axes_values_list)):
                current_ax = right_ax if ax == 1 else left_ax
                for key, values in axis_values.items():
                    plot_values = values[:max_len]
                    time_values_plot = time_values[:len(plot_values)]
                    current_ax.plot(time_values_plot, plot_values, label=f"{key}", color=colors[counter])
                    counter += 1

            left_ax.set_xlim(0, timer)
            left_ax.set_ylim(-max_values[0], max_values[0])
            left_ax.set_xlabel('Time [s]')
            left_ax.set_ylabel(left_axis_key)

            if right_axis_values is not None:
                right_ax.set_ylim(-max_values[1], max_values[1])
                right_ax.set_ylabel(right_axis_key)

            fig.legend(loc='upper right', ncol=2)
            left_ax.axhline(y=0.0, linestyle='--')
            plt.grid(alpha=0.5)
            plt.savefig(f'graphs/{name}')
        except Exception as e:
            print(f"Error in plot_measurements: {e}")

    def plot_angles(self, data_sets, timer, colors=None, name="Angles"):
        """
        Plot angles over time.

        :param data_sets: List of tuples containing data and labels [(data, label), ...]
        :param timer: Duration for the x-axis
        :param colors: List of colors for the plots
        :param name: Name for the saved plot file
        """
        try:
            _, left_ax = plt.subplots(figsize=(6.4, 4.8))

            if colors is None:
                colors = ["#a2a2a2", "#4e4e4e", "#22e032"]

            for i, (data, label) in enumerate(data_sets):
                time_values = np.linspace(0, timer, len(data))
                color = colors[i % len(colors)]
                left_ax.plot(time_values, data, color=color, label=label)

            left_ax.axhline(y=0, color='k', linestyle='--')
            left_ax.set_xlim(0, timer)
            left_ax.set_xlabel('Time [s]')
            left_ax.set_ylabel('Angle [°]')
            plt.legend()
            plt.savefig(f'graphs/{name}')
        except Exception as e:
            print(f"Error in plot_angles: {e}")

    def print_averages(self, collected_data):
        """
        Print average values of collected data.

        :param collected_data: Dictionary of collected data
        """
        try:
            print("----- Average values -----")
            for key, data in collected_data.items():
                if "terms" in key:
                    avg_pterms, avg_iterms, avg_dterms = np.mean(data, axis=0)
                    print(f'{key:18}: P: {avg_pterms:6.5f}, I: {avg_iterms:6.5f}, D: {avg_dterms:6.5f}')
                else:
                    avg = np.mean(data, axis=0)
                    print(f'{key:18}: {avg:.5f}')
        except Exception as e:
            print(f"Error in print_averages: {e}")

if __name__ == "__main__":
    angle_pid_const = [10, 5, 1]
    pos_pid_const = [1, 5, 10]
    plotter = Plotter(angle_pid_const, pos_pid_const)
    left_values = {'1': [1, 2, 3, 4, 5], '2': [5, 4, 3, 2, 1]}
    right_values = {'steps': [100, 200, 300, 400, 500]}
    plotter.plot_measurements('Left Axis', left_values, 10, 'Right Axis', right_values)