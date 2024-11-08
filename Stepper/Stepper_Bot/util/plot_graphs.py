import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self, apid, spid, collected_data, timer=10):
        self.apid = apid
        self.spid = spid
        self.collected_data = collected_data
        self.title = f"(AP={self.apid[0]}, AI={self.apid[1]}, AD={self.apid[2]}, VP={self.spid[0]}, VI={self.spid[1]}, VD={self.spid[2]})"
        self.timer = timer

    def stackplot_pid_values(self, pterms, iterms, dterms, name="PID_stackplot"):
        fig, plts = plt.subplots(figsize = (10, 10))

        # Need size of smallest dataset to be able to plot properly
        smaller_value = min(len(pterms), len(iterms), len(dterms))
        time_values = np.linspace(0, 10, smaller_value)

        plts.set_xlim(0, self.timer)
        plts.set_ylim(-100, 100)
        
        plts.set_alpha(0.5)
        color_map = ["#0000FF", "#00FF00", "#FF0000"]
        y = np.vstack([pterms, iterms, dterms])
        plts.stackplot(time_values, y, colors = color_map)
        
        plt.title(f'Stackplot PID values {self.title}')
        plt.savefig(name)

    def subplot_p_i_d_values(self, pid_const, pterms, iterms, dterms, ylim=10, name="PID_subplot"):
        fig, plts = plt.subplots(3, figsize = (10, 10))

        plts[0].set_xlim(0, self.timer)
        plts[0].set_ylim(-ylim, ylim)
        plts[1].set_xlim(0, self.timer)
        plts[1].set_ylim(-ylim, ylim)
        plts[2].set_xlim(0, self.timer)
        plts[2].set_ylim(-ylim, ylim)

        p = np.vstack([pterms])
        i = np.vstack([iterms])
        d = np.vstack([dterms])

        plts[0].stackplot(np.linspace(0, self.timer, len(pterms)), p)
        plts[0].set_title(f'Proportional (P ={pid_const[0]})')
        plts[1].stackplot(np.linspace(0, self.timer, len(iterms)), i)
        plts[1].set_title(f'Integral (I = {pid_const[1]})')
        plts[2].stackplot(np.linspace(0, self.timer, len(dterms)), d)
        plts[2].set_title(f'Derivative (D = {pid_const[2]})')

        plt.savefig(name)

    def plot_angle_speed(self, angles, velocities, accelerations=None):
        fig, left_ax = plt.subplots(figsize = (10, 10))
        right_ax = left_ax.twinx()

        time_values = np.linspace(0, self.timer, len(angles))
        p1, = left_ax.plot(time_values, angles, "b-", label="Angle")

        time_values = np.linspace(0, self.timer, len(velocities))
        p2, = right_ax.plot(time_values, velocities, "r-", label="Velocities")

        if accelerations is not None:
            time_values = np.linspace(0, self.timer, len(accelerations))
            right_ax.plot(time_values, accelerations, "g-", label="Accelerations")

        left_ax.axhline(y=self.collected_data['angle_setpoint'], color='k', linestyle='--', label=f"Balancing point at {self.collected_data['angle_setpoint']}°")

        left_ax.set_xlim(0, self.timer)
        left_ax.set_ylim(-5, 5)
        right_ax.set_ylim(-50, 50)

        left_ax.set_xlabel('Time (s)')
        left_ax.set_ylabel('Angle (°)')
        right_ax.set_ylabel('Speed')

        left_ax.yaxis.label.set_color(p1.get_color())
        right_ax.yaxis.label.set_color(p2.get_color())

        plt.legend()
        plt.title(f'Angle and Speed {self.title}')
        plt.savefig('speedplot.png')

    def plot_angles(self, accel_angles, gyro_angles):
        fig, left_ax = plt.subplots(figsize = (10, 10))

        time_values = np.linspace(0, self.timer, len(accel_angles))
        p1, = left_ax.plot(time_values, accel_angles, "b-", label="accel")
        
        time_values = np.linspace(0, self.timer, len(gyro_angles))
        p2, = left_ax.plot(time_values, gyro_angles, "r-", label="gyro")

        avg_angle = np.mean(self.collected_data['angles'], axis=0)
        left_ax.axhline(y=avg_angle, color='k', linestyle='--', label=f"avg_angle at {avg_angle}°")

        left_ax.set_xlim(0, self.timer)
        left_ax.set_ylim(-11, 11)

        left_ax.set_xlabel('Time (s)')
        left_ax.set_ylabel('Angle (°)')

        plt.legend()
        plt.title(f'Accel- and Gyroangles {self.title}')
        plt.savefig('angles.png')