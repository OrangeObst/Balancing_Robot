class PID_Controller(object):
    def __init__(self, kp, ki, kd, min_output, max_output, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_output = min_output
        self.max_output = max_output
        self.setpoint = setpoint

        # self.previous_error = 0
        self.previous_input = 0
        self.sum_error = 0

    def update(self, measured_value, dt):
        # Get error state
        error = self.setpoint - measured_value
        self.sum_error += error * dt

        # Calculate PID terms
        pterm = self.kp * error
        iterm = self.ki * self.sum_error
        # dterm = self.kd * (error - self.previous_error) / dt
        # http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/
        # Derivative on measurement, in case I want to change the setpoint in the future
        dterm = self.kd * -(measured_value - self.previous_input) / dt

        output = pterm + iterm + dterm
        output = max(self.min_output, min(self.max_output, output))

        # self.previous_error = error
        self.previous_input = measured_value

        # pterm, iterm und dterm sind nur zum plotten
        return output, pterm, iterm, dterm


    def set_setpoint(self, setpoint):
        self.set_setpoint = setpoint