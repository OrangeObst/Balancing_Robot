class PID_Filtered(object):
    def __init__(self, 
                 kp: float, 
                 ki: float, 
                 kd: float, 
                 n: int, 
                 dt: float, 
                 min_output: float, 
                 max_output: float, 
                 setpoint=0.0):
        
        self.kp = kp    # P
        self.ki = ki    # I
        self.kd = kd    # D
        self.n = n      # Derivative filtering constant

        self.dt = dt
        self.intf = 0.0
        self.df1 = 0.0
        self.df2 = 0.0

        self.min_output = min_output
        self.max_output = max_output
        self.setpoint = setpoint

        # self.previous_error = 0
        self.previous_input = 0
        self.previous_dterm = 0
        self.iterm = 0

        self.update_parameters()


    def set_parameters(self, p, i, d, n):
        self.kp = p
        self.ki = i
        self.kd = d
        self.n = n
        self.update_parameters()

    def set_dt(self, dt):
        self.dt = dt
        self.update_parameters()

    def update_parameters(self):
        self.intf = self.dt * self.ki
        self.df1 = self.kd / (self.kd + self.n * self.dt)
        self.df2 = (self.kd * self.n) / (self.kd + self.n * self.dt)

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def update(self, measured_value):
        error = self.setpoint - measured_value

        pterm = self.kp * error
        self.iterm += self.intf * error
        
        # dterm = self.kd * (error - self.previous_error) / dt
        # http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/
        # Derivative on measurement, in case I want to change the setpoint in the future
        dterm = self.previous_dterm * self.df1 - (measured_value - self.previous_input) * self.df2

        output = pterm + self.iterm + dterm
        output = max(self.min_output, min(self.max_output, output))

        # self.previous_error = error
        self.previous_input = measured_value

        # pterm, iterm und dterm sind nur zum plotten
        return output, pterm, self.iterm, dterm
