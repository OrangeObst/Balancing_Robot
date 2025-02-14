class PID_Controller(object):
    def __init__(self, 
                 kp: float, 
                 ki: float, 
                 kd: float,  
                 min_output: float, 
                 max_output: float, 
                 setpoint=0.0,
                 alpha=0.98):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.min_output = min_output
        self.max_output = max_output
        self.setpoint = setpoint
        self.alpha = alpha

        self.previous_input = 0
        self.sum_error = 0

    def set_parameters(self, p = None, i = None, d = None):
        if p is not None:
            self.kp = p
        if i is not None:
            self.ki = i
        if i is not None:
            self.kd = d

    def update(self, input, dt):
        # Get error state
        error = self.setpoint - input
        self.sum_error += error * dt

        # Calculate PID terms
        pterm = self.kp * error
        iterm = self.ki * self.sum_error
        
        # dterm = self.kd * (self.alpha * self.previous_dterm + (error - self.previous_error) / dt)
        # http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/
        # Derivative on measurement, in case I want to change the setpoint in the future
        dterm = self.kd * ((input - self.previous_input) / dt)

        output = pterm + iterm - dterm

        # self.previous_error = error
        self.previous_input = input

        # pterm, iterm und dterm sind nur zum plotten
        return (
            max(self.min_output, min(self.max_output, output)),
            max(self.min_output, min(self.max_output, pterm)),
            max(self.min_output, min(self.max_output, iterm)),
            max(self.min_output, min(self.max_output, dterm))
        )


    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

if __name__ == "__main__":
    pid = PID_Controller(10, 0.4, 0.2, -100, 100, 0.0, 0.5)
    dt = 0.01
    print(pid.update(1, dt))
    print(pid.update(2, dt))
    print(pid.update(3, dt))
    print(pid.update(4, dt))
    print(pid.update(5, dt))
    print(pid.update(5, dt))
    print(pid.update(5, dt))
    print(pid.update(4, dt))
    print(pid.update(3, dt))
    print(pid.update(2, dt))
    print(pid.update(1, dt))