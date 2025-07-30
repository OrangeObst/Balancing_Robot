class PID_Controller(object):
    """
        A PID (Proportional-Integral-Derivative) controller class used to compute 
        and adjust control outputs based on feedback from a measured process.

        The PID controller calculates an error value as the difference between a 
        desired setpoint and a measured process value. It then applies 
        proportional, integral, and derivative terms to this error to compute the 
        output.

        Attributes:
            kp (float): The proportional gain.
            ki (float): The integral gain.
            kd (float): The derivative gain.
            min_output (float): The minimum value of the controller output.
            max_output (float): The maximum value of the controller output.
            setpoint (float): The target value that the controller aims to achieve.
            alpha (float): The smoothing factor for the derivative term.
            deadband (float): The range within which the process is allowed to vary
                without accumulating the error state for the integral term.

        Parameters:
            kp (float): The proportional gain parameter.
            ki (float): The integral gain parameter.
            kd (float): The derivative gain parameter.
            min_output (float): The minimum allowable output from the controller.
            max_output (float): The maximum allowable output from the controller.
            setpoint (float, optional): The desired setpoint. Defaults to 0.0.
            alpha (float, optional): The smoothing factor for the derivative. Defaults to 0.0.
            deadband (float, optional): The deadband range. Defaults to 0.0.

        Instance Variables:
            previous_input (float): Stores the previous input value used in the 
                derivative term calculation.
            previous_dterm (float): Stores the previous derivative term value.
            sum_error (float): Stores the cumulative error for the integral term.
    """
    def __init__(self, 
                 kp: float, 
                 ki: float, 
                 kd: float,  
                 min_output: float, 
                 max_output: float, 
                 setpoint=0.0,
                 alpha=0.0,
                 deadband=0.0):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.min_output = min_output
        self.max_output = max_output
        self.setpoint = setpoint
        self.alpha = alpha
        self.deadband = deadband

        self.previous_input = 0.0
        self.previous_dterm = 0.0
        self.sum_error = 0.0


    def set_parameters(self, p = None, i = None, d = None):
        """
            Set or update the PID controller parameters.

            Args:
                p (float, optional): Proportional gain. Defaults to None.
                i (float, optional): Integral gain. Defaults to None.
                d (float, optional): Derivative gain. Defaults to None.

            Notes:
                Only the parameters provided (not None) will update the controller.
                Updates the corresponding attributes (kp, ki, kd) if the input is not None.
        """
        if p is not None:
            self.kp = p
        if i is not None:
            self.ki = i
        if d is not None:
            self.kd = d


    def update(self, input, dt):
        """
            Updates the PID controller with new input and time step.

            Args:
                input: The current value of the process being controlled.
                dt: The time step since the last update.

            Returns:
                A tuple containing:
                - The clamped output of the PID controller.
                - The clamped proportional term (pterm).
                - The clamped integral term (iterm).
                - The clamped derivative term (dterm).

            The output is computed using the PID algorithm:
                - The proportional term (pterm) is based on the current error.
                - The integral term (iterm) incorporates the accumulation of past errors, adjusted for deadband.
                - The derivative term (dterm) is based on the rate of change of the input, optionally filtered.

            The final output and each term are clamped within [min_output, max_output] to ensure proper return values.
        """
        error = self.setpoint - input
        pterm = self.kp * error
        iterm = 0.0
        dterm = 0.0
        # https://gitlab.com/kloppertje/balancingrobot/-/blob/ps3control/Software/lib/PID/PID.cpp?ref_type=heads
        if self.ki > 0.0:
            out_of_deadzone = 0.0
            if self.deadband > 0.0:
                if abs(error) < self.deadband:
                    out_of_deadzone = 0.0
                elif error < -self.deadband:
                    out_of_deadzone = error + self.deadband
                else:
                    out_of_deadzone = error - self.deadband
            else:
                out_of_deadzone = error

            self.sum_error += out_of_deadzone * dt
            iterm = self.ki * self.sum_error
        
        # dterm = self.kd * (self.alpha * self.previous_dterm + (1 - self.alpha) * ((error - self.previous_error) / dt))
        # http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/
        # Derivative on measurement, in case I want to change the setpoint in the future
        dterm = self.kd * ((input - self.previous_input) / dt)
        if self.alpha > 0.0:
            dterm = self.alpha * self.previous_dterm + (1 - self.alpha) * dterm

        output = pterm + iterm - dterm

        # self.previous_error = error
        self.previous_input = input
        self.previous_dterm = dterm

        # pterm, iterm and dterm are only returned for debugging purposes
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
