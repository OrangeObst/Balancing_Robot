from .processed_motors import MultiprocessingStepper

class MotorController:
    def __init__(self, left_motor, right_motor, use_motors=False, processed=False):
        self.use_motors = use_motors
        self.processed = processed
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.processed_motors = MultiprocessingStepper(left_motor, right_motor)

    def start(self):
        if self.processed:
            self.processed_motors.start()
        elif self.use_motors:
            self.left_motor.start()
            self.right_motor.start()

    def stop(self):
        if self.processed:
            self.processed_motors.reset_motors()
        elif self.use_motors:
            self.left_motor.reset_motor()
            self.right_motor.reset_motor()

    def activate_processed_motors(self):
        if not self.processed:
            self.processed = True
            self.processed_motors.start()

    def deactivate_processed_motors(self):
        self.processed = False
        self.processed_motors.stop()

    def activate_motors(self):
        self.use_motors = True
        if not self.processed:
            self.left_motor.start()
            self.right_motor.start()
    
    def deactivate_motors(self):
        self.use_motors = False
        if not self.processed:
            self.left_motor.reset_motor()
            self.right_motor.reset_motor()

    def set_velocity(self, left, right):
        if self.processed:
            self.processed_motors.set_velocity(left, right)
        elif self.use_motors:
            self.left_motor.set_velocity(left)
            self.right_motor.set_velocity(right)

    def get_steps(self):
        if self.processed:
            return self.processed_motors.get_steps()
        elif self.use_motors:
            return self.left_motor.get_position(), self.right_motor.get_position()
        else:
            return 0,0

    def shutdown(self):
        if self.processed:
            self.processed_motors.shutdown()
        elif self.use_motors:
            self.left_motor.shutdown()
            self.right_motor.shutdown()

    def loop(self):
        if self.use_motors and not self.processed:
            self.left_motor.loop()
            self.right_motor.loop()