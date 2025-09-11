class MotorController:
    def __init__(self, left_motor, right_motor, processed=False):
        if processed:
            from processed_motors import MultiprocessingStepper
            self.controller = MultiprocessingStepper(left_motor, right_motor)
            self.processed = True
        else:
            self.left_motor = left_motor
            self.right_motor = right_motor
            self.processed = False

    def start(self):
        if self.processed:
            self.controller.start()
        else:
            self.left_motor.start()
            self.right_motor.start()

    def stop(self):
        if self.processed:
            self.controller.reset_motors()
        else:
            self.left_motor.reset_motor()
            self.right_motor.reset_motor()

    def set_velocity(self, left, right):
        if self.processed:
            self.controller.set_velocity(left, right)
        else:
            self.left_motor.set_velocity(left)
            self.right_motor.set_velocity(right)

    def get_steps(self):
        if self.processed:
            return self.controller.get_steps()
        else:
            return self.left_motor.get_position(), self.right_motor.get_position()

    def shutdown(self):
        if self.processed:
            self.controller.shutdown()
        else:
            self.left_motor.shutdown()
            self.right_motor.shutdown()

    def loop(self):
        if not self.processed:
            self.left_motor.loop()
            self.right_motor.loop()