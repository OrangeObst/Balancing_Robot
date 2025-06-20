import multiprocessing
import time
from codetiming import Timer

class MultiprocessingStepper():
    def __init__(self, left_motor_motor, right_motor_motor):
        self.left_motor = left_motor_motor
        self.right_motor = right_motor_motor

        self.run_process = multiprocessing.Value('b', True)
        self.left_velocity = multiprocessing.Value('f', 0.0)
        self.right_velocity = multiprocessing.Value('f', 0.0)
        self.velocity_update_event = multiprocessing.Event()

        self.motor_loop_process = multiprocessing.Process(target=self._run)

    # @Timer(name="run", text="Run: {milliseconds:6.4f} ms")
    def _run(self):
        while self.run_process.value:
            if self.velocity_update_event.is_set():
                self.velocity_update_event.clear()
                left_velocity = self.left_velocity.value
                right_velocity = self.right_velocity.value
                self.left_motor.set_velocity(left_velocity)
                self.right_motor.set_velocity(right_velocity)
            self.left_motor.loop()
            self.right_motor.loop()

    @Timer(name="set_velocity", text="Set_velocity: {milliseconds:6.4f} ms")
    def set_velocity(self, left_velocity=None, right_velocity=None):
        if left_velocity is not None:
            self.left_velocity.value = left_velocity
            self.velocity_update_event.set()
        if right_velocity is not None:
            self.right_velocity.value = right_velocity
            self.velocity_update_event.set()

    def get_steps(self):
        left_motor_steps = self.left_motor.get_position()
        right_motor_steps = self.right_motor.get_position()
        return left_motor_steps, right_motor_steps
    
    def start(self):
        self.left_motor.start()
        self.right_motor.start()
        self.motor_loop_process.start()

    def shutdown(self):
        self.run_process.value = False
        time.sleep(0.1)
        self.motor_loop_process.join()
        self.left_motor.shutdown()
        self.right_motor.shutdown()

if __name__ == "__main__":
    import sys
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from src.robot.stepper_motor import Stepper

    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), microsteps=8)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), microsteps=8, invert_direction=True)

    multiprocessing_motors = MultiprocessingStepper(left_motor, right_motor)
    multiprocessing_motors.start()


    end_time = time.time() + 15
    last_time = time.time()
    velocity = 0
    counter = 0
    try:
        while time.time() < end_time:
            start_time = time.time()
            if (start_time - last_time) > 0.5:
                velocity += 10 if counter<10 else -10
                multiprocessing_motors.set_velocity(velocity, velocity)
                counter +=1
                last_time = start_time
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        multiprocessing_motors.shutdown()
