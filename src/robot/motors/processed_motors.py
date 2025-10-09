import multiprocessing
import time

class MultiprocessingStepper():
    def __init__(self, left_motor_motor, right_motor_motor):
        self.left_motor = left_motor_motor
        self.right_motor = right_motor_motor

        self.run_process = multiprocessing.Value('b', False)
        self.left_motor_velocity = multiprocessing.Value('f', 0.0)
        self.right_motor_velocity = multiprocessing.Value('f', 0.0)
        self.velocity_update_event = multiprocessing.Event()
        self.left_motor_steps = multiprocessing.Value('i', 0)
        self.right_motor_steps = multiprocessing.Value('i', 0)
        self.want_step_update_event = multiprocessing.Event()
        self.done_step_update_event = multiprocessing.Event()
        self.motor_loop_process = multiprocessing.Process(target=self._run)

    def _run(self):
        while self.run_process.value:
            if self.velocity_update_event.is_set():
                self.velocity_update_event.clear()
                left_motor_velocity = self.left_motor_velocity.value
                right_motor_velocity = self.right_motor_velocity.value
                self.left_motor.set_velocity(left_motor_velocity)
                self.right_motor.set_velocity(right_motor_velocity)
            if self.want_step_update_event.is_set():
                self.want_step_update_event.clear()
                self.done_step_update_event.set()
                self.left_motor_steps.value = self.left_motor.get_position()
                self.right_motor_steps.value = self.right_motor.get_position()
            self.left_motor.loop()
            self.right_motor.loop()

    def set_velocity(self, left_motor_velocity=None, right_motor_velocity=None):
        if left_motor_velocity is not None:
            self.left_motor_velocity.value = left_motor_velocity
            self.velocity_update_event.set()
        if right_motor_velocity is not None:
            self.right_motor_velocity.value = right_motor_velocity
            self.velocity_update_event.set()

    def get_steps(self):
        self.want_step_update_event.set()
        self.done_step_update_event.wait(timeout=0.02)
        self.done_step_update_event.clear()
        return self.left_motor_steps.value, self.right_motor_steps.value
    
    def start(self):
        self.run_process.value = True
        self.left_motor.start()
        self.right_motor.start()
        if not self.motor_loop_process.is_alive():
            self.motor_loop_process = multiprocessing.Process(target=self._run)
            self.motor_loop_process.start()

    def stop(self):
        self.run_process.value = False
        self.left_motor.reset_motor()
        self.right_motor.reset_motor()

    def shutdown(self):
        self.run_process.value = False
        time.sleep(0.05)
        self.motor_loop_process.join()
        self.left_motor.shutdown()
        self.right_motor.shutdown()

    def reset_motors(self):
        self.left_motor.reset_motor()
        self.right_motor.reset_motor()

if __name__ == "__main__":
    from robot.motors.stepper_motor import Stepper

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
