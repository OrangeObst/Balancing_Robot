import threading

class ThreadedStepper(threading.Thread):
    def __init__(self, stepper_motor):
        super().__init__()
        self.stepper = stepper_motor
        self._stop_event = threading.Event()
        self._lock = threading.RLock()

    def run(self):
        while not self._stop_event.is_set():
            self.stepper.loop()

    # @Timer(name="Set velocity", text="Set velocity: {milliseconds:.6f} ms")
    def set_velocity(self, velocity):
        with self._lock:
            self.stepper.set_velocity(velocity)

    def get_position(self):
        return self.stepper.get_position()

    def shutdown(self):
        self._stop_event.set()
        self.join()
        self.stepper.shutdown()


if __name__ == "__main__":
    from stepper_motor import Stepper
    import time

    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), microsteps=8)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), microsteps=8, invert_direction=True)
    left_motor.start()
    right_motor.start()

    threaded_left_motor = ThreadedStepper(left_motor)
    threaded_right_motor = ThreadedStepper(right_motor)
    threaded_left_motor.start()
    threaded_right_motor.start()

    try:
        threaded_left_motor.set_velocity(40)
        threaded_right_motor.set_velocity(40)
        time.sleep(3)
        threaded_left_motor.set_velocity(-40)
        threaded_right_motor.set_velocity(-40)
        time.sleep(3)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        threaded_left_motor.shutdown()
        threaded_right_motor.shutdown()