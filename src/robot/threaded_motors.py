import threading
import time
import RPi.GPIO as GPIO
from math import pi
from codetiming import Timer
from multiprocessing import Lock


class Stepper:
    MIN_VELOCITY_THRESHOLD = 0.5
    MIN_STEP_DELAY = 0.00033        # Max 3000 Steps pro Sekunde
    MAX_STEP_DELAY = 1e6
    STEP_PULSE_WIDTH = 1e-6

    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins, invert_direction=False, steps=200):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        self.invert_direction = invert_direction
        self.steps = steps
        self.lock = Lock()
        self._stop_request = threading.Event()
        self._thread = threading.Thread(target=self._run)
        self._thread.daemon = True

        self.position = 0
        self.dx = 1
        self.step_delay = 1
        self.last_step_ts = 0.0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)


    def set_direction(self, direction):
        if self.invert_direction:
            direction = not direction
        GPIO.output(self.dir_pin, direction)
        if direction == Stepper.CW:
            self.dx = -1 if self.invert_direction else 1
        else:
            self.dx = 1 if self.invert_direction else -1


    # @Timer(name="Motor step", text="Motor step: {milliseconds:.6f}ms")
    def step(self):
        with self.lock:
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(Stepper.STEP_PULSE_WIDTH)
            GPIO.output(self.step_pin, GPIO.LOW)
            self.position += self.dx


    def start(self):
        GPIO.output(self.enable_pin, GPIO.HIGH)
        self._thread.start()


    def stop(self):
        GPIO.output(self.enable_pin, GPIO.LOW)
        self._stop_request.set()


    def get_position(self):
        return self.position


    def set_velocity(self, velocity):
        step_rate = (velocity / 100) * 3000
        if abs(velocity) < Stepper.MIN_VELOCITY_THRESHOLD:
            self.step_delay = Stepper.MAX_STEP_DELAY
        else:
            self.step_delay = 1 / abs(step_rate) - Stepper.STEP_PULSE_WIDTH
            if self.step_delay < self.MIN_STEP_DELAY:
                self.step_delay = self.MIN_STEP_DELAY

            self.set_direction(Stepper.CW if velocity > 0 else Stepper.CCW)


    # @Timer(name="Motor loop", text="Motor loop: {milliseconds:.6f}ms")
    def loop(self):
        now = time.time()
        if now - self.last_step_ts >= self.step_delay:
            self.step()
            self.last_step_ts = now

    def _run(self):
        while not self._stop_request.is_set():
            self.loop()


if __name__ == "__main__":
    # motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=800)
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=800)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), invert_direction=True, steps=800)

    left_motor.start()
    right_motor.start()

    end_time = time.time() + 15
    last_time = time.time()
    velocity = 0
    counter = 0
    try:
        while time.time() < end_time:
            start_time = time.time()
            if (start_time - last_time) > 1:
                velocity += 10 if counter<5 else -10
                left_motor.set_velocity(velocity)
                right_motor.set_velocity(velocity)

                counter +=1
                last_time = start_time
            left_motor.loop()
            right_motor.loop()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        pass
    finally:
        left_motor.stop()
        right_motor.stop()
        GPIO.cleanup()
