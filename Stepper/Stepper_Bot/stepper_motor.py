import time
import RPi.GPIO as GPIO
from math import pi


class Stepper:
    MIN_VELOCITY_THRESHOLD = 1e-6
    MIN_STEP_DELAY = 0.00033        # Max 3000 Steps pro Sekunde
    MAX_STEP_DELAY = 1e6
    STEP_PULSE_WIDTH = 1e-6

    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins, steps=200):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        self.steps = steps

        self.position = 0
        self.dx = 1
        self.step_delay = 0
        self.last_step_ts = 0.0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)

    def set_direction(self, direction):
        GPIO.output(self.dir_pin, direction)
        if direction == Stepper.CW:
            self.dx = 1
        else:
            self.dx = -1

    def step(self):
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(Stepper.STEP_PULSE_WIDTH)
        GPIO.output(self.step_pin, GPIO.LOW)
        self.position += self.dx
        # print(self.position)

    def start(self):
        GPIO.output(self.enable_pin, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.enable_pin, GPIO.LOW)

    def set_velocity(self, velocity):
        if abs(velocity) < Stepper.MIN_VELOCITY_THRESHOLD:
            self.step_delay = Stepper.MAX_STEP_DELAY
        else:
            self.step_delay = 1/(abs(30*velocity)) - Stepper.STEP_PULSE_WIDTH
            if self.step_delay < self.MIN_STEP_DELAY:
                self.step_delay = self.MIN_STEP_DELAY
                
            self.set_direction(Stepper.CW if velocity > 0 else Stepper.CCW)

    def return_to_origin(self):
        steps = self.position % self.steps
        

    def loop(self):
        now = time.time()
        if now - self.last_step_ts >= self.step_delay:
            self.step()
            self.last_step_ts = now

if __name__ == "__main__":
    motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=400)

    motor.set_direction(Stepper.CW)
    motor.set_velocity(-6.0 * pi)
    motor.start()
    
    try:
        while abs(motor.position) < motor.steps:
        # while True:
            motor.loop()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        pass

    motor.stop()
    GPIO.cleanup()