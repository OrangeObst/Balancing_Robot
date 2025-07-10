import time
import RPi.GPIO as GPIO


class Stepper:
    MIN_VELOCITY_THRESHOLD = 0.5    # Could maybe be increased
    MIN_STEP_DELAY = 0.00033        # Max 3000 Steps pro Sekunde
    MAX_STEP_DELAY = 1e6
    MAX_STEPS_PER_SECOND = 3000
    STEP_PULSE_WIDTH = 1e-6

    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins, microsteps, invert_direction=False):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        self.invert_direction = invert_direction
        self.microsteps = microsteps

        self.steps = 0
        self.dx = 1
        self.step_delay = 1
        self.last_step_ts = 0.0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.mode_pins, GPIO.OUT)

    def _set_direction(self, direction):
        self.dx = 1 if direction == Stepper.CW else -1
        direction = not direction if self.invert_direction else direction
        GPIO.output(self.dir_pin, direction)

    def step(self):
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(Stepper.STEP_PULSE_WIDTH)
        GPIO.output(self.step_pin, GPIO.LOW)
        self.steps += self.dx

    # Note: System is calibrated for 1/8 - microstepping,
    # Increase 8 to the microstep setting the system was calibrated for and recalibrate system
    def set_velocity(self, velocity):
        scaled_velocity = velocity * (self.microsteps / 8)
        step_rate = (scaled_velocity / 100) * Stepper.MAX_STEPS_PER_SECOND
        if abs(scaled_velocity) < Stepper.MIN_VELOCITY_THRESHOLD:
            self.step_delay = Stepper.MAX_STEP_DELAY
        else:
            self.step_delay = 1 / abs(step_rate) - Stepper.STEP_PULSE_WIDTH
            if self.step_delay < Stepper.MIN_STEP_DELAY:
                self.step_delay = Stepper.MIN_STEP_DELAY

            self._set_direction(Stepper.CW if scaled_velocity > 0 else Stepper.CCW)

    def start(self):
        GPIO.output(self.enable_pin, GPIO.HIGH)

    def stop(self):
        GPIO.output(self.enable_pin, GPIO.LOW)

    def shutdown(self):
        print("Cleaning up GPIO ...")
        GPIO.cleanup()

    def get_position(self):
        return self.steps
    
    def get_microsteps(self):
        return self.microsteps

    def loop(self):
        now = time.time()
        if now - self.last_step_ts >= self.step_delay:
            self.step()
            self.last_step_ts = now


if __name__ == "__main__":
    microstepping = 8           # Microstepping setting = 1/8
    spr = 200 * microstepping   # steps per resolution

    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), microsteps=microstepping)
    right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), microsteps=microstepping, invert_direction=True)

    left_motor.start()
    right_motor.start()

    end_time = time.time() + 20
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

        # left_motor.set_velocity(20)
        # right_motor.set_velocity(20)
        # while left_motor.get_steps() < spr:
        #     left_motor.loop()
        #     right_motor.loop()

    except KeyboardInterrupt:
        print("Keyboard interrupt")
        pass
    finally:
        left_motor.stop()
        right_motor.stop()
        GPIO.cleanup()
