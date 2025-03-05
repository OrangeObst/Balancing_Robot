import threading
import RPi.GPIO as GPIO
from codetiming import Timer
from time import time, sleep


class Stepper:
    MIN_VELOCITY_THRESHOLD = 0.5    # Could maybe be increased
    MIN_STEP_DELAY = 0.00033        # Max 3000 Steps pro Sekunde
    MAX_STEP_DELAY = 1e6
    STEP_PULSE_WIDTH = 1e-6
    MAX_STEPS_PER_SECOND = 3000

    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins, microsteps, invert_direction=False):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        self.invert_direction = invert_direction
        self.microsteps = microsteps
        self.microstep_scaling = microsteps / 8

        self.position = 0
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


    # @Timer(name="Motor step", text="Motor step: {milliseconds:.6f}ms")
    def _step(self):
        # with self.lock:
        GPIO.output(self.step_pin, GPIO.HIGH)
        sleep(Stepper.STEP_PULSE_WIDTH)
        GPIO.output(self.step_pin, GPIO.LOW)
        self.position += self.dx


    def start(self):
        GPIO.output(self.enable_pin, GPIO.HIGH)
        # self._thread.start()


    def stop(self):
        GPIO.output(self.enable_pin, GPIO.LOW)
        # self._stop_request.set()


    def get_position(self):
        return self.position

    # Note: System is calibrated for 1/8 - microstepping,
    # Increase 8 to the microstep setting the system was calibrated for and recalibrate system
    def set_velocity(self, velocity):
        """
        Sets the velocity of the stepper motor.

        :param velocity: Desired velocity
        """
        velocity_to_step_scaling = 1 / 100 * Stepper.MAX_STEPS_PER_SECOND

        scaled_velocity = velocity * self.microstep_scaling

        # Apply minimum velocity threshold
        if abs(scaled_velocity) < Stepper.MIN_VELOCITY_THRESHOLD:
            # with self.lock:
            self.step_delay = Stepper.MAX_STEP_DELAY
            
            return  # Early exit for simplicity

        step_rate = scaled_velocity * velocity_to_step_scaling
        delay = max(Stepper.MIN_STEP_DELAY, (1 / abs(step_rate)) - Stepper.STEP_PULSE_WIDTH)
        # print(f'Delay: {delay:6.5f} | Step rate: {step_rate} | Scaled velo: {scaled_velocity}')
        # Update step delay and direction under lock
        # with self.lock:
        self.step_delay = delay
        self._set_direction(Stepper.CW if scaled_velocity > 0 else Stepper.CCW)



    # @Timer(name="Motor loop", text="Motor loop: {milliseconds:.6f}ms")
    def loop(self):
        now = time()
        if now - self.last_step_ts >= self.step_delay:
            self._step()
            self.last_step_ts = now

    # def _run(self):
    #     while not self._stop_request.is_set():
    #         self.loop()


if __name__ == "__main__":
    microstepping = 8           # Microstepping setting = 1/8
    spr = 200 * microstepping   # steps per resolution
    
    left_motor = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), microsteps=8)
    # right_motor = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), microsteps=8, invert_direction=True)

    left_motor.start()
    # right_motor.start()

    lock = threading.Lock()
    _stop_request = threading.Event()
    def _run():
        while not _stop_request.is_set():
            left_motor.loop()
    _thread = threading.Thread(target=_run)
    _thread.daemon = True
    _thread.start()

    timer = 6
    end_time = time() + timer
    last_time = time()
    velocity = 0
    counter = 0
    try:
        while time() < end_time:
            start_time = time()
            if (start_time - last_time) > 1:
                velocity += 10 # if counter < 3 else -10
                left_motor.set_velocity(velocity)
                # right_motor.set_velocity(velocity)

                counter +=1
                last_time = start_time
            # else:
            #     left_motor._set_direction(left_motor.CW)
                # right_motor._set_direction(right_motor.CW)

    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        left_motor.stop()
        # right_motor.stop()
        GPIO.cleanup()
