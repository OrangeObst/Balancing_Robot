import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.robot.stepper_motor import Stepper
import RPi.GPIO as GPIO
import unittest

class TestStepperClass(unittest.TestCase):
    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def setUp(self):
        if not hasattr(self, 'gpio_initialized'):
            self.left_stepper = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), steps=800)
            self.right_stepper = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), invert_direction=True, steps=800)
            self.gpio_initialized = True

    def tearDown(self):
        GPIO.cleanup()

    def test_init(self):
        self.assertEqual(self.left_stepper.dir_pin, 13)
        self.assertEqual(self.left_stepper.steps, 800)
        self.assertEqual(self.right_stepper.dir_pin, 24)
        self.assertEqual(self.right_stepper.steps, 800)

    def test_set_direction(self):
        self.left_stepper.set_direction(self.CW)
        self.assertEqual(self.left_stepper.dx, 1)
        self.right_stepper.set_direction(self.CW)
        self.assertEqual(self.right_stepper.dx, 1)
        self.left_stepper.set_direction(self.CCW)
        self.assertEqual(self.left_stepper.dx, -1)
        self.right_stepper.set_direction(self.CCW)
        self.assertEqual(self.right_stepper.dx, -1)


if __name__ == '__main__':
    unittest.main()