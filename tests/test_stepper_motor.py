import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot.motors.stepper_motor import Stepper
import RPi.GPIO as GPIO
import unittest

class TestStepperClass(unittest.TestCase):
    CW = GPIO.HIGH
    CCW = GPIO.LOW

    def setUp(self):
        if not hasattr(self, 'gpio_initialized'):
            self.left_stepper = Stepper(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20), microsteps=8)
            self.right_stepper = Stepper(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27), microsteps=8, invert_direction=True)
            self.left_stepper.start()
            self.right_stepper.start()
            self.gpio_initialized = True

    def tearDown(self):
        self.left_stepper.stop()
        self.right_stepper.stop()
        GPIO.cleanup()

    def testSetDirection(self):
        self.left_stepper._set_direction(self.CW)
        self.assertEqual(self.left_stepper.dx, 1)
        self.right_stepper._set_direction(self.CW)
        self.assertEqual(self.right_stepper.dx, 1)
        self.left_stepper._set_direction(self.CCW)
        self.assertEqual(self.left_stepper.dx, -1)
        self.right_stepper._set_direction(self.CCW)
        self.assertEqual(self.right_stepper.dx, -1)

    def testSetVelocity(self):
        self.left_stepper.set_velocity(10)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.00333, places=5)
        self.right_stepper.set_velocity(10)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.00333, places=5)
        self.left_stepper.set_velocity(-10)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.00333, places=5)
        self.right_stepper.set_velocity(-10)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.00333, places=5)
        self.left_stepper.set_velocity(50)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000666, places=5)
        self.right_stepper.set_velocity(50)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000666, places=5)
        self.left_stepper.set_velocity(-50)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000666, places=5)
        self.right_stepper.set_velocity(-50)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000666, places=5)
        self.left_stepper.set_velocity(100)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000333, places=5)
        self.right_stepper.set_velocity(100)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000333, places=5)
        self.left_stepper.set_velocity(-100)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000333, places=5)
        self.right_stepper.set_velocity(-100)
        self.assertAlmostEqual(self.left_stepper.step_delay, 0.000333, places=5)

    def test360Turn(self):
        self.left_stepper.set_velocity(20)
        target_position = 200 * self.left_stepper.get_microsteps()
        while self.left_stepper.get_position() != target_position:
            self.left_stepper.set_velocity(20)
            self.left_stepper.loop()
        self.assertEqual(self.left_stepper.get_position(), target_position)

        self.left_stepper.set_velocity(-20)
        target_position = 0
        while self.left_stepper.get_position() != target_position:
            self.left_stepper.set_velocity(-20)
            self.left_stepper.loop()
        self.assertEqual(self.left_stepper.get_position(), target_position)

        self.right_stepper.set_velocity(20)
        target_position = 200 * self.right_stepper.get_microsteps()
        while self.right_stepper.get_position() != target_position:
            self.right_stepper.set_velocity(20)
            self.right_stepper.loop()
        self.assertEqual(self.right_stepper.get_position(), target_position)

        self.right_stepper.set_velocity(-20)
        target_position = 0
        while self.right_stepper.get_position() != target_position:
            self.right_stepper.set_velocity(-20)
            self.right_stepper.loop()
        self.assertEqual(self.right_stepper.get_position(), target_position)


if __name__ == '__main__':
    unittest.main()