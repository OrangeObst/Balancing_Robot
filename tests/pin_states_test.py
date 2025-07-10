import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

pins = {
    4: GPIO.OUT,
    12: GPIO.OUT,
    13: GPIO.OUT,
    18: GPIO.OUT,
    19: GPIO.OUT,
    24: GPIO.OUT,
    16: GPIO.OUT,
    17: GPIO.OUT,
    20: GPIO.OUT,
    21: GPIO.OUT,
    22: GPIO.OUT,
    27: GPIO.OUT
}

def set_pin_direction(pin_map):
    for pin, state in pin_map.items():
        GPIO.setup(pin, GPIO.IN)

def print_pin_state(pin_map):
    for pin, state in pin_map.items():
        pin_state = GPIO.input(pin)
        if pin_state == GPIO.HIGH:
            print(f"Pin {pin}, Dir {state}: HIGH")
        else:
            print(f"Pin {pin}, Dir {state}: LOW")


if __name__ == "__main__":
    set_pin_direction(pins)
    print_pin_state(pins)
    GPIO.cleanup()