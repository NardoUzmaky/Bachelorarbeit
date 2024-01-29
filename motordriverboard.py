from gpiozero import OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory as pin_factory
import time

factory = pin_factory()

class MotorControl:
    def __init__(self, axis):
        if axis == 1: # Initialize Motor for axis 1
            self.pin1 = OutputDevice(20, pin_factory=factory) #if pin1 = 1 and pin2 = 0: motor turns forward
            self.pin2 = OutputDevice(21, pin_factory=factory)
            self.pwm = PWMOutputDevice(26, frequency=5000, pin_factory=factory)
        elif axis == 2:  # Initialize Motor for axis 2
            self.pin1 = OutputDevice(6, pin_factory=factory)
            self.pin2 = OutputDevice(13, pin_factory=factory)
            self.pwm = PWMOutputDevice(12, frequency=5000, pin_factory=factory)
        else:
            raise ValueError(f"Invalid axis: {axis}")
        self.direction = None
        self.speed = 0

    def update(self, control_input, angle):
        new_speed = abs(control_input) / 100  # Assuming control_input is a percentage

        if control_input < 0:
            new_direction = "forward"
        else:
            new_direction = "reverse"

        if (new_direction == "forward" and angle < -12) or (new_direction == "reverse" and angle > 12):
            print("stopper activated")
            self.pwm.value = 0
            return

        self.update_motor(new_direction, new_speed)

    def update_motor(self, direction, speed):
        if direction == "forward":
            self.pin1.on()
            self.pin2.off()
        elif direction == "reverse":
            self.pin1.off()
            self.pin2.on()
        else:
            raise ValueError("Invalid Direction")
            
        self.pwm.value = speed
        self.speed = speed
        self.direction = direction

    def clean_up(self):
        self.pwm.close()
        self.pin1.close()
        self.pin2.close()

# Main Execution Block
if __name__ == "__main__":
    y_motor = None
    try:
        y_motor = MotorControl(2)
        y_motor.update_motor("forward", 0.2)  # Using 0.5 for 50% speed
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    except Exception as e:
        print(e)
    finally:
        print("Clean up")
        y_motor.clean_up()
