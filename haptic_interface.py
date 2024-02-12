import pygame
#from potentiometer import read_potentiometer, initialize_potentiometer
import time
from gpiozero import OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory as pin_factory

factory = pin_factory()

class MotorControl:
    def __init__(self, axis):
        if axis == 1: # Initialize Motor for axis 1
            self.pin1 = OutputDevice(20, pin_factory=factory) #if pin1 = 1 and pin2 = 0: motor turns forward
            self.pin2 = OutputDevice(21, pin_factory=factory)
            self.pwm = PWMOutputDevice(26, frequency=500, pin_factory=factory)
        elif axis == 2:  # Initialize Motor for axis 2
            self.pin1 = OutputDevice(6, pin_factory=factory)
            self.pin2 = OutputDevice(13, pin_factory=factory)
            self.pwm = PWMOutputDevice(12, frequency=500, pin_factory=factory)
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
try:
	interval = 0.01
	x_motor = MotorControl(1)
	y_motor = MotorControl(2)
	#ads = initialize_potentiometer()
	# Initialize Pygame and the joystick
	pygame.init()
	pygame.joystick.init()

	# Create a joystick object
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	
	for i in range(10000):
		#x_angle = read_potentiometer(ads, 0)
		#y_angle = read_potentiometer(ads, 1)
		x_angle = 0
		y_angle = 0
		print("Angles: ", x_angle, y_angle)
		
		pygame.event.pump()
		joystick_x = joystick.get_axis(2)*(-1)
		joystick_y = joystick.get_axis(3)
		print(joystick_x, joystick_y)
		
		x_motor.update(joystick_x*10, 0)
		y_motor.update(joystick_y*10, 0)
			
		time.sleep(interval)
		
except KeyboardInterrupt:
	print("Keyboard Interrupt")
except Exception as e:
	print(e)
finally:
	print("clean up")
	pygame.quit()
	x_motor.clean_up()
	y_motor.clean_up()
	
