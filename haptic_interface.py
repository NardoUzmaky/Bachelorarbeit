import pygame
from potentiometer import read_potentiometer, init_adc_continous
import time
from gpiozero import OutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory as pin_factory
from motordriverboard import MotorControl

factory = pin_factory()


try:
	interval = 0.01
	#intizalize motor interface
	x_motor = MotorControl(1)
	y_motor = MotorControl(2)
	#initialize ADCs
	x_adc = init_adc_continous(1)
	y_adc = init_adc_continous(2)
	# Initialize Pygame and the joystick
	pygame.init()
	pygame.joystick.init()

	# Create a joystick object
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	
	for i in range(10000):
		x_angle = read_potentiometer(x_adc, 1)
		y_angle = read_potentiometer(y_adc, 2)
		print("Angles: ", x_angle, y_angle)
		
		pygame.event.pump()
		joystick_x = joystick.get_axis(2)*(-1)
		joystick_y = joystick.get_axis(3)
		print(joystick_x, joystick_y)
		
		x_motor.update(joystick_x*10, x_angle)
		y_motor.update(joystick_y*10, y_angle)
			
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
	
