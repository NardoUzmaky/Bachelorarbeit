import pygame
import RPi.GPIO as gpio
from potentiometer import read_potentiometer, initialize_potentiometer
import time


def init_motor1(): #x-axis
	gpio.setmode(gpio.BCM)
	gpio.setup(23, gpio.OUT) #in1
	gpio.setup(22, gpio.OUT) #in2
	gpio.setup(24, gpio.OUT)
	pwm = gpio.PWM(24, 300) #EN
	pwm.start(0)
	return pwm
	
def init_motor2():
	gpio.setmode(gpio.BCM)
	gpio.setup(19, gpio.OUT) #in1
	gpio.setup(20, gpio.OUT) #in2
	gpio.setup(16, gpio.OUT)
	pwm = gpio.PWM(16, 300) #EN
	pwm.start(0)
	return pwm
	
def turn_motor(sec, direction, speed, pwm, motor=1):
	if speed > 10:
		print("Too fast")
		return
	if motor == 1:
		if direction == "forward":
			#print("forward")
			gpio.output(23, True) #In1
			gpio.output(22, False)#In2
		elif direction == "reverse":
			#print("reverse")
			gpio.output(23, False) #In1
			gpio.output(22, True)#In2
		else:
			print("Enter valid Direction")
			return
	if motor == 2:
		if direction == "forward":
			#print("forward")
			gpio.output(19, True) #In1
			gpio.output(20, False)#In2
		elif direction == "reverse":
			#print("reverse")
			gpio.output(19, False) #In1
			gpio.output(20, True)#In2
		else:
			print("Enter valid Direction")
			return

	pwm.ChangeDutyCycle(speed)
	#time.sleep(sec)	
    
try:
	interval = 0.01
	pwm1 = init_motor1()
	pwm2 = init_motor2()
	ads = initialize_potentiometer()
	# Initialize Pygame and the joystick
	pygame.init()
	pygame.joystick.init()

	# Create a joystick object
	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	
	for i in range(10000):
		x_angle = read_potentiometer(ads, 0)
		y_angle = read_potentiometer(ads, 1)
		print("Angles: ", x_angle, y_angle)
		
		pygame.event.pump()
		joystick_x = joystick.get_axis(3)
		joystick_y = joystick.get_axis(4)*(-1)
		print(joystick_x, joystick_y)
		
		x_direction = None
		y_direction = None
		if joystick_x < 0:
			x_direction = "forward"
		else:
			x_direction = "reverse"
		if joystick_y < 0:
			y_direction = "forward"
		else:
			y_direction = "reverse"
			
			
		if x_direction == "forward" and x_angle < -12:
			print("stopper activated")
			turn_motor(interval, x_direction, 0, pwm1, 1)
			continue
		elif x_direction == "reverse" and x_angle > 12:
			print("stopper activated")
			turn_motor(interval, x_direction, 0, pwm1, 1)
			continue
		elif x_direction == None:
			print("Invalid direction")
			break
			
		if y_direction == "forward" and y_angle < -12:
			print("stopper activated")
			turn_motor(interval, y_direction, 0, pwm2, 2)
			continue
		elif y_direction == "reverse" and y_angle > 12:
			print("stopper activated")
			turn_motor(interval, y_direction, 0, pwm2, 2)
			continue
		elif y_direction == None:
			print("Invalid direction")
			break	
			
		turn_motor(interval, x_direction, abs(joystick_x*10), pwm1, 1)
		turn_motor(interval, y_direction, abs(joystick_y*10), pwm2, 2)
		#time.sleep(interval)
		
except KeyboardInterrupt:
	print("Keyboard Interrupt")
except Exception as e:
	print(e)
finally:
	print("clean up")
	pygame.quit()
	pwm1.stop()
	pwm2.stop()
	gpio.cleanup()
	
