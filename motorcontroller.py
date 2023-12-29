import RPi.GPIO as gpio
import time
from potentiometer import read_potentiometer
 
def init():
 gpio.setmode(gpio.BCM)
 gpio.setup(23, gpio.OUT)
 gpio.setup(22, gpio.OUT)
 gpio.setup(24, gpio.OUT)
 
 
def turn_motor(sec, direction, speed):
	#init()
	if speed > 50:
		print("Too fast")
		return
	
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
	pwm = gpio.PWM(24, 1000) #EN
	pwm.start(0)
	pwm.ChangeDutyCycle(speed)
	interval = 0.01 # measurement interval
	"""
	for i in range(int(sec/interval)):
		val = read_potentiometer()
		if direction == "forward" and val < -12:
			print("stopper activated")
			break
		if direction == "reverse" and val > 12:
			print("stopper activated")
			break
		time.sleep(interval)
	"""
	time.sleep(sec)	
	pwm.ChangeDutyCycle(0)

def activate_motor(time, direction, speed):
	try: 
		init()
		turn_motor(time, direction, speed)
		#turn_motor(5, "forward", 3)
		#turn_motor(5, "reverse", 3)
	except KeyboardInterrupt:
		print("Keyboard interrupt")
	except Exception as e:
		print(e)
	finally:
		print("clean up")
		gpio.cleanup()
	
#activate_motor(5, "reverse", 3)
