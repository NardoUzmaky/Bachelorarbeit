import RPi.GPIO as gpio
import time
 
def init():
 gpio.setmode(gpio.BCM)
 gpio.setup(23, gpio.OUT)
 gpio.setup(22, gpio.OUT)
 gpio.setup(27, gpio.OUT)
 
def forward(sec):
	init()
	gpio.output(23, True) #In1
	gpio.output(22, False)#In2
	pwm = gpio.PWM(27, 1000) #EN
	pwm.start(0)
	pwm.ChangeDutyCycle(100)
	time.sleep(sec)
 
def reverse(sec):
 init()
 gpio.output(23, False)
 gpio.output(22, True)
 #gpio.output(23, False) 
 #gpio.output(24, True)
 time.sleep(sec)
 
try: 
	print("forward")
	forward(100)
	print("reverse")
	reverse(2)
except KeyboardInterrupt:
	print("Keyboard interrupt")
except Exception as e:
	print(e)
finally:
	print("clean up")
	gpio.cleanup()
