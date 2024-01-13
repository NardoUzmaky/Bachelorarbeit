from simple_pid import PID
from potentiometer import read_potentiometer, initialize_potentiometer
import matplotlib.pyplot as plt
import time
import numpy as np
import RPi.GPIO as gpio

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None

    def update(self, new_value):
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha * new_value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

 
def init():
	gpio.setmode(gpio.BCM)
	gpio.setup(23, gpio.OUT)
	gpio.setup(22, gpio.OUT)
	gpio.setup(24, gpio.OUT)
	pwm = gpio.PWM(24, 500) #EN
	pwm.start(0)
	return pwm
	
def turn_motor(sec, direction, speed, pwm):
	if speed > 80:
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

	pwm.ChangeDutyCycle(speed)
	time.sleep(sec)	

# Initialize your PID controller
pid = PID(3.5, 0.3, 0.33, setpoint=0)  # Example coefficients and setpoint
pid.output_limits = (-80, 80)

interval = 0

def read_sensor(ads):
    # Replace this with your sensor reading logic
    return read_potentiometer(ads)

def update_actuator(value, direction, pwm):
    # Replace this with your actuator control logic
	print("Speed: ", value)
	turn_motor(interval, direction, abs(value), pwm)
	pass
	
	
try:
	values = []
	pwm = init()
	ads = initialize_potentiometer()
	alpha = 0.3
	lpf = LowPassFilter(alpha)
	signal = []
	signal = [i*0.02 for i in range(500)]
	
	for i in range(200):
		time1 = time.time()
		current_value = read_sensor(ads)
		print("Angle: ", current_value)
		values.append(current_value)
		filtered_value = lpf.update(current_value)
		#values.append(filtered_value)
		#setpoint = signal[i]
		#pid.setpoint = setpoint
		control = pid(filtered_value)
		direction = None
		if control < 0:
			direction = "forward"
		else:
			direction = "reverse"
				
		if direction == "forward" and current_value < -12:
			print("stopper activated")
			break
		elif direction == "reverse" and current_value > 12:
			print("stopper activated")
			break
		elif direction == None:
			print("Invalid direction")
			break
		update_actuator(control, direction, pwm)
		print("1 Loop Time: ", time.time()-time1)
		
except KeyboardInterrupt:
	print("Keyboard Interrupt")
except Exception as e:
	print(e)
finally:
	print("clean up")
	pwm.stop()
	gpio.cleanup()

	#raw_sensor_values = values
	#filtered_values = [lpf.update(value) for value in raw_sensor_values]
	time = np.linspace(0, (len(values)-1)*0.01, len(values))
	plt.plot(time, values)
	plt.title("Step response")
	plt.xlabel("Time [s]")
	plt.ylabel("Angle [deg]")
	plt.show()
	
