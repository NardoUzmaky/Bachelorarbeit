from simple_pid import PID
from potentiometer import read_potentiometer, init_adc_continous
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

class Motor:
	def __init__(self, axis):
		if axis == 1:
			self.pwm = init_motor1()
			self.in1 = 23
			self.in2 = 22
		elif axis == 2:
			self.pwm = init_motor2()
			self.in1 = 19
			self.in2 = 20
		else:
			raise ValueError(f"Invalid axis: {axis}")
		self.axis = axis
		self.direction = None
		self.speed = 0
		
	def update(self, control_input, angle):
		if control_input < 0:
			new_direction = "forward"
		else:
			new_direction = "reverse"
			
		if new_direction == "forward" and angle < -12:
				print("stopper activated")
				self.update_motor(new_direction, 0)
				return
		elif new_direction == "reverse" and angle > 12:
				print("stopper activated")
				self.update_motor(new_direction, 0)
				return
		self.update_motor(new_direction, abs(control_input))
				
	def update_motor(self, direction, speed):
		if direction == "forward":
			gpio.output(self.in1, True) #In1
			gpio.output(self.in2, False) #In2
		elif direction == "reverse":
			gpio.output(self.in1, False) #In1
			gpio.output(self.in2, True) #In2
		else:
			print("Invalid Direction")
			return
		print("Speed: ", speed)	
		self.pwm.ChangeDutyCycle(speed)
		self.speed = speed
		self.direction = direction
		
	def clean_up(self):
		self.pwm.stop()	
		
def init_motor1(): #x-axis
	gpio.setmode(gpio.BCM)
	gpio.setup(23, gpio.OUT) #in1
	gpio.setup(22, gpio.OUT) #in2
	gpio.setup(24, gpio.OUT)
	pwm = gpio.PWM(24, 500) #EN
	pwm.start(0)
	return pwm
	
def init_motor2():
	gpio.setmode(gpio.BCM)
	gpio.setup(19, gpio.OUT) #in1
	gpio.setup(20, gpio.OUT) #in2
	gpio.setup(16, gpio.OUT)
	pwm = gpio.PWM(16, 500) #EN
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
pid.output_limits = (-10, 10)

interval = 0
	
try:
	values = []
	motor = Motor(1)
	adc = init_adc_continous(motor.axis)
	alpha = 0.3
	lpf = LowPassFilter(alpha)
	signal = []
	signal = [i*0.02 for i in range(500)]
	
	for i in range(20000):
		time1 = time.time()
		current_value = read_potentiometer(adc, motor.axis)
		print("Angle: ", current_value)
		values.append(current_value)
		filtered_value = lpf.update(current_value)
		#values.append(filtered_value)
		#setpoint = signal[i]
		#pid.setpoint = setpoint
		control = pid(filtered_value)
		motor.update(control, current_value)
		#update_actuator(control, direction, pwm)
		print("1 Loop Time: ", time.time()-time1)
		
except KeyboardInterrupt:
	print("Keyboard Interrupt")
except Exception as e:
	print(e)
finally:
	print("clean up")
	#pwm.stop()
	motor.clean_up()
	gpio.cleanup()

	#raw_sensor_values = values
	#filtered_values = [lpf.update(value) for value in raw_sensor_values]
	time = np.linspace(0, (len(values)-1)*0.01, len(values))
	plt.plot(time, values)
	plt.title("Step response")
	plt.xlabel("Time [s]")
	plt.ylabel("Angle [deg]")
	plt.show()
	
