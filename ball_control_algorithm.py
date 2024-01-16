from simple_pid import PID
from potentiometer import read_potentiometer, initialize_potentiometer
import matplotlib.pyplot as plt
import time
import numpy as np
import RPi.GPIO as gpio
import queue
import threading
from Camera import capture_video

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
		self.direction = None
		self.speed = 0
		
	def update(self, control_input, angle):
		if control_input < 0:
			new_direction = "forward"
		else:
			new_direction = "reverse"
			
		if new_direction == "forward" and angle < -12:
				print("stopper activated")
				self.update_motor(self, new_direction, 0)
		elif new_direction == "reverse" and angle > 12:
				print("stopper activated")
				self.update_motor(self, new_direction, 0)
				
	def update_motor(self, direction, speed):
		if direction == "forward":
			gpio.output(self.in1, True) #In1
			gpio.output(self.in2, False) #In2
		elif direction == "reverse":
			gpio.output(self.in1, False) #In1
			gpio.output(self.in2, True) #In2
		else:
			print("Enter valid Direction")
			return
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
	
def turn_motor(direction, speed, pwm, motor=1):
	if speed > 50:
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

def angle_control_loop():
	# Initialize your PID controller
	x_pid = PID(3.5, 0.3, 0.33, setpoint=0)  # Example coefficients and setpoint
	y_pid = PID(3.5, 0.3, 0.33, setpoint=0)
	x_pid.output_limits = (-10, 10)
	y_pid.output_limits = (-10, 10)
	interval = 0
	global x_setpoint
	global y_setpoint
	
	try:
		values = []
		#pwm1 = init_motor1()
		#pwm2 = init_motor2()
		x_motor = Motor(1)
		y_motor = Motor(2)
		ads = initialize_potentiometer()
		alpha = 0.3
		lpf = LowPassFilter(alpha)
		
		while not shutdown_flag.is_set():
			time1 = time.time()
			x_angle = read_potentiometer(ads, 0)
			y_angle = read_potentiometer(ads, 1)
			#print("Angle: ", current_value)
			#values.append(current_value)
			#filtered_value = lpf.update(current_value)
			#values.append(filtered_value)
			x_pid.setpoint = x_setpoint
			y_pid.setpoint = y_setpoint
			
			x_control = x_pid(x_angle)
			y_control = y_pid(y_angle)
			"""
			x_direction = None
			y_direction = None
			if x_control < 0:
				x_direction = "forward"
			else:
				x_direction = "reverse"
			if y_control < 0:
				y_direction = "forward"
			else:
				y_direction = "reverse"
					
			if x_direction == "forward" and x_angle < -12:
				print("stopper activated")
				turn_motor(x_direction, 0, pwm1, 1)
				continue
			elif x_direction == "reverse" and x_angle > 12:
				print("stopper activated")
				turn_motor(x_direction, 0, pwm1, 1)
				continue
			elif x_direction == None:
				print("Invalid direction")
				break
				
			if y_direction == "forward" and y_angle < -12:
				print("stopper activated")
				turn_motor(y_direction, 0, pwm2, 2)
				continue
			elif y_direction == "reverse" and y_angle > 12:
				print("stopper activated")
				turn_motor(y_direction, 0, pwm2, 2)
				continue
			elif y_direction == None:
				print("Invalid direction")
				break	
			turn_motor(x_direction, abs(x_control), pwm1, 1)
			turn_motor(y_direction, abs(y_control), pwm2, 2)
			"""
			x_motor.update(x_control, x_angle)
			y_motor.update(y_control, y_angle)
			time.sleep(0.01)
			print("1 Loop Time: ", time.time()-time1)
			
	except KeyboardInterrupt:
		print("Keyboard Interrupt")
	except Exception as e:
		print(e)
	finally:
		print("clean up")
		#pwm1.stop()
		#pwm2.stop()
		gpio.cleanup()
		x_motor.clean_up()
		y_motor.clean_up()

		#time = np.linspace(0, (len(values)-1)*0.01, len(values))
		#plt.plot(time, values)
		#plt.title("Step response")
		#plt.xlabel("Time [s]")
		#plt.ylabel("Angle [deg]")
		#plt.show()
		
data_queue = queue.Queue()
shutdown_flag = threading.Event()
x_setpoint = 0
y_setpoint = 0

def ball_position_loop():
	global x_setpoint
	global y_setpoint
	x_ball_pid = PID(1, 1, 1, setpoint = 0)
	x_ball_pid.output_limits = (-14, 14)
	
	y_ball_pid = PID(1, 1, 1, setpoint = 0)
	y_ball_pid.output_limits = (-14, 14)
	
	while not shutdown_flag.is_set():
		time.sleep(0.01)
		if not data_queue.empty():
			data = data_queue.get()
			print("POSITION: ", data["position"])
			x_pos = data["position"][0]
			y_pos = data["position"][1]
			#x_vel = data["speed"][0]
			#y_vel = data["speed"][1]
			x_setpoint = x_ball_pid(x_pos)
			y_setpoint = y_ball_pid(y_pos)
			
if __name__ == "__main__":			
	video_thread = threading.Thread(target = capture_video, args=(data_queue,))
	angle_loop_thread = threading.Thread(target = angle_control_loop)
	ball_control_thread = threading.Thread(target = ball_position_loop)

	video_thread.start()
	angle_loop_thread.start()
	ball_control_thread.start()
	try:
		while True:
			time.sleep(0.1)
	except KeyboardInterrupt:
		shutdown_flag.set()
	video_thread.join()
	angle_loop_thread.join()
	ball_control_thread.join()
