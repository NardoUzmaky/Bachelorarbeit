from simple_pid import PID
from potentiometer import read_potentiometer, init_adc_continous
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

ball_position = [] 
ball_position_times = []
measured_angles = []
measured_angles_times = []
reference_angles = []
reference_angles_times = []

def angle_control_loop():
	# Initialize your PID controller
	x_pid = PID(3.5, 0.3, 0.33, setpoint=0)  # Example coefficients and setpoint
	y_pid = PID(3.5, 0.3, 0.33, setpoint=0)
	x_pid.output_limits = (-20, 20)
	y_pid.output_limits = (-20, 20)
	interval = 0
	global x_setpoint
	global y_setpoint
	
	try:
		values = []

		x_motor = Motor(1)
		y_motor = Motor(2)
		x_adc = init_adc_continous(1)
		y_adc = init_adc_continous(2)
		alpha = 0.3
		lpf = LowPassFilter(alpha)
		
		while not shutdown_flag.is_set():
			#time1 = time.time()
			time.sleep(0.001)
			x_angle = read_potentiometer(x_adc, 1)
			y_angle = read_potentiometer(y_adc, 2)
			
			measured_angles_times.append(time.time())
			measured_angles.append(x_angle)
			
			x_pid.setpoint = x_setpoint
			y_pid.setpoint = y_setpoint
			print(x_setpoint)
			x_control = x_pid(x_angle)
			y_control = y_pid(y_angle)
			#print("speed: ", x_control, y_control)

			x_motor.update(x_control, x_angle)
			y_motor.update(y_control, y_angle)
			#print("1 Loop Time: ", time.time()-time1)
			
	except KeyboardInterrupt:
		print("Keyboard Interrupt")
	finally:
		print("clean up")
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
	x_ball_pid = PID(5, 10, 1, setpoint = 0)
	x_ball_pid.output_limits = (-10, 10)
	
	y_ball_pid = PID(5, 10, 1, setpoint = 0)
	y_ball_pid.output_limits = (-10, 10)
	
	while not shutdown_flag.is_set():
		time.sleep(0.01)
		if not data_queue.empty():
			data = data_queue.get()
			#print("Delay: ", time.time()-data["time"])
			#print("POSITION: ", data["position"])
			x_pos = data["position"][0]
			y_pos = data["position"][1]
			ball_position.append(x_pos)
			ball_position_times.append(data["time"])
			#x_vel = data["speed"][0]
			#y_vel = data["speed"][1]
			x_setpoint = x_ball_pid(x_pos)
			y_setpoint = y_ball_pid(y_pos)
			reference_angles_times.append(time.time())
			reference_angles.append(x_setpoint)
			print("Setpoint: ", x_setpoint)	

			
if __name__ == "__main__":		
	video_thread = threading.Thread(target = capture_video, args=(data_queue,))
	angle_loop_thread = threading.Thread(target = angle_control_loop)
	ball_control_thread = threading.Thread(target = ball_position_loop)

	video_thread.start()
	angle_loop_thread.start()
	ball_control_thread.start()
	try:
		while video_thread.is_alive():
			video_thread.join(timeout=1)
		print("Video thread dead")
	except KeyboardInterrupt:
		print("Shutting down gracefully...")
	shutdown_flag.set()
	angle_loop_thread.join()
	ball_control_thread.join()
	start_point = measured_angles_times[0]

	measured_angles_times = [x - start_point for x in measured_angles_times]
	reference_angles_times = [x - start_point for x in reference_angles_times]
	ball_position_times = [x - start_point for x in ball_position_times]
	ball_position = [x*10 for x in ball_position]

	plt.plot(measured_angles_times, measured_angles, color='green')
	plt.plot(reference_angles_times, reference_angles, color='red')
	plt.plot(ball_position_times, ball_position, color='blue')
	plt.title("Reference Angle vs Measured Angle")
	plt.xlabel("Time [s]")
	plt.ylabel("Angle [deg]")
	plt.show()
