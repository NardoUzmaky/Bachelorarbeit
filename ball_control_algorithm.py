from Camera import capture_video
from motordriverboard import MotorControl
from potentiometer import read_potentiometer, init_adc_continous
from simple_pid import PID
import matplotlib.pyplot as plt
import time
import numpy as np
import RPi.GPIO as gpio
import queue
import threading
from motordriverboard import MotorControl

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


x_ball_position = [] 
y_ball_position = []
ball_position_times = []
measured_angles = []
measured_angles_times = []
reference_angles = []
reference_angles_times = []
velocities = []
velocities_times = []

def angle_control_loop():
	# Initialize your PID controller
	x_pid = PID(7, 0.6, 0.66, setpoint=0)  # Example coefficients and setpoint
	y_pid = PID(7, 0.6, 0.66, setpoint=0)
	x_pid.output_limits = (-45, 45)
	y_pid.output_limits = (-45, 45)
	interval = 0
	global x_setpoint
	global y_setpoint
	
	try:
		values = []

		x_motor = MotorControl(1)
		y_motor = MotorControl(2)
		x_adc = init_adc_continous(1)
		y_adc = init_adc_continous(2)
		alpha = 0.3
		lpf = LowPassFilter(alpha)
		
		while not shutdown_flag.is_set():
			#time1 = time.time()
			time.sleep(0.001)
			x_angle = read_potentiometer(x_adc, 1)
			y_angle = read_potentiometer(y_adc, 2)
			#print("ANGLE: ", x_angle)
			
			measured_angles_times.append(time.time())
			measured_angles.append(x_angle)
			
			x_pid.setpoint = x_setpoint
			y_pid.setpoint = y_setpoint
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
	x_ball_pid = PID(20, 1, 1, setpoint = 0)
	x_ball_pid.output_limits = (-10, 10)
	
	y_ball_pid = PID(20, 1, 1, setpoint = 0)
	y_ball_pid.output_limits = (-10, 10)
	
	x_lowpass = LowPassFilter(0.5)
	y_lowpass = LowPassFilter(0.5)
	start_time = time.time()
	while not shutdown_flag.is_set():
		time.sleep(0.001)
		if not data_queue.empty():
			data = data_queue.get()
			current_time = time.time()
			x_ball_pid.setpoint = 0.15*np.sin(current_time-start_time)
			y_ball_pid.setpoint = 0.15*np.cos(current_time-start_time)
			#print("Delay: ", time.time()-data["time"])
			x_pos = data["position"][0]
			y_pos = data["position"][1]
			x_ball_position.append(x_pos)
			y_ball_position.append(y_pos)
			ball_position_times.append(data["time"])
			x_vel = data["speed"][0]
			y_vel = data["speed"][1]
			x_vel = x_lowpass.update(x_vel)
			y_vel = y_lowpass.update(y_vel)
			velocities.append(x_vel)
			velocities_times.append(data["time"])
			print("speed: ", x_vel)
			x_setpoint = x_ball_pid(x_pos) -x_vel*20
			if x_setpoint>10:
				x_setpoint = 10
			elif x_setpoint<-10:
				x_setpoint= -10
				
			y_setpoint = y_ball_pid(y_pos) -y_vel*20
			if y_setpoint>10:
				y_setpoint = 10
			elif y_setpoint<-10:
				y_setpoint= -10
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
	velocities_times = [x-start_point for x in velocities_times]
	"""
	fig, ax1 = plt.subplots()
	ax1.plot(measured_angles_times, measured_angles, color='green')
	ax1.plot(reference_angles_times, reference_angles, color='red')
	ax2 = ax1.twinx()
	ax2.plot(ball_position_times, x_ball_position, color='blue')
	#ax2.plot(velocities_times, velocities, color='pink')
	plt.title("Reference Angle vs Measured Angle")
	ax1.set_xlabel("Time [s]")
	ax1.set_ylabel("Angle [deg]")
	ax2.set_ylabel("Distance from Center [m]")
	ax1.legend(['meas. angle', 'ref. angle'])
	ax2.legend(['dist. from center'], loc='lower right')
	ax1.set_ylim(12, -12)
	ax2.set_ylim(-0.35, 0.35) 
	"""
	fig, ax = plt.subplots()
	circle = plt.Circle((0, 0), 0.15, color='red', fill=False)
	ax.add_patch(circle)
	ax.set_aspect('equal', adjustable='box')
	ax.plot(x_ball_position, y_ball_position, color='blue')
	ax.set_ylim(-0.35, 0.35)
	ax.set_xlim(-0.35, 0.35)
	ax.set_ylabel("Y-Coordinate [m]")
	ax.set_xlabel("X-Coordinate [m]")
	plt.legend(['Circle with Radius of 15cm', 'Ball Path'])
	plt.show()
