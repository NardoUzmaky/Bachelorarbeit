import socket
import subprocess
import json
import time
import struct
import os 
import queue
import threading
import cProfile
from simple_pid import PID
from motordriverboard import MotorControl
from potentiometer import read_potentiometer, init_adc_continous
from ball_control_algorithm import LowPassFilter

from Camera import capture_video

HOST = "192.168.1.3"
#HOST = "10.5.63.57"
PORT = 65432
CHUNK_SIZE = 8192

# = "./images/"
#if not os.path.exists(folder_path):
#	os.mkdir(folder_path)

def receive_data(socket):
	raw_len = socket.recv(4)
	if not raw_len:
		return None
	msg_len = struct.unpack('>I', raw_len)[0]
	return socket.recv(msg_len)

def send_data(socket, data):
	data = struct.pack('>I', len(data)) + data
	socket.sendall(data)
	
def send_image(socket, img_path):
	with open(img_path, 'rb') as f:
		while True:
			bytes_read = f.read(CHUNK_SIZE)
			if not bytes_read:
				break
			socket.sendall(bytes_read)
			
def receive_image(socket, img_size):
	received = 0
	with open("./images/image" + str(i) + ".jpg", 'wb') as f:		
		while received < img_size:
			bytes_read = socket.recv(CHUNK_SIZE)
			if not bytes_read:
				break
			f.write(bytes_read)
			received += len(bytes_read)
			
data_queue = queue.Queue()
shutdown_flag = threading.Event()
x_setpoint = 0
y_setpoint = 0

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
			time.sleep(0.001)
			x_angle = read_potentiometer(x_adc, 1)
			y_angle = read_potentiometer(y_adc, 2)
			
			x_pid.setpoint = x_setpoint
			y_pid.setpoint = y_setpoint
			x_control = x_pid(x_angle)
			y_control = y_pid(y_angle)

			x_motor.update(x_control, x_angle)
			y_motor.update(y_control, y_angle)
			#print("1 Loop Time: ", time.time()-time1)
			
	except KeyboardInterrupt:
		print("Keyboard Interrupt")
	finally:
		print("clean up")
		x_motor.clean_up()
		y_motor.clean_up()

def send_thread(s):
	while not shutdown_flag.is_set():
		time.sleep(0.001)
		#print("looking for data")
		if not data_queue.empty():
				data = data_queue.get()
				json_data = json.dumps(data).encode('utf-8')
				send_data(s, json_data)
			
def receive_thread(s):
	global x_setpoint
	global y_setpoint
	s.settimeout(1)
	while not shutdown_flag.is_set():
		try:
			#print("waiting for data")
			recv_data = receive_data(s)
			if recv_data:
				recv_data = json.loads(recv_data.decode('utf-8'))
				#print(recv_data)
				x_setpoint = recv_data["x_angle_setpoint"]
				y_setpoint = recv_data["y_angle_setpoint"]
		except socket.timeout:
			continue
		except OSError:
			break

def socket_communication():
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((HOST, PORT))
		s.listen()
		conn, addr = s.accept()
		with conn:
			print(f"Connected by {addr}")
			video_thread = threading.Thread(target = capture_video, args=(data_queue,))
			s_thread = threading.Thread(target=send_thread, args=(conn,))
			r_thread = threading.Thread(target=receive_thread, args=(conn,))
			angle_control_thread = threading.Thread(target=angle_control_loop)

			s_thread.start()
			r_thread.start()
			angle_control_thread.start()
			video_thread.start()
			
			try:
				while video_thread.is_alive():
					video_thread.join(timeout=1)
				print("Video thread dead")
			except KeyboardInterrupt:
				print("Shutting down gracefully...")
			shutdown_flag.set()
			angle_control_thread.join()
			s_thread.join()
			r_thread.join()
			print("all threads terminated")
if __name__ == "__main__":

	socket_communication()
		
			
