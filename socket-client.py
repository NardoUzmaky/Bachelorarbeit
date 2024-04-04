import socket
import json
import os
import struct
import queue
import threading
import time
import csv
from simple_pid import PID
import numpy as np

HOST = "192.168.1.3"
PORT = 65432

CHUNK_SIZE = 8192
#img_path = "image.jpg"
#filesize = os.path.getsize(img_path)

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None
        
    def update(self, new_value):
        if self.filtered_value is None:
            self.filtered_value = new_value
        else:
            self.filtered_value = self.alpha*new_value + (1-self.alpha)*self.filtered_value
        return self.filtered_value

def send_data(socket, data): #Used for sending json data, so receiver knows how many bytes
    data = struct.pack('>I', len(data)) + data #add length (amount of bytes) of data in front of data 
    socket.sendall(data)

def send_image(socket, img_path):
    with open(img_path, 'rb') as f:
        while True:
            bytes_read = f.read(CHUNK_SIZE)
            if not bytes_read:
                break  # File sending is done
            socket.sendall(bytes_read)

def receive_image(socket, img_size, img_format):
    received = 0
    with open(f"./images/img.{img_format}", "wb") as f:
        while received < img_size:
            bytes_read = socket.recv(CHUNK_SIZE)
            if not bytes_read:
                break
            f.write(bytes_read)
            received += len(bytes_read)
            
def receive_data(socket):
    raw_len = socket.recv(4)
    if not raw_len:
        return None
    msg_len = struct.unpack('>I', raw_len)[0]
    return socket.recv(msg_len)

data_queue = queue.Queue()
shutdown_flag = threading.Event()
ball_state = {"position": [0,0], "velocity": [0,0]}
x_angle_setpoint = 0
y_angle_setpoint = 0

def send_thread(s):
    while not shutdown_flag.is_set():
        time.sleep(0.005)
        data = {"x_angle_setpoint": x_angle_setpoint, "y_angle_setpoint": y_angle_setpoint}
        json_data = json.dumps(data).encode('utf-8')
        send_data(s, json_data)
        #send_image(s, img_path)
        
def receive_thread(s):
    global ball_state
    with open('data.csv', 'w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(["time", "position", "speed"])
        
    while not shutdown_flag.is_set():
        print("waiting for data")
        recv_data = receive_data(s)
        recv_data = json.loads(recv_data.decode('utf-8'))
        #print(recv_data)
        ball_state["position"] = recv_data["position"]
        ball_state["velocity"] = recv_data["speed"]
        with open('data.csv', 'a') as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow(recv_data.values())
            
def ball_controller():
    global ball_state
    global x_angle_setpoint
    global y_angle_setpoint
    
    x_lowpass = LowPassFilter(0.5)
    y_lowpass = LowPassFilter(0.5)
    x_pid = PID(20, 1, 1, setpoint=0)
    x_pid.output_limits = (-10, 10)
    y_pid = PID(20, 1, 1, setpoint=0)
    y_pid.output_limits = (-10, 10)
    start_time = time.time()
    while not shutdown_flag.is_set():
        time.sleep(0.001)
        current_time = time.time()-start_time
        x_pid.setpoint = 0.15*np.sin(current_time)
        y_pid.setpoint = 0.15*np.cos(current_time)
        
        x_pos = ball_state["position"][0]
        y_pos = ball_state["position"][1]
        x_vel = ball_state["velocity"][0]
        y_vel = ball_state["velocity"][1]
        
        x_vel = x_lowpass.update(x_vel)
        y_vel = y_lowpass.update(y_vel)
        
        x_angle_setpoint = x_pid(x_pos)-20*x_vel
        y_angle_setpoint = y_pid(y_pos)-20*y_vel
        
        if x_angle_setpoint > 10:
            x_angle_setpoint = 10
        if x_angle_setpoint < -10:
            x_angle_setpoint = -10
            
        if y_angle_setpoint > 10:    
            y_angle_setpoint = 10
        if y_angle_setpoint < -10:
            y_angle_setpoint = -10
  
def socket_communication(): 
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        send_t = threading.Thread(target=send_thread, args=(s,))
        receive_t = threading.Thread(target=receive_thread, args=(s,))
        ball_control_thread = threading.Thread(target=ball_controller)

        send_t.start()
        receive_t.start()
        ball_control_thread.start()
        print("threads started")
        try:
            while ball_control_thread.is_alive():
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down...")
        shutdown_flag.set()
        ball_control_thread.join()
        send_t.join()
        receive_t.join()
            
socket_communication()
