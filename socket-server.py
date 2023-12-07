import socket
import subprocess
import json
import time
import struct
import os 

HOST = "192.168.1.3"
PORT = 65432
CHUNK_SIZE = 8192

folder_path = "./images/"
if not os.path.exists(folder_path):
	os.mkdir(folder_path)

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

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.bind((HOST, PORT))
	s.listen()
	conn, addr = s.accept()
	with conn:
		time1 = time.perf_counter()
		print(f"Connected by {addr}")
		for i in range(60):
			data = receive_data(conn)	
			string_data = data.decode('utf-8')
			json_data = json.loads(string_data)
			command = json_data["command"]
			number = json_data["number"]
			if json_data["image"] == True:
				img_size = json_data["img_size"]
				receive_image(conn, img_size)
			#result = subprocess.run(command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
			#print("STDOUT:", result.stdout)
			#print("STDERR:", result.stderr)
			conn.sendall(str(number).encode('utf-8'))
		print("Time to send 60 pictures: ", time.perf_counter()-time1)
		
			
