import socket
import json
import os
import struct
import queue
import threading
import time

HOST = "192.168.1.3"
PORT = 65432

CHUNK_SIZE = 8192
img_path = "image.jpg"
filesize = os.path.getsize(img_path)

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

def send_thread(s):
    while not shutdown_flag.is_set():
        time.sleep(0.1)
        if not data_queue.empty():
            data = data_queue.get()
            json_data = json.dumps(data).encode('utf-8')
            send_data(s, json_data)
            #send_image(s, img_path)
        
def receive_thread(s):
    while not shutdown_flag.is_set():
        print("waiting for data")
        recv_data = receive_data(s)
        if recv_data:
            recv_data = json.loads(recv_data.decode('utf-8'))
        print(recv_data)
        print("Time delay: ", time.time() - recv_data["time"])
  
def socket_communication(): 
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        send_t = threading.Thread(target=send_thread, args=(s,))
        receive_t = threading.Thread(target=receive_thread, args=(s,))

        send_t.start()
        receive_t.start()
        print("threads started")
        try:
            while True:
                data = {"test": "test"}
                data_queue.put(data)
                time.sleep(0.01)
        except KeyboardInterrupt:
            shutdown_flag.set()
            send_t.join()
            receive_t.join()
            
socket_communication()