import cv2 as cv
import os
import subprocess
import time
import numpy as np

np.set_printoptions(precision=2)
#Takes inputs as vector [x, y] in pixels:
#Outputs speed as vector [v_x, v_y] in pixels/second:
def calculate_speed(pos1, pos2, time1, time2):
	x_distance = pos2[0]-pos1[0]
	y_distance = pos2[1]-pos1[1]
	time_diff = time2-time1
	return [x_distance/time_diff, y_distance/time_diff]

#translates from pixels to meters
def pixel2meter(velocity):
	v = [0.00125*x for x in velocity]
	return v
	
#gives position, where (0, 0) is the center of the plate
def center_position(position, center):
	p = np.subtract(position,center)
	return np.matmul([[0, -1], [-1, 0]], p)
	

def detect_position(frame, Width, Height):
	#frame = frame[90:650, 377:937]
	frame = frame[int(.116666*Height):int(0.955*Height), int(0.28333*Width):int(0.75*Width)]
	#frame = frame[:, int(0.23*Width):int(0.8*Width)]
	
	gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

	_, thresh = cv.threshold(gray, 140, 255, cv.THRESH_BINARY_INV)
	kernel = np.ones((5, 5), np.uint8)

	thresh = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
	contours, _ = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

	for contour in contours:
		area = cv.contourArea(contour)
		if 100 < area:
			perimeter = cv.arcLength(contour, True)
			circularity = 4*np.pi*area/(perimeter**2) if perimeter > 0 else 0
			cv.drawContours(frame, [contour], -1, (0, 255, 0), 2)
			if 0.6 < circularity < 1 and perimeter > 80:
				M = cv.moments(contour)
				if M['m00'] != 0:
					cx = int(M['m10'] / M['m00'])
					cy = int(M['m01'] / M['m00'])
					cv.circle(frame, (cx, cy), 3, (255, 0, 0), -1)
					#cv.imshow('Frame', frame)
					return [cx, cy]
	return

def capture_video(data_queue = None):
	import time 
	cap = cv.VideoCapture(0, cv.CAP_V4L2) 

	Width = 1280
	Height = 720
	FPS = 60
	number_of_frames = 30000

	cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
	
	#cap.set(cv.CAP_PROP_EXPOSURE, 100)
	subprocess.check_call("v4l2-ctl -d /dev/video0 --set-ctrl white_balance_automatic=0", shell=True)
	subprocess.check_call("v4l2-ctl -d /dev/video0 --set-ctrl white_balance_temperature=3500", shell=True)
	cap.set(cv.CAP_PROP_FRAME_WIDTH, Width)
	cap.set(cv.CAP_PROP_FRAME_HEIGHT, Height)
	cap.set(cv.CAP_PROP_FPS, FPS)
	print("Resolution: ", cap.get(cv.CAP_PROP_FRAME_WIDTH), cap.get(cv.CAP_PROP_FRAME_HEIGHT))
	print("FPS: ", cap.get(cv.CAP_PROP_FPS))
	print("Exposure time: ", cap.get(cv.CAP_PROP_EXPOSURE))

	fourcc = cv.VideoWriter_fourcc(*'H264')
	#out = cv.VideoWriter('output.avi', fourcc, FPS, (Width, Height))

	if not cap.isOpened():
		print("Cannot open Camera")
		exit()

	cap.read()
	cap.read()
	start = time.perf_counter()
	previous_time = time.time()
	previous_pos = [0, 0]
	
	for i in range(number_of_frames):
		
		#print(1/(current_time-previous_time))
		current_time = time.time()
		ret, frame = cap.read()

		if not ret:
			print("Can't receive frame (stream end?). Exiting ...")
			break
		#cv.imshow('frame', frame)
		#out.write(frame)
		data = {}
		current_pos = detect_position(frame, Width, Height)
		if current_pos is not None:
			current_pos = center_position(current_pos, [295, 310])
			current_pos = pixel2meter(current_pos)
			formatted_list = [format(num, ".2f") for num in current_pos]
			print("Ball position: ", formatted_list)
			data["time"] = time.time()#current_time
			data["position"] = current_pos
		else:
			print("Could not find Ball")
			current_pos = previous_pos
			data["time"] = time.time()
			data["position"] = previous_pos

		if i != 0 and current_pos is not None:
			speed = calculate_speed(previous_pos, current_pos, previous_time, current_time)
			data["speed"] = speed
		else:
			data["speed"] = [0, 0]
		#print(data)
		if data_queue:
			data_queue.put(data)
		previous_pos = current_pos
		previous_time = current_time
		if cv.waitKey(1) == ord('q'):
			break
		#print("Time to process image: ", time.time()-previous_time)

	time = time.perf_counter()-start
	print("time :", time)
	print("real fps: ", number_of_frames/time)
	#out.release()
	cap.release()
	cv.destroyAllWindows()
	
if __name__ == "__main__":
	capture_video()
#print(center_position([0, 0], [296, 285]))
