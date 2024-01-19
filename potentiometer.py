import time
import busio
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import math

# need to calibrate these values
# 0 degrees: 1.64 Volts
# 11 degrees: 2.0836 Volts
# -14.35 degrees: 1.069 Volts
#x-axis:
v0_x = 1.64 
v1_x = 2.086
#y-axis:
v0_y = 1.65
v1_y = 2.187
def voltage_to_degrees_x(voltage):
	return (11)*(voltage - v0_x)/(v1_x - v0_x)
	
def voltage_to_degrees_y(voltage):
	return (11)*(voltage - v0_y)/(v1_y - v0_y)
	
def initialize_potentiometer():
	i2c = busio.I2C(board.SCL, board.SDA)
	ads = ADS.ADS1115(i2c)
	return ads
	
def read_potentiometer(chan, axis = 1):
	val = chan.voltage
	if axis == 1:
		return voltage_to_degrees_x(val)
	if axis == 2:
		return voltage_to_degrees_y(val)
	raise ValueError(f"Invalid axis: {axis}")
	
def init_adc_continous(axis): #axis = 1: adc for potentiometer of x-axis, axis = 1: adc for potentiometer of y-axis
	i2c = busio.I2C(board.SCL, board.SDA)
	if axis == 1:
		ads = ADS.ADS1115(i2c) #this will take the default address of the ADS1115, which is 0x48
		ads.data_rate = 128
		ads.mode = ADS.Mode.CONTINUOUS
		chan = AnalogIn(ads, ADS.P0)
		return chan # get voltage with chan.voltage
	if axis == 2:
		ads = ADS.ADS1115(i2c, address = 0x49) #to connect a second ADC we need to specify an address
		ads.data_rate = 128
		ads.mode = ADS.Mode.CONTINUOUS
		chan = AnalogIn(ads, ADS.P0)
		return chan
	print("Invalid axis")	

if __name__ == "__main__":	
	#ads = initialize_potentiometer()
	#ads.gain = 1
	
	#chan = AnalogIn(ads, ADS.P0)
	chan = init_adc_continous(2)
	try:
		while True:
			time1 = time.time()
			print(chan.voltage)
			print("TIME: ", time.time()-time1)
	except KeyboardInterrupt:
		print("interpupt")
	
	#print(read_potentiometer(ads, 0))
	#print(read_potentiometer(ads, 1))

