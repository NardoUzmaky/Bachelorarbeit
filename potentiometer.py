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
offset = 0
v0 = 1.62 - offset
v1 = 2.063 -offset
def voltage_to_degrees(voltage):
	return (11)*(voltage - v0)/(v1 - v0)
	
def initialize_potentiometer():
	i2c = busio.I2C(board.SCL, board.SDA)
	ads = ADS.ADS1115(i2c)
	return ads
	
def read_potentiometer(ads, channel = 0):
	chan = None
	if channel == 0:
		chan = AnalogIn(ads, ADS.P0)
	elif channel == 1:
		chan = AnalogIn(ads, ADS.P1)
	else:
		print("Incorrect channel number")
	val = chan.voltage
	return voltage_to_degrees(val)
	
ads = initialize_potentiometer()

print(read_potentiometer(ads, 0))
print(read_potentiometer(ads, 1))
