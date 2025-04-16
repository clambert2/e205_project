import RPi.GPIO as GPIO
import time
import board
import threading
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3
from rplidar import RPLidar

class AlphaBot2(object, threading.Thread):

		def __init__(self,ain1=12,ain2=13,ena=6,bin1=20,bin2=21,enb=26):
			# Set GPIO pin numbers and PWM frequencies
			self.AIN1 = ain1
			self.AIN2 = ain2
			self.BIN1 = bin1
			self.BIN2 = bin2
			self.ENA = ena
			self.ENB = enb
			self.PA  = 50
			self.PB  = 50

			# Set GPIO mode, warnings, and pin setup
			GPIO.setmode(GPIO.BCM)
			GPIO.setwarnings(False)
			GPIO.setup(self.AIN1,GPIO.OUT)
			GPIO.setup(self.AIN2,GPIO.OUT)
			GPIO.setup(self.BIN1,GPIO.OUT)
			GPIO.setup(self.BIN2,GPIO.OUT)
			GPIO.setup(self.ENA,GPIO.OUT)
			GPIO.setup(self.ENB,GPIO.OUT)
			self.PWMA = GPIO.PWM(self.ENA,500)
			self.PWMB = GPIO.PWM(self.ENB,500)
			self.PWMA.start(self.PA)
			self.PWMB.start(self.PB)
			self.stop()

			# Initialize I2C and LSM6DS3 sensor
			self.i2c = board.I2C()
			self.imu = LSM6DS3(self.i2c)

			# Initialize Lidar
			self.lidar = RPLidar('/dev/ttyUSB0')
			self.lidar.start_motor()
			time.sleep(1)
			



		def forward(self):
			self.PWMA.ChangeDutyCycle(self.PA)
			self.PWMB.ChangeDutyCycle(self.PB)
			GPIO.output(self.AIN1,GPIO.LOW)
			GPIO.output(self.AIN2,GPIO.HIGH)
			GPIO.output(self.BIN1,GPIO.LOW)
			GPIO.output(self.BIN2,GPIO.HIGH)


		def stop(self):
			self.PWMA.ChangeDutyCycle(0)
			self.PWMB.ChangeDutyCycle(0)
			GPIO.output(self.AIN1,GPIO.LOW)
			GPIO.output(self.AIN2,GPIO.LOW)
			GPIO.output(self.BIN1,GPIO.LOW)
			GPIO.output(self.BIN2,GPIO.LOW)

		def backward(self):
			self.PWMA.ChangeDutyCycle(self.PA)
			self.PWMB.ChangeDutyCycle(self.PB)
			GPIO.output(self.AIN1,GPIO.HIGH)
			GPIO.output(self.AIN2,GPIO.LOW)
			GPIO.output(self.BIN1,GPIO.HIGH)
			GPIO.output(self.BIN2,GPIO.LOW)


		def left(self):
			self.PWMA.ChangeDutyCycle(30)
			self.PWMB.ChangeDutyCycle(30)
			GPIO.output(self.AIN1,GPIO.HIGH)
			GPIO.output(self.AIN2,GPIO.LOW)
			GPIO.output(self.BIN1,GPIO.LOW)
			GPIO.output(self.BIN2,GPIO.HIGH)


		def right(self):
			self.PWMA.ChangeDutyCycle(30)
			self.PWMB.ChangeDutyCycle(30)
			GPIO.output(self.AIN1,GPIO.LOW)
			GPIO.output(self.AIN2,GPIO.HIGH)
			GPIO.output(self.BIN1,GPIO.HIGH)
			GPIO.output(self.BIN2,GPIO.LOW)

		def setPWMA(self,value):
			self.PA = value
			self.PWMA.ChangeDutyCycle(self.PA)

		def setPWMB(self,value):
			self.PB = value
			self.PWMB.ChangeDutyCycle(self.PB)	

		def setMotor(self, left, right):
			if((right >= 0) and (right <= 100)):
				GPIO.output(self.AIN1,GPIO.HIGH)
				GPIO.output(self.AIN2,GPIO.LOW)
				self.PWMA.ChangeDutyCycle(right)
			elif((right < 0) and (right >= -100)):
				GPIO.output(self.AIN1,GPIO.LOW)
				GPIO.output(self.AIN2,GPIO.HIGH)
				self.PWMA.ChangeDutyCycle(0 - right)
			if((left >= 0) and (left <= 100)):
				GPIO.output(self.BIN1,GPIO.HIGH)
				GPIO.output(self.BIN2,GPIO.LOW)
				self.PWMB.ChangeDutyCycle(left)
			elif((left < 0) and (left >= -100)):
				GPIO.output(self.BIN1,GPIO.LOW)
				GPIO.output(self.BIN2,GPIO.HIGH)
				self.PWMB.ChangeDutyCycle(0 - left)

if __name__=='__main__':

	Ab = AlphaBot2()
	Ab.left()
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		GPIO.cleanup()
