#This is a simple test for ultrasonic sensor

from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=22, trigger=27, max_distance =4)
while True:
    print('Distance: ', sensor.distance * 100)
    sleep(1)
