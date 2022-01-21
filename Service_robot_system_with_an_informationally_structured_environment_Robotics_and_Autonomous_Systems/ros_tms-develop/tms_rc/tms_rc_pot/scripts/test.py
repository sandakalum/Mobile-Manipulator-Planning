#!/usr/bin/env python
from std_srvs.srv import Empty
import wiringpi2 as wpi
import rospy
import time

#leds = [7,0,10]
#leds = [7,0,2]
leds = [7,2,3]

def led_initialize():
  x=0
  for x in leds:
    wpi.pinMode(x,1)
    print(str(x) + " led initialize")
    wpi.digitalWrite(x, 0)

def led_test():
  led_initialize()
  while True:
    time.sleep(2)
    wpi.digitalWrite(leds[0],1)
    wpi.digitalWrite(leds[1],1)
    wpi.digitalWrite(leds[2],1)
    print("led on")
    time.sleep(2)
    wpi.digitalWrite(leds[0],0)
    wpi.digitalWrite(leds[1],0)
    wpi.digitalWrite(leds[2],0)
    print("led off")

if __name__ == "__main__":
    print "led_tika"
    wpi.wiringPiSetup()
    led_test()
