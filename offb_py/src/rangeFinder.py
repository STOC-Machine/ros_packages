#! /usr/bin/env python

import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

class range_finder(object):

    def __init__(self):
        self.TRIG=23
        self.ECHO=24
        self.range = 0
        self.setup()

    def setup(self):
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.output(self.TRIG, False)

    def loop(self):
        self.range = self.rangeFinder()
        print self.range, " cm"
        print self.multiObjectDetection(self.range)

    def activateUltrasonicSensor(self):
        GPIO.output(self.TRIG, False)
        time.sleep(0.000002)
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

    def simpleObjectDetection(self, distance):
        if distance <= 30:
            print "True"
            return True
        else:
            print "False"
            return False

    def multiObjectDetection(self, distance):
        if distance <= 30:
            print "Hault"
            return 0
        elif distance > 30 and distance <= 60:
            print "SLOW DOWN QUICKLY"
            return 1
        elif distance > 60 and distance <= 100:
            print "SLOW DOWN"
            return 2
        elif distance > 100 and distance <= 300:
            print "KEEP GOING CAREFULLY"
            return 3
        else:
            print "KEEP GOING"
            return 4

    def rangeFinder(self):
        sum = 0
        for _ in range(5):
            sum += self.simpleRangeFinder()
            time.sleep(0.1)
        return round(sum/5)

    def simpleRangeFinder(self):
        self.activateUltrasonicSensor()
        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()
        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()
        duration = pulse_end - pulse_start
        distance = duration * 17150
        return round(distance, 2)


if __name__ == "__main__":
    sensor = range_finder()
    sensor.loop()
    GPIO.cleanup()
