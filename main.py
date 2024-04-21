#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import struct
import sys
import time
from ctypes import *

lib = CDLL("./newlx16a.so")

def connect_servo_controller(filename):
    return lib.IO_init(c_char_p(filename.encode('ascii')))

class Servo():
    def __init__(self, id):
        self.id = id

    def setServoID(self, new_id):
        lib.setServoID(self.id, new_id)

    def move(self, position, time=0):
        lib.move(self.id, position, time)

    def getMove(self):
        data = lib.getMove(self.id)
        pos = data >> 16
        time = data & 0xffff
        return pos, time
        
    def movePrepare(self, position, time):
        lib.movePrepare(self.id, position, time)

    def getPreparedMove(self):
        data = lib.getPreparedMove(self.id)
        pos = data >> 16
        time = data & 0xffff
        return pos, time

    def moveStart(self):
        lib.moveStart(self.id)

    def moveStop(self):
        lib.moveStop(self.id)

    def setPositionOffset(self, deviation):
        lib.setPositionOffset(self.id, deviation)

    def getPositionOffset(self):
        return lib.getPositionOffset(self.id)

    def setPositionLimits(self, minPos, maxPos):
        lib.setPositionLimits(self.id, minPos, maxPos)

    def getPositionLimits(self):
        data = lib.getPositionLimits(self.id)
        minPos = data >> 16
        maxPos = data & 0xffff
        return minPos, maxPos

    def savePositionOffset(self):
        lib.savePositionOffset(self.id)

    def setVoltageLimits(self, minVolt, maxVolt):
        lib.setVoltageLimits(self.id, minVolt, maxVolt)

    def getVoltageLimits(self):
        data = lib.getVoltageLimits(self.id)
        minVolt = data >> 16
        maxVolt = data & 0xffff
        return minVolt, maxVolt

    def setMaxTemp(self, temp):
        lib.setMaxTemp(self.id, temp)

    def getMaxTemp(self):
        return lib.getMaxTemp(self.id)

    def getTemp(self):
        return lib.getTemp(self.id)

    def getVoltage(self):
        return lib.getVoltage(self.id)

    def motorOn(self):
        lib.motorOn(self.id)

    def motorOff(self):
        lib.motorOff(self.id)

    def isMotorOn(self):
        return lib.isMotorOn(self.id)

    def LEDOn(self):
        lib.setLED(self.id,1)

    def LEDOff(self):
        lib.setLED(self.id,0)

    def isLEDOn(self):
        return lib.isLEDOn(self.id)

    def setLEDErrors(self, error):
        lib.setLEDErrors(self.id, error)

    def getLEDErrors(self):
        return lib.getLEDErrors(self.id)

    def setPositionMode(self):
        lib.setPositionMode(self.id)

    def getMode(self):
        return lib.getMode(self.id)

    def setSpeed(self, speed):
        lib.setSpeed(self.id, speed)

    def getSpeed(self):
        return lib.getSpeed(self.id)

    def getPos(self):
        return lib.getPos(self.id)

    #This function is not very reliable.
    def isMoving(self):
        return lib.isMoving(self.id, 2, 150000)
    
    def waitForMove(self):
        lib.waitForMove(self.id, 10)
                
if __name__ == '__main__':  
    
    print('start test!')

    if connect_servo_controller("/dev/ttyUSB0")<0:
        print("Can't connect to controller")
    else:
        
        print("Controller connected");
            
        m1 = Servo(1)

        m1.setSpeed(300)

        for i in range(10):
            pos = m1.getPos()
            print(m1.isMoving(), ' get run pos: ', pos)

            spd = m1.getSpeed()
            print(m1.isMoving(), ' get run spd: ', spd)

            vol = m1.getVoltage()
            print(m1.isMoving(), ' get vol: ', vol)

            temp = m1.getTemp()
            
            print(m1.isMoving(), ' get temp1: ', temp)
            
            time.sleep(0.5)
            

        m1.setPositionMode()
        m1.move(0)
        m1.waitForMove()
        print("reached ", m1.getPos())
        m1.move(500)
        m1.waitForMove()
        print("reached ", m1.getPos())
        m1.motorOff()
        
    print('End!')
