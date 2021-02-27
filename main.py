#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
peanut_motor = Motor(Port.D)
peanut_sensor = ColorSensor(Port.S1)

WHEEL_DIAMETER = 54
AXEL_TRACK = 200
robot = DriveBase(left_motor,right_motor,wheel_diameter=55.5,axle_track=104)
# Write your program here.

while True:
    if peanut_sensor.reflection()>6:
        robot.drive(20,0)
    else:
        robot.stop()
        robot.straight(10)
        peanut_motor.run_angle(10, 10,then=Stop.HOLD, wait=True)
        robot.straight(-80)
        peanut_motor.run_angle(-10,90 ,then=Stop.HOLD, wait=True)
        robot.straight(50)
        peanut_motor.run_angle(10,80 ,then=Stop.HOLD, wait=True)


"""
ev3.speaker.beep()

robot.straight(-1000)
ev3.speaker.beep()

robot.turn(360)
ev3.speaker.beep()

robot.turn(-360)
ev3.speaker.beep()
"""