#! /usr/bin/env python3

from ev3dev2.motor import LargeMotor, OUTPUT_D, OUTPUT_C
from time import sleep
from ev3dev2.button import Button
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_4
from math import pi

K = 0.35
DIFF = 15
SPEED = 35
D = 5.55
B = 17.55
MAP = [[(2, 1), (3, 3), (2, 1), (1, 1), (1, 1), (4, 2), (1, 1), (2, 2)],
       [(4, 1), (1, 1), (5, 1), (3, 3), (7, 1), (4, 1), (3, 3), (1, 2)],
       [(4, 1), (3, 3), (1, 2), (7, 1), (7, 1), (4, 1), (1, 1), (4, 3)],
       [(2, 4), (1, 1), (4, 4), (1, 1), (1, 1), (2, 3), (3, 1), (2, 4)]]


class Robot:

    def __init__(self):
        self.snsr1 = ColorSensor(INPUT_1)
        self.snsr2 = ColorSensor(INPUT_4)
        self.snsr1.mode = 'COL-REFLECT'
        self.snsr2.mode = 'COL-REFLECT'
        self.mD = LargeMotor(OUTPUT_D)
        self.mC = LargeMotor(OUTPUT_C)
        self.btn = Button()
        self.mC.reset()
        self.mD.reset()

    def run_motors(self):
        s1, s4 = robot.snsr1.value(), robot.snsr2.value()
        error = s4 - s1 - DIFF
        u = error * K
        robot.mD.on(SPEED - u)
        robot.mC.on(SPEED + u)

    def fix_pos(self):
        s1, s4 = robot.snsr1.value(), robot.snsr2.value()
        error = s4 - s1 - DIFF
        while abs(error) > 5:
            s1, s4 = robot.snsr1.value(), robot.snsr2.value()
            error = s4 - s1 - DIFF
            u = error * K
            robot.mD.on(-u)
            robot.mC.on(u)
            sleep(0.05)
        robot.mD.stop()
        robot.mC.stop()
        sleep(0.1)

    def move_dist(self, l):
        c = (360 * l * 2) / (pi * D) + robot.mD.position + robot.mC.position
        motor1, motor4 = robot.mD.position, robot.mC.position
        while motor1 + motor4 < c:
            self.run_motors()
            motor1, motor4 = robot.mD.position, robot.mC.position
            sleep(0.01)
        robot.mD.stop()
        robot.mC.stop()

    def stop_near_crossroad(self):
        s1, s4 = robot.snsr1.value(), robot.snsr2.value()
        while s1 + s4 > 50:
            s1, s4 = robot.snsr1.value(), robot.snsr2.value()
            self.run_motors()
            sleep(0.01)
        self.move_dist(7.5)
        robot.mD.stop()
        robot.mC.stop()

    def get_type_of_cell(self):
        pass

    def turn_around(self, deg):
        l = []
        a = 2 * B * deg / D + robot.mD.position + robot.mC.position
        self.mD.on(30)
        self.mC.on(-30)
        while abs(robot.mD.position) + abs(robot.mC.position) < a:
            l.append(robot.snsr1.value())
            sleep(0.01)
        col, num, ch = [], 0, len(l) / 4
        while l:
            if l[0] < 20 and len(l) > 5 and l[5] < 20:
                col.append(round(num // ch + 1))
                while l[0] < 20 and l:
                    l.pop(0)
                    num += 1
            l.pop(0)
            num += 1
        robot.mD.stop()
        robot.mC.stop()
        if col[0] == 1:
            self.fix_pos()
        if len(col) == 1:
            field = (3, col[0])
        elif len(col) == 2:
            if col[0] == col[1] - 2:
                field = (1, col[0])
            elif col[0] == col[1] - 1:
                field = (2, col[0])
        elif len(col) == 3:
            print(col)
            if col[2] == col[1] + 1 and col[1] == col[0] + 1:
                field = (4, col[1])
            elif col[2] == col[1] + 2 and col[1] == col[0] + 1:
                field = (4, col[0])
            elif col[2] == col[1] + 1 and col[1] == col[0] + 2:
                field = (4, col[2])
        elif len(col) == 4:
            field = (5, 1)
        print(field)


robot = Robot()

robot.turn_around(360)
# robot.stop_near_crossroad()
# robot.get_type_of_cell()
# steer.on_for_degrees(steering=100, speed=20, degrees=1000)
