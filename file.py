#! /usr/bin/env python3

from ev3dev2.motor import LargeMotor, OUTPUT_D, OUTPUT_C
from time import sleep
from ev3dev2.button import Button
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_4
from math import pi

K = 0.3
DIFF = 15
SPEED = 35
D = 5.55
B = 17.55
MAP = [[(2, 1), (3, 3), (2, 1), (1, 1), (1, 1), (4, 2), (1, 1), (2, 2)],
       [(4, 1), (1, 1), (5, 1), (3, 3), (7, 1), (4, 1), (3, 3), (1, 2)],
       [(4, 1), (3, 3), (1, 2), (7, 1), (7, 1), (4, 1), (1, 1), (4, 3)],
       [(2, 4), (1, 1), (4, 4), (1, 1), (1, 1), (2, 3), (3, 1), (2, 3)]]


class Robot:

    def __init__(self):
        self.snsr1 = ColorSensor(INPUT_1)
        self.snsr2 = ColorSensor(INPUT_4)
        self.snsr1.mode = 'COL-REFLECT'
        self.snsr2.mode = 'COL-REFLECT'
        self.mD = LargeMotor(OUTPUT_D)
        self.mC = LargeMotor(OUTPUT_C)
        self.btn = Button()
        self.way = 1
        self.mC.reset()
        self.mD.reset()

    def run_motors(self):
        s1, s4 = self.snsr1.value(), self.snsr2.value()
        error = s4 - s1 - DIFF
        u = error * K
        self.mD.on(SPEED - u)
        self.mC.on(SPEED + u)

    def fix_pos(self):
        s1, s4 = self.snsr1.value(), self.snsr2.value()
        error = s4 - s1 - DIFF
        while abs(error) > 5:
            s1, s4 = self.snsr1.value(), self.snsr2.value()
            error = s4 - s1 - DIFF
            u = error * K
            self.mD.on(-u)
            self.mC.on(u)
            sleep(0.05)
        self.mD.stop()
        self.mC.stop()
        sleep(0.1)

    def move_dist(self, l):
        c = (360 * l * 2) / (pi * D) + self.mD.position + self.mC.position
        motor1, motor4 = self.mD.position, self.mC.position
        while motor1 + motor4 < c:
            self.run_motors()
            motor1, motor4 = self.mD.position, self.mC.position
            sleep(0.01)
        self.mD.stop()
        self.mC.stop()

    def stop_near_crossroad(self):
        s1, s4 = self.snsr1.value(), self.snsr2.value()
        while s1 + s4 > 50:
            s1, s4 = self.snsr1.value(), self.snsr2.value()
            self.run_motors()
            sleep(0.01)
        self.move_dist(7.5)
        self.mD.stop()
        self.mC.stop()

    def get_type_of_cell(self):
        l = []
        self.mC.reset()
        self.mD.reset()
        a = 2 * B * 360 / D + self.mD.position + self.mC.position
        self.mD.on(30)
        self.mC.on(-30)
        while abs(self.mD.position) + abs(self.mC.position) < a:
            l.append(self.snsr1.value())
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
        self.mD.stop()
        self.mC.stop()
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
            if col[2] == col[1] + 1 and col[1] == col[0] + 1:
                field = (4, col[1])
            elif col[2] == col[1] + 2 and col[1] == col[0] + 1:
                field = (4, col[0])
            elif col[2] == col[1] + 1 and col[1] == col[0] + 2:
                field = (4, col[2])
        elif len(col) == 4:
            field = (5, 1)
        return field

    def turn_around(self, deg):
        self.mC.reset()
        self.mD.reset()
        a = 2 * B * deg / D + self.mD.position + self.mC.position
        self.mD.on(30)
        self.mC.on(-30)
        while abs(self.mD.position) + abs(self.mC.position) < a:
            sleep(0.01)

    def localization(self):
        pos, variants = {}, []
        self.move_x = 0
        self.move_y = 0
        pos1 = self.get_type_of_cell()
        pos[(self.move_x, self.move_y)] = pos1
        for i in range(4):
            for j in range(8):
                if MAP[i][j] == pos[(self.move_x, self.move_y)]:
                    variants.append((j,  i))
        while len(variants) > 1:
            variants = []
            self.way = pos1[1]
            self.turn_around(90 * (self.way - 1))
            self.move_dist(30)
            pos1 = self.get_type_of_cell()
            if self.way % 2 == 0:
                self.move_y = self.move_y + 3 - self.way
            elif self.way % 2 == 1:
                self.move_x = self.move_x + 2 - self.way
            pos[(self.move_x, self.move_y)] = pos1
            for i in range(4):
                for j in range(8):
                    f = True
                    for k in list(pos.keys()):
                        if 0 <= i + k[1] <= 3 and 0 <= j + k[0] <= 7:
                            if MAP[i + k[1]][j + k[0]] != pos[k]:
                                f = False
                    if f:
                        variants.append((j, i))
        self.move_dist(1.5)
        self.start_pos = variants[0]
        self.current_pos = (variants[0][0] + self.move_x, variants[0][1] + self.move_y)
        print(self.start_pos, self.current_pos)

    def way(self, start, end):
        move = []
        if MAP[start[1]][start[0]][0] == 1:
            if MAP[start[1]][start[0]][1] == 1:
                move = [[start, (start[0] - 1, start[1])], [start, (start[0] + 1, start[1])]]
            else:
                move = [[start, (start[0], start[1] - 1)], [start, (start[0], start[1] + 1)]]
        elif MAP[start[1]][start[0]][0] == 2:
            if MAP[start[1]][start[0]][1] == 1:
                move = [[start, (start[0] + 1, start[1])], [start, (start[0], start[1] + 1)]]
            elif MAP[start[1]][start[0]][1] == 2:
                move = [[start, (start[0] - 1, start[1])], [start, (start[0], start[1] + 1)]]
            elif MAP[start[1]][start[0]][1] == 3:
                move = [[start, (start[0] - 1, start[1])], [start, (start[0], start[1] - 1)]]
            else:
                move = [[start, (start[0] + 1, start[1])], [start, (start[0], start[1] - 1)]]
        elif MAP[start[1]][start[0]][0] == 3:
            if MAP[start[1]][start[0]][1] == 1:
                move = [[start, (start[0] + 1, start[1])]]
            elif MAP[start[1]][start[0]][1] == 2:
                move = [[start, (start[0], start[1] + 1)]]
            elif MAP[start[1]][start[0]][1] == 3:
                move = [[start, (start[0] - 1, start[1])]]
            else:
                move = [[start, (start[0], start[1] - 1)]]
        elif MAP[start[1]][start[0]][0] == 4:
            if MAP[start[1]][start[0]][1] == 1:
                move = [[start, (start[0], start[1] - 1)],
                        [start, (start[0] + 1, start[1])],
                        [start, (start[0], start[1] + 1)]]
            elif MAP[start[1]][start[0]][1] == 2:
                move = [[start, (start[0] + 1, start[1])],
                        [start, (start[0], start[1] + 1)],
                        [start, (start[0] - 1, start[1])]]
            elif MAP[start[1]][start[0]][1] == 3:
                move = [[start, (start[0], start[1] + 1)],
                        [start, (start[0] - 1, start[1])],
                        [start, (start[0], start[1] - 1)]]
            else:
                move = [[start, (start[0] - 1, start[1])],
                        [start, (start[0], start[1] - 1)],
                        [start, (start[0] + 1, start[1])]]
        elif MAP[start[1]][start[0]][0] == 5:
            move = [[start, (start[0], start[1] - 1)],
                    [start, (start[0] + 1, start[1])],
                    [start, (start[0], start[1] + 1)],
                    [start, (start[0] - 1, start[1])]]
        while all(x[-1] != end for x in move):
            new_move = []
            for i in range(len(move)):
                if MAP[move[i][-1][1]][move[i][-1][0]][0] == 1:
                    if MAP[move[i][-1][1]][move[i][-1][0]][1] == 1:
                        new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                        new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                    else:
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
                elif MAP[move[i][-1][1]][move[i][-1][0]][0] == 2 or MAP[move[i][-1][1]][move[i][-1][0]][0] == 6:
                    if MAP[move[i][-1][1]][move[i][-1][0]][1] == 1:
                        new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                    elif MAP[move[i][-1][1]][move[i][-1][0]][1] == 2:
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                        new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                    elif MAP[move[i][-1][1]][move[i][-1][0]][1] == 3:
                        new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
                    else:
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
                        new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                elif MAP[move[i][-1][1]][move[i][-1][0]][0] == 3:
                    if MAP[move[i][-1][1]][move[i][-1][0]][1] == 1:
                        new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                    elif MAP[move[i][-1][1]][move[i][-1][0]][1] == 2:
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                    elif MAP[move[i][-1][1]][move[i][-1][0]][1] == 3:
                        new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                    else:
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
                elif MAP[move[i][-1][1]][move[i][-1][0]][0] == 4:
                    if MAP[move[i][-1][1]][move[i][-1][0]][1] == 1:
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
                        new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                    elif MAP[move[i][-1][1]][move[i][-1][0]][1] == 2:
                        new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                        new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                    elif MAP[move[i][-1][1]][move[i][-1][0]][1] == 3:
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                        new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
                    else:
                        new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                        new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
                        new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                elif MAP[move[i][-1][1]][move[i][-1][0]][0] == 5:
                    new_move.append(move[i] + [(move[i][-1][0] - 1, move[i][-1][1])])
                    new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] + 1)])
                    new_move.append(move[i] + [(move[i][-1][0] + 1, move[i][-1][1])])
                    new_move.append(move[i] + [(move[i][-1][0], move[i][-1][1] - 1)])
            i = 0
            while i < len(new_move):
                if new_move[i][-1] == new_move[i][-3]:
                    del new_move[i]
                else:
                    i += 1
            move = new_move[:]
        var = []
        for i in range(len(move)):
            if move[i][-1] == end:
                var.append(move[i])
        if len(var) > 1:
            i = 0
            while i < len(var):
                f = False
                for j in var[i]:
                    if MAP[j[1]][j[0]][0] == 6:
                        f = True
                if f:
                    del var[i]
                else:
                    i += 1
        if all(len(x) == len(var[0]) for x in var):
            print(var[0])
        else:
            l = [len(x) for x in var]
            print(var[l.index(min(l))])




robot = Robot()

robot.way((2, 1), (0, 0))
# robot.stop_near_crossroad()
# robot.get_type_of_cell()
