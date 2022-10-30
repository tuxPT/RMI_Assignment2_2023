
import sys
from croblink import *
from math import *
import math
import xml.etree.ElementTree as ET
from tree_search import *

CELLROWS=7
CELLCOLS=14


class PIDController():
    def __init__(self, kp, ki, kd, sample, kp_rot, ki_rot, kd_rot):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sample = 0.050
        self.error_1 = 0
        self.error_2 = 0
        self.out_1 = 0
        self.td = 0.0
        self.delta = 0.05
        self.max_out = 0.15
        self.error_1_rot = 0
        self.error_2_rot = 0
        self.out_1_rot = 0
        self.kp_rot = kp_rot
        self.ki_rot = ki_rot
        self.kd_rot = kd_rot
        self.k0=self.kp*(1 + self.sample/self.ki + self.kd/self.sample)
        self.k1=-self.kp*(1 + 2*self.kd/self.sample)               
        self.k2=self.kp*self.kd/self.sample
        self.k0_rot=self.kp_rot*(1 + self.sample/self.ki_rot + self.kd_rot/self.sample)
        self.k1_rot=-self.kp_rot*(1 + 2*self.kd_rot/self.sample)               
        self.k2_rot=self.kp_rot*self.kd_rot/self.sample


    def go(self, set_vel, curr_vel, rot=False): 
        if rot:
            error = set_vel - curr_vel
            out = self.out_1_rot + self.k0_rot*error + self.k1_rot*self.error_1_rot + self.k2_rot*self.error_2_rot
            
            self.out_1_rot = out
            self.error_2_rot = self.error_1_rot
            self.error_1_rot = error
        else:
            error = set_vel - curr_vel
            out = self.out_1 + self.k0*error + self.k1*self.error_1 + self.k2*self.error_2
            
            self.out_1 = out
            self.error_2 = self.error_1
            self.error_1 = error

        if self.out_1>self.max_out:
            self.out_1 = self.max_out
        if self.out_1<-self.max_out:
            self.out_1 = -self.max_out
        return out
class Domain(SearchDomain):

    def __init__(self, connections):
        self.connections = connections # all knwon cells

    def actions (self, cell):
        actlist = []
        for (c1, c2) in self.connections:
            if (c1 == cell):
                actlist = actlist + [(c1, c2)]    
            elif (c2 == cell): 
                actlist = actlist + [(c2, c1)]
        return   actlist

    def result(self, cell, action):
        #return
        (c1, c2) = action
        if c1 == cell:
             return c2

    def cost (self, pos, action):
        return 1

    def heuristic(self, pos, goal):
        return math.hypot(pos[0]-goal[0], pos[1]-goal[1])

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, challenge, outfile):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.challenge = challenge
        self.outfile = outfile
        self.pid_Controller = PIDController(0.07, 1000, 0.000001, 0.025, 0.3, math.inf, 0.000001)
        if self.challenge == "1":
            self.track = []
        if self.challenge == "2" or self.challenge == "3":
            self.map = [[" " for j in range(1,50)] for i in range(1,22)]
            self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
            self.path = []
            #print(len(self.map)*len(self.map[0]))
            self.map[10][24] = "I"
            #print(self.map)
            self.unexploredpaths = []
            self.connections = []
            self.neighborhood = []
            self.pathtoexplore = []
            self.exploredpath = []
            self.has_path = 'random'
            self.rotation = None
            self.dest = None
            self.beacons = []


            

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        self.readSensors()
        self.diffpos = [24 - self.measures.x, 10 - self.measures.y]

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            

    def wander(self):
        """ center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1) """

        if self.challenge == "1":
            self.wanderC1()
        elif self.challenge == "2":
            self.wanderC2()
        elif self.challenge == "3":
            self.wanderC3()

    def go(self,lin,k,m,r):
        rot = k*(m - r)
        lpow = lin + (rot/2)
        rpow = lin - (rot/2)
        return [lpow, rpow]

    def wanderC1(self):

        print(f'{"Line sensor: "}{self.measures.lineSensor}')

        """ # Rotate left
        if self.measures.lineSensor[0] == '1':
            #self.driveMotors(-0.12,0.15)
            self.driveMotors(-0.08,0.1)
            self.track.append("left")

        # Rotate right
        elif self.measures.lineSensor[6] == '1':
            #self.driveMotors(0.15,-0.12)
            self.driveMotors(0.1,-0.08)
            self.track.append("right")   """
        
        if self.measures.lineSensor[0] == '1':
            #pow = self.pid_Controller.go(3/7, self.measures.lineSensor.count("1")/7)
            self.driveMotors(-0.12, 0.15)

        elif self.measures.lineSensor[6] == '1':
            #pow = self.pid_Controller.go(3/7, self.measures.lineSensor.count("1")/7)
            self.driveMotors(0.15, -0.12)
        else:
            pow = 0.15 - self.pid_Controller.go(3/7, self.measures.lineSensor.count("1")/7)
            lpow, rpow = self.go(pow, 0.1, self.measures.lineSensor.count('1'), 3)
            self.driveMotors(lpow, rpow)


        """ # Adjust left
        elif self.measures.lineSensor[1] == '1':
            #self.driveMotors(0.08,0.15)
            self.driveMotors(0.08,0.1)

        # Adjust right
        elif self.measures.lineSensor[5] == '1':
            #self.driveMotors(0.15,0.08)
            self.driveMotors(0.1,0.08)

        # Forward
        elif '1' in self.measures.lineSensor[2:5]:
            #self.driveMotors(0.15, 0.15)
            self.driveMotors(0.1, 0.1)       """



    def wanderC2(self):
        
        x,y = self.get_correct_measures()

        print(f'{"Pos: "}{x},{y}')
        x = round(x)
        y = round(y)

        print(f'{"Compass: "}{self.measures.compass}')
        print(f'{"Line sensor: "}{self.measures.lineSensor}')

        # every odd line
        if y % 2 == 1:
            self.map[y][x] = "|"
        # every odd column and even linedist
        # every odd column and even line
        elif x % 2 == 1:
            self.map[y][x] = "-"
        else:
            pass
        if self.has_path == 'path_finding':
            x,y = self.get_correct_measures()
            #if abs(round(x) - x) < 0.05:
              #  x = round(x)
            #else:
            
            #if abs(round(y) - y) < 0.05:
               # y = round(y)
            #if:
            
            print('x, y = ' + str(x) + ', ' + str(y))
            if [x, y] in self.unexploredpaths:
                self.unexploredpaths.remove([x, y])
            # path exists ?
            if len(self.path):
                print('HAS PATH')
                if (self.dest[0] == 'x' and self.dest[1] == x) or (self.dest[0] == 'y' and self.dest[1] == y):
                    print('cell in path')
                    self.path = self.path[1:]
                    if len(self.path) == 0:
                        print('GET PATH')
                        # go for unexplored paths
                        unexploredcell = self.find_next(x, y)
                        self.path = self.get_best_path([2*round(x/2),2*round(y/2)], unexploredcell)
                        self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
                        for cell in self.path:
                            self.path_map[cell[1]][cell[0]] = 'P'
                        self.save_map(self.path_map, "path.txt")
                    horizontal = self.path[0][0] - round(x)
                    vertical = self.path[0][1] - round(y)
                    if horizontal:
                        self.dest = ('x', self.path[0][0])
                    elif vertical:
                        self.dest = ('y', self.path[0][1])
                    if horizontal > 0:
                        self.rotation = 0
                    elif horizontal < 0:
                        self.rotation = -180
                    elif vertical > 0:
                        self.rotation = 90
                    elif vertical < 0:
                        self.rotation = -90
            # get path
            else:
                print('GET PATH')
                # go for unexplored paths
                unexploredcell = self.find_next(x, y)
                self.path = self.get_best_path([2*round(x/2),2*round(y/2)], unexploredcell)
                self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
                for cell in self.path:
                    self.path_map[cell[1]][cell[0]] = 'P'
                self.save_map(self.path_map, "path.txt")
                horizontal = self.path[0][0] - round(x)
                vertical = self.path[0][1] - round(y)
                if horizontal:
                    self.dest = ('x', self.path[0][0])
                elif vertical:
                    self.dest = ('y', self.path[0][1])
                if horizontal > 0:
                    self.rotation = 0
                elif horizontal < 0:
                    self.rotation = -180
                elif vertical > 0:
                    self.rotation = 90
                elif vertical < 0:
                    self.rotation = -90
           
            
                
            # foward
            if ((self.dest[0] == 'x' and self.dest[1] != x) or (self.dest[0] == 'y' and self.dest[1] != y)) and abs(self.rotation-self.measures.compass) <= 1:
                #self.rotation = None
                # if there is a path both to the right and to the left
                

                """
                print('FOWARD')
                # if the agent is swinging left   
                if self.measures.lineSensor[1] == '1':
                    print('Adjust left')
                    self.driveMotors(0.01,0.05)

                # if the agent is swinging right   
                elif self.measures.lineSensor[5] == '1':
                    print('Adjust right')
                    self.driveMotors(0.05,0.01) """

                # if there is a path forward
                #if '1' in self.measures.lineSensor[2:5]:
                print('Forward')
                if self.dest[0] == 'x':
                    if self.rotation == -180:
                        pow = self.pid_Controller.go(x, self.dest[1])
                    else:
                        pow = self.pid_Controller.go(self.dest[1], x)
                elif self.dest[0] == 'y':
                    if self.rotation == -90:
                        pow = self.pid_Controller.go(y, self.dest[1])
                    else:
                        pow = self.pid_Controller.go(self.dest[1], y)
                lpow, rpow = self.go(pow, 0.1, self.measures.compass/180, self.rotation/180)
                self.driveMotors(lpow, rpow)
                #self.unknown_pos(x, y, 'front')
                
                # if it is a dead end
                #elif self.measures.lineSensor.count('0') == 7:
                #    #self.driveMotors(0,0)
                #    self.driveMotors(0.05,-0.05)
            # rotate
            else:
                print('ROTATE')
                #pow = self.pid_Controller.go(self.rotation/180, self.measures.compass/180, True)
                pow = self.pid_Controller.go(self.rotation/180, self.measures.compass/180, True)
                self.driveMotors(-pow, pow)
            

            x = round(x)
            y = round(y)
            if self.measures.lineSensor.count('1') == 7:
                    self.unknown_pos(x, y, 'left')
                    self.unknown_pos(x, y, 'right')

            # if there is a path to the left
            elif self.measures.lineSensor[0] == '1':
                if self.measures.lineSensor.count('1') > 3:
                    self.unknown_pos(x, y, 'left')
            
            # if there is a path to the left
            elif self.measures.lineSensor[6] == '1':
                if self.measures.lineSensor.count('1') > 3:
                    self.unknown_pos(x, y, 'right')
            #if '1' in self.measures.lineSensor[2:5]:
            #    self.unknown_pos(x, y, 'front')
            
            print('rotation = ' + str(self.rotation))
            print('PATH = ' + str(self.path))
            #print('pow = ' + str(pow))
            #print('lpow, rpow = ' + str(lpow) + ', ' + str(rpow))
            #print('curr = ' + str(curr) + ', dest = ' + str(self.dest[1]) + ', dest_dir = ' + str(self.dest[0]))


            """ if self.dest_cell == None:
                print('DEST_CELL==None')
                self.dest_cell = self.path[0]
                print('DEST_CELL = ' + str(self.dest_cell))
                self.path = self.path[1:]
            if self.rotation and abs(self.rotation - self.measures.compass) > 10:
                print('ROTATE')
                lpow, rpow = self.go(0.0, 0.1, self.rotation, self.measures.compass)
                self.driveMotors(-lpow, -rpow)
            elif [x, y] == self.dest_cell:
                print('DEST_CELL==[x,y]')
                if len(self.path):
                    self.dest_cell = self.path[0]
                    print('DEST_CELL = ' + str(self.dest_cell))
                    self.path = self.path[1:]
                else:
                    # go for unexplored paths
                    unexploredcell = self.find_next(x, y)
                    self.path = self.get_best_path([2*round(x/2),2*round(y/2)], unexploredcell)
                    self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
                    for cell in self.path:
                        self.path_map[cell[1]][cell[0]] = 'P'
                    self.save_map(self.path_map, "path.txt")
                horizontal = self.dest_cell[0] - x
                vertical = self.dest_cell[1] - y
                if horizontal > 0:
                    self.rotation = 0
                elif horizontal < 0:
                    self.rotation = -180
                elif vertical > 0:
                    self.rotation = 90
                elif vertical < 0:
                    self.rotation = -90
                else:
                    print('ACABOU')
                print('rotation' + str(self.rotation))
            else:
                print('ELSE FORWARD')
                self.rotation = None
                # if the agent is swinging left   
                if self.measures.lineSensor[1] == '1':
                    print('Adjust left')
                    self.driveMotors(0.08,0.15)

                # if the agent is swinging right   
                elif self.measures.lineSensor[5] == '1':
                    print('Adjust right')
                    self.driveMotors(0.15,0.08)

                # if there is a path forward
                elif '1' in self.measures.lineSensor[2:5]:
                    print('Forward')
                    self.driveMotors(0.15, 0.15)
                    self.unknown_pos(x, y, 'front')
                
                # if it is a dead end
                elif self.measures.lineSensor.count('0') == 7:
                    self.driveMotors(0,0)
                    self.driveMotors(0.15,-0.15)
            print('PATH = ' + str(self.path)) """
                
        else:
            # if the agent makes a complete lap (for search testing purposes)
            if [x,y] == [24, 9]:
                self.driveMotors(0,0)
                # go for unexplored paths
                unexploredcell = self.find_next(x, y)
                print(f'{"UNEXPLROED: "}{unexploredcell}')
                self.path = self.get_best_path([2*round(x/2), 2*round(y/2)], unexploredcell)
                horizontal = self.path[0][0] - round(x)
                vertical = self.path[0][1] - round(y)
                if horizontal:
                    self.dest = ('x', self.path[0][0])
                elif vertical:
                    self.dest = ('y', self.path[0][1])
                if horizontal > 0:
                    self.rotation = 0
                elif horizontal < 0:
                    self.rotation = -180
                elif vertical > 0:
                    self.rotation = 90
                elif vertical < 0:
                    self.rotation = -90
                print(f'{"PATH: "}{self.path}')
                for cell in self.path:
                    self.path_map[cell[1]][cell[0]] = 'P'
                
                self.save_map(self.path_map, "path.txt")

                self.has_path = 'path_finding'

            else:
                # if there is a path both to the right and to the left
                if self.measures.lineSensor.count('1') == 7:
                    self.unknown_pos(x, y, 'left')
                    self.unknown_pos(x, y, 'right')

                # if there is a path to the left
                if self.measures.lineSensor[0] == '1':
                    if self.measures.lineSensor.count('1') > 3:
                        self.unknown_pos(x, y, 'left')

                    print('Rotate left')
                    self.driveMotors(-0.12,0.15)
                
                # if there is a path to the left
                elif self.measures.lineSensor[6] == '1':
                    if self.measures.lineSensor.count('1') > 3:
                        self.unknown_pos(x, y, 'right')
                    
                    print('Rotate right')
                    self.driveMotors(0.15,-0.12)

                # if the agent is swinging left   
                elif self.measures.lineSensor[1] == '1':
                    print('Adjust left')
                    self.driveMotors(0.08,0.15)

                # if the agent is swinging right   
                elif self.measures.lineSensor[5] == '1':
                    print('Adjust right')
                    self.driveMotors(0.15,0.08)

                # if there is a path forward
                elif '1' in self.measures.lineSensor[2:5]:
                    print('Forward')
                    self.driveMotors(0.15, 0.15)
                    self.unknown_pos(x, y, 'front')
                
                # if it is a dead end
                elif self.measures.lineSensor.count('0') == 7:
                    self.driveMotors(0,0)
                    self.driveMotors(0.15,-0.15)

        self.save_map()

    def wanderC3(self):
        x,y = self.get_correct_measures()

        print(f'{"Pos: "}{x},{y}')
        x = round(x)
        y = round(y)

        print(f'{"Compass: "}{self.measures.compass}')
        print(f'{"Line sensor: "}{self.measures.lineSensor}')

        if self.measures.ground != -1:
            if [x, y] not in self.beacons:
                self.beacons.append([2*round(x/2), 2*round(y/2)])

        if self.has_path == 'path_finding':
            x,y = self.get_correct_measures()
            #if abs(round(x) - x) < 0.05:
              #  x = round(x)
            #else:
            
            #if abs(round(y) - y) < 0.05:
               # y = round(y)
            #if:
            
            print('x, y = ' + str(x) + ', ' + str(y))
            if [x, y] in self.unexploredpaths:
                self.unexploredpaths.remove([x, y])
            # path exists ?
            if len(self.path):
                print('HAS PATH')
                if (self.dest[0] == 'x' and self.dest[1] == x) or (self.dest[0] == 'y' and self.dest[1] == y):
                    print('cell in path')
                    self.path = self.path[1:]
                    if len(self.path) == 0:
                        print('GET PATH')
                        # go for unexplored paths
                        unexploredcell = self.find_next(x, y)
                        self.path = self.get_best_path([2*round(x/2),2*round(y/2)], unexploredcell)
                        self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
                        for cell in self.path:
                            self.path_map[cell[1]][cell[0]] = 'P'
                    horizontal = self.path[0][0] - round(x)
                    vertical = self.path[0][1] - round(y)
                    if horizontal:
                        self.dest = ('x', self.path[0][0])
                    elif vertical:
                        self.dest = ('y', self.path[0][1])
                    if horizontal > 0:
                        self.rotation = 0
                    elif horizontal < 0:
                        self.rotation = -180
                    elif vertical > 0:
                        self.rotation = 90
                    elif vertical < 0:
                        self.rotation = -90
            # get path
            else:
                print('GET PATH')
                # go for unexplored paths
                unexploredcell = self.find_next(x, y)
                self.path = self.get_best_path([2*round(x/2),2*round(y/2)], unexploredcell)
                self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
                for cell in self.path:
                    self.path_map[cell[1]][cell[0]] = 'P'
                horizontal = self.path[0][0] - round(x)
                vertical = self.path[0][1] - round(y)
                if horizontal:
                    self.dest = ('x', self.path[0][0])
                elif vertical:
                    self.dest = ('y', self.path[0][1])
                if horizontal > 0:
                    self.rotation = 0
                elif horizontal < 0:
                    self.rotation = -180
                elif vertical > 0:
                    self.rotation = 90
                elif vertical < 0:
                    self.rotation = -90
           
            
                
            # foward
            if ((self.dest[0] == 'x' and self.dest[1] != x) or (self.dest[0] == 'y' and self.dest[1] != y)) and abs(self.rotation-self.measures.compass) <= 1:
                #self.rotation = None
                # if there is a path both to the right and to the left
                

                """
                print('FOWARD')
                # if the agent is swinging left   
                if self.measures.lineSensor[1] == '1':
                    print('Adjust left')
                    self.driveMotors(0.01,0.05)

                # if the agent is swinging right   
                elif self.measures.lineSensor[5] == '1':
                    print('Adjust right')
                    self.driveMotors(0.05,0.01) """

                # if there is a path forward
                #if '1' in self.measures.lineSensor[2:5]:
                print('Forward')
                if self.dest[0] == 'x':
                    if self.rotation == -180:
                        pow = self.pid_Controller.go(x, self.dest[1])
                    else:
                        pow = self.pid_Controller.go(self.dest[1], x)
                elif self.dest[0] == 'y':
                    if self.rotation == -90:
                        pow = self.pid_Controller.go(y, self.dest[1])
                    else:
                        pow = self.pid_Controller.go(self.dest[1], y)
                lpow, rpow = self.go(pow, 0.1, self.measures.compass/180, self.rotation/180)
                self.driveMotors(lpow, rpow)
                #self.unknown_pos(x, y, 'front')
                
                # if it is a dead end
                #elif self.measures.lineSensor.count('0') == 7:
                #    #self.driveMotors(0,0)
                #    self.driveMotors(0.05,-0.05)
            # rotate
            else:
                print('ROTATE')
                #pow = self.pid_Controller.go(self.rotation/180, self.measures.compass/180, True)
                pow = self.pid_Controller.go(self.rotation/180, self.measures.compass/180, True)
                self.driveMotors(-pow, pow)
            

            x = round(x)
            y = round(y)
            if self.measures.lineSensor.count('1') == 7:
                    self.unknown_pos(x, y, 'left')
                    self.unknown_pos(x, y, 'right')

            # if there is a path to the left
            elif self.measures.lineSensor[0] == '1':
                if self.measures.lineSensor.count('1') > 3:
                    self.unknown_pos(x, y, 'left')
            
            # if there is a path to the left
            elif self.measures.lineSensor[6] == '1':
                if self.measures.lineSensor.count('1') > 3:
                    self.unknown_pos(x, y, 'right')
            #if '1' in self.measures.lineSensor[2:5]:
            #    self.unknown_pos(x, y, 'front')
            
            print('rotation = ' + str(self.rotation))
            print('PATH = ' + str(self.path))
            #print('pow = ' + str(pow))
            #print('lpow, rpow = ' + str(lpow) + ', ' + str(rpow))
            #print('curr = ' + str(curr) + ', dest = ' + str(self.dest[1]) + ', dest_dir = ' + str(self.dest[0]))


            """ if self.dest_cell == None:
                print('DEST_CELL==None')
                self.dest_cell = self.path[0]
                print('DEST_CELL = ' + str(self.dest_cell))
                self.path = self.path[1:]
            if self.rotation and abs(self.rotation - self.measures.compass) > 10:
                print('ROTATE')
                lpow, rpow = self.go(0.0, 0.1, self.rotation, self.measures.compass)
                self.driveMotors(-lpow, -rpow)
            elif [x, y] == self.dest_cell:
                print('DEST_CELL==[x,y]')
                if len(self.path):
                    self.dest_cell = self.path[0]
                    print('DEST_CELL = ' + str(self.dest_cell))
                    self.path = self.path[1:]
                else:
                    # go for unexplored paths
                    unexploredcell = self.find_next(x, y)
                    self.path = self.get_best_path([2*round(x/2),2*round(y/2)], unexploredcell)
                    self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
                    for cell in self.path:
                        self.path_map[cell[1]][cell[0]] = 'P'
                    self.save_map(self.path_map, "path.txt")
                horizontal = self.dest_cell[0] - x
                vertical = self.dest_cell[1] - y
                if horizontal > 0:
                    self.rotation = 0
                elif horizontal < 0:
                    self.rotation = -180
                elif vertical > 0:
                    self.rotation = 90
                elif vertical < 0:
                    self.rotation = -90
                else:
                    print('ACABOU')
                print('rotation' + str(self.rotation))
            else:
                print('ELSE FORWARD')
                self.rotation = None
                # if the agent is swinging left   
                if self.measures.lineSensor[1] == '1':
                    print('Adjust left')
                    self.driveMotors(0.08,0.15)

                # if the agent is swinging right   
                elif self.measures.lineSensor[5] == '1':
                    print('Adjust right')
                    self.driveMotors(0.15,0.08)

                # if there is a path forward
                elif '1' in self.measures.lineSensor[2:5]:
                    print('Forward')
                    self.driveMotors(0.15, 0.15)
                    self.unknown_pos(x, y, 'front')
                
                # if it is a dead end
                elif self.measures.lineSensor.count('0') == 7:
                    self.driveMotors(0,0)
                    self.driveMotors(0.15,-0.15)
            print('PATH = ' + str(self.path)) """
                
        else:
            # if the agent makes a complete lap (for search testing purposes)
            if [x,y] == [24, 9]:
                self.driveMotors(0,0)
                self.beacons_path([2*round(x/2), 2*round(y/2)])
                # go for unexplored paths
                unexploredcell = self.find_next(x, y)
                print(f'{"UNEXPLROED: "}{unexploredcell}')
                self.path = self.get_best_path([2*round(x/2), 2*round(y/2)], unexploredcell)
                horizontal = self.path[0][0] - round(x)
                vertical = self.path[0][1] - round(y)
                if horizontal:
                    self.dest = ('x', self.path[0][0])
                elif vertical:
                    self.dest = ('y', self.path[0][1])
                if horizontal > 0:
                    self.rotation = 0
                elif horizontal < 0:
                    self.rotation = -180
                elif vertical > 0:
                    self.rotation = 90
                elif vertical < 0:
                    self.rotation = -90
                print(f'{"PATH: "}{self.path}')
                for cell in self.path:
                    self.path_map[cell[1]][cell[0]] = 'P'
                

                self.has_path = 'path_finding'

            else:
                # if there is a path both to the right and to the left
                if self.measures.lineSensor.count('1') == 7:
                    self.unknown_pos(x, y, 'left')
                    self.unknown_pos(x, y, 'right')
                    self.driveMotors(0.1,-0.08)
                # if there is a path to the left
                elif self.measures.lineSensor[0] == '1':
                    if self.measures.lineSensor.count('1') > 3:
                        self.unknown_pos(x, y, 'left')

                    print('Rotate left')
                    self.driveMotors(-0.08,0.1)
                
                # if there is a path to the left
                elif self.measures.lineSensor[6] == '1':
                    if self.measures.lineSensor.count('1') > 3:
                        self.unknown_pos(x, y, 'right')
                    
                    print('Rotate right')
                    self.driveMotors(0.1,-0.08)

                # if the agent is swinging left   
                elif self.measures.lineSensor[1] == '1':
                    print('Adjust left')
                    self.driveMotors(0.05,0.1)

                # if the agent is swinging right   
                elif self.measures.lineSensor[5] == '1':
                    print('Adjust right')
                    self.driveMotors(0.1,0.05)

                # if there is a path forward
                elif '1' in self.measures.lineSensor[2:5]:
                    print('Forward')
                    self.driveMotors(0.1, 0.1)
                    self.unknown_pos(x, y, 'front')
                
                # if it is a dead end
                elif self.measures.lineSensor.count('0') == 7:
                    self.driveMotors(0,0)
                    self.driveMotors(0.15,-0.15)

    # returns the closest non explored point to the current position
    def find_next(self, x, y):
        # pesquisa no mapa em vez da lista
        # unexplored = [(row.index('?'), index) for index, row in enumerate(reversed(self.map)) if '?' in row]
        # min(unexplored, key=lambda spot: math.dist(spot, (x, y)))
        minimum = 100000
        target = []
        for point in self.unexploredpaths: 
            if point != [x,y]:
                dist = math.dist([x,y], point)
                if dist < minimum:
                    minimum = dist
                    target = point

        return target

    # based on tree_search module and list of connections, returns the best path from origin to destination
    def get_best_path(self, origin, destination):
        self.pathtoexplore = []
        print("connections: " + str(sorted(self.connections)))
        print("origin: " + str(origin))
        print("destination: " + str(destination))

        self.pathtoexplore += SearchTree(SearchProblem(Domain(self.connections), origin, destination), 'a*').search(2000)[0]

        return self.pathtoexplore

    def unknown_pos(self, x, y, direction):
        calc = [0, 0]
        self.neighborhood = []
        # up
        if 80 < self.measures.compass < 100:
            if direction == 'left':
                calc = [-1, 0]
            elif direction == 'front':
                calc = [0, 1]
            else:
                calc = [1, 0]
        # right
        elif -10 < self.measures.compass < 10:
            if direction == 'left':
                calc = [0, 1]
            elif direction == 'front':
                calc = [1, 0]
            else:
                calc = [0, -1]
        # left
        elif -170 < self.measures.compass < -180 or 170 < self.measures.compass < 180:
            if direction == 'left':
                calc = [0, -1]
            elif direction == 'front':
                calc = [-1, 0]
            else:
                calc = [0, 1]
        # down
        elif -100 < self.measures.compass < -80:
            if direction == 'left':
                calc = [1, 0]
            elif direction == 'front':
                calc = [0, -1]
            else:
                calc = [-1, 0]


        # calculate neighbor cells and add them to a list of connections between cells
        # add the current cell to a list of explored cells
        current_cell = [x, y] 
        nx = x + (calc[0]*2)
        ny = y + (calc[1]*2)
        
        if [nx, ny] != current_cell  and current_cell[0]%2==0 and current_cell[1]%2==0:
            self.neighborhood.append([nx, ny])
            if current_cell not in self.exploredpath:
                self.exploredpath.append(current_cell)
            if ([nx, ny] not in self.unexploredpaths) and ([nx, ny] not in self.exploredpath):
                self.unexploredpaths.append([nx, ny])

        if current_cell in self.unexploredpaths:
            self.unexploredpaths.remove(current_cell)
        
        #print(f'{"Neighbor: "}{self.neighborhood}')
        #print(f'{"Unexplored paths: "}{self.unexploredpaths}')
        #print(f'{"Explored paths: "}{self.exploredpaths}')

        self.connections += [[current_cell, neighbor_cell] for neighbor_cell in self.neighborhood
            if [current_cell, neighbor_cell] not in self.connections 
            and [neighbor_cell, current_cell] not in self.connections] 
        
        #print(f'{"Connections list: "}{self.connections}')

        x+= calc[0]
        y+= calc[1]
        
        if self.map[y][x] not in ['-', '|', 'I'] and calc != [0, 0]:
            if y % 2 == 1 and x % 2 == 0:
                self.map[y][x] = "?"
            elif x % 2 == 1 and y % 2 == 0:
                self.map[y][x] = "?"

    def beacons_path(self, current_cell):
        print("BEACONS ENTREI")
        path = []
        num = int(self.nBeacons)
        print(f'{"beacons list: "}{self.beacons}')

        
        if len(self.beacons) == num:
            print("BEACONS PATH")
            for n in range(0, num-1):
                print("n-1 :"+str(self.beacons[n]))
                print("n :"+str(self.beacons[n+1]))

                path += self.get_best_path(self.beacons[n], self.beacons[n+1])


            #print("num -1 : "+str(self.beacons[num-1]))
            #print("0 : "+str(self.beacons[0]))
            #path += self.get_best_path(self.beacons[num-1], self.beacons[0])

            self.save_path(path)
            print(path)
            return path

    def save_path(self, path=None, fout=None):
        if not path:
            path = self.path
        if not fout:
            fout=self.outfile
        s = ""
        for point in path:
            s += str(point[0]) + ', ' + str(point[1])
            s += "\n"

        #print(s)
        with open(fout, "w+") as f:
            f.truncate(0)
            f.write(s)
            f.close()

    def save_map(self, map=None, fout=None):
        if not map:
            map = self.map
        if not fout:
            fout=self.outfile
        s = ""
        for row in reversed(map):
            for val in row:
                s += str(val)
            s += "\n"

        #print(s)
        with open(fout, "w+") as f:
            f.truncate(0)
            f.write(s)
            f.close()
        
    def get_correct_measures(self):
        m = [self.measures.x + self.diffpos[0], self.measures.y + self.diffpos[1]]
        #print("measures: " + str(m))
        return m

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
challenge = 1
outfile = 'solution'

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = sys.argv[i + 1]
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        outfile = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host, challenge, outfile)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
