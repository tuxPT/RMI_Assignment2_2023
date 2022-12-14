
import sys
from colorama import Fore, Back, Style
import numpy
from croblink import *
from math import *
import math
import xml.etree.ElementTree as ET
from tree_search import *
from collections import deque
import wander

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
        self.rob_name = rob_name
        self.challenge = challenge
        self.outfile = outfile
        self.pid_Controller = PIDController(0.07, 10, 0.000001, 0.050, 0.001, math.inf, 0.000001)
        self.action = 'stop'
        self.checkNeighbors = 0
        if self.challenge == "1":
            self.track = []
        else:
            self.map = [[" " for j in range(1,50)] for i in range(1,22)]
            self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
            self.path = []
            #print(len(self.map)*len(self.map[0]))
            self.map[10][24] = "I"
            #print(self.map)
            self.unexploredpaths = set()
            self.connections = []
            self.neighborhood = []
            self.pathtoexplore = []
            self.exploredpath = set()
            self.has_path = 'path_finding'
            self.rotation = 0
            self.dest = None
            self.beacons = []
            self.lin = 0
            self.prev_pos = (24, 10)
            self.prev_angle = 0
        self.linesensor_buffer = deque()
        self.compass_buffer = deque()

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
                print(self.robName + " exiting")
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
        if self.challenge == "1":
            self.wanderC1()
        elif self.challenge == "2":
            self.wanderC2()
        elif self.challenge == "3":
            self.wanderC3()
        elif self.challenge == "4":
            self.wanderC4()

    def go(self,lin,k,m,r):
        rot = k*(m - r)
        lpow = lin + (rot/2)
        rpow = lin - (rot/2)
        return [lpow, rpow]


    def detect_and_correct_error(self, array):
        count = 0
        index = None
        if array.count(0) == 7:
            print(Back.RED+f'OUT')
            print(Style.RESET_ALL)
        for i in range(0, 7):
            for j in range(i+1, 7):
                if array[i] == 1 and array[j] == 1 and array[i:j+1].count(0):
                    if index == None:
                        print(Back.YELLOW+Fore.BLACK+f'DETECTED: {array}')
                        print(Style.RESET_ALL)
                        index = array.index(0, i, j)
                    if count < array[i:j+1].count(0):
                        count = array[i:j+1].count(0)
        if index:
            print(Fore.GREEN+f'index: {index}')
            print(f'count: {count}')
            print(Style.RESET_ALL)
            if count == 1:
                array[index+1] = 1
            else:
                #if count == 1:
                #    array[index] = '1'
                """ if array[6] == '1' and count < 2 and index > 3:
                    array = ['0']*count + array[0:index] + array[index+count:7]
                elif array[0] == '1' and count < 2 and index < 3:
                    array = array[0:index] + array[index+count:7] + ['0']*count """
                if array[0:index].count(1) > array[index+1:7].count(1):
                    array = array[0:index] + array[index+count:7] + [0 for i in range(0, count)]
                else:
                    array = [0 for i in range(0, count)] + array[0:index] + array[index+count:7]

            print(f'Corrected: {array}')
        return array
    
    @staticmethod
    def angle_dista(angle1, angle2):
        diff = (angle2 - angle1 + 180) % 360 -180
        if diff < -180:
            diff += 360
        return diff

    def weighted_average(self):
        if self.measures.lineSensor.count('0') == 7:
            print(Back.RED+f'OUT')
            print(Style.RESET_ALL)
        last_value = [0 if i == '0' else 1 for i in self.measures.lineSensor]

        if last_value.count(1) == 2:
            index = last_value.index(1)
            if 0 < index < 5:
                if index < 3:
                    last_value[index-1] = 1
                else:
                    last_value[index+2] = 1
        last_value = self.detect_and_correct_error(last_value)
        print(f'last_value:       {last_value}')
        if len(self.linesensor_buffer) < 3:
            self.linesensor_buffer.extend([last_value, last_value, last_value])
        self.linesensor_buffer.append(last_value)
        if len(self.linesensor_buffer) > 3:
            self.linesensor_buffer.popleft()
        print(f'deque:            {self.linesensor_buffer}')
        s = [0]*7
        for i in range(0, len(self.linesensor_buffer)):
            for j in range(0, 7):
                s[j] += self.linesensor_buffer[i][j]*(i+1)/6

        print(f'Weighted average(lineSensor): {s}')
        for i in range(0, 7):
            s[i] = round(s[i]-0.00001)

        s = self.detect_and_correct_error(s)
        return s

    def get_correct_compass(self):
        last_value = self.measures.compass
        print(f'last_value:       {last_value}')
        if len(self.compass_buffer) < 3:
            self.compass_buffer.extend([last_value, last_value, last_value])
        self.compass_buffer.append(last_value)
        if len(self.compass_buffer) > 3:
            self.compass_buffer.popleft()
        print(f'deque:            {self.compass_buffer}')
        s = 0
        aux_array = self.compass_buffer
        for i in range(0, len(self.compass_buffer)):
            if i > 0:
                diff = self.angle_dista(self.compass_buffer[i], self.compass_buffer[i-1])
                print(f'DIFF COMPASS {diff}')
                if self.compass_buffer[i] - self.compass_buffer[i-1] > 180:
                    aux_array[i] -= 360
                elif self.compass_buffer[i] - self.compass_buffer[i-1] < -180:
                    aux_array[i] += 360
            s += aux_array[i]*(i+1)/6

        s = (s+180)%360-180
        self.prev_angle = s/180*math.pi

        print(f'Weighted average(angle): {s}')
        return s

    def hasNeighbor(self, x, y):
        # pesquisa no mapa em vez da lista
        for point in self.unexploredpaths:
            if point != (x,y):
                dist = abs(x-point[0]) + abs(y-point[1])
                if dist == 2:
                    return True
        return False


    def wanderC4(self):
        print("#############################################")

        lineSensor = self.weighted_average()
        x,y = self.prev_pos = self.get_correct_measures()
        xn, yn = [2*round(x/2), 2*round(y/2)]
        angle = self.get_correct_compass()
        print(f'x: {x}, y: {y}')

        if self.measures.ground != -1:
            if [x, y] not in self.beacons:
                self.beacons.append([xn, yn])
                self.map[yn][xn] = str(self.measures.ground)

        self.exploredpath.add((xn, yn))
        self.rem_unexplored(x, y)

        diff = self.angle_dista(self.rotation, angle)

        # if there is a path both to the right and to the left
        if lineSensor.count(1) == 7 and abs(diff) < 10:
            nx, ny, _ = self.getAdjacentPos('left', angle)
            x, y = self.prev_pos
            self.add_unexplored(nx, ny)                 
            self.add_connection(nx, ny, x, y)
            nx, ny, _ = self.getAdjacentPos('right', angle)
            self.add_unexplored(nx, ny)              
            self.add_connection(nx, ny, x, y)

        # if there is a path to the left
        elif lineSensor[0] == 1 and abs(diff) < 10:
            if lineSensor.count(1) > 4:
                nx, ny, _ = self.getAdjacentPos('left', angle)
                x, y = self.prev_pos

                self.add_unexplored(nx, ny)            
                self.add_connection(nx, ny, x, y)
            
        # if there is a path to the right
        elif lineSensor[6] == 1 and abs(diff) < 10:
            if lineSensor.count(1) > 4:
                nx, ny, _ = self.getAdjacentPos('right', angle)
                x, y = self.prev_pos

                self.add_unexplored(nx, ny)            
                self.add_connection(nx, ny, x, y)
        
        print(lineSensor)

        nx, ny, coef = self.getAdjacentPos('front', angle)
        if 4 > lineSensor.count(1) > 1 and 4 > lineSensor[2:5].count(1) > 1 and lineSensor[0] != 1 and lineSensor[6] != 1 \
            and ((abs(x - 2*round(x/2)) < 0.3 and coef[0]) or (abs(y - 2*round(y/2)) < 0.3 and coef[1])) and abs(diff) < 10:
            print(f'lineSENSOR CRLLLLL {lineSensor}')
            x, y = self.prev_pos
            # add connection on x,y at border cell
            print(f'NX, NY: {nx}, {ny}')
            self.add_unexplored(nx, ny)                
            self.add_connection(nx, ny, x, y)
            self.checkNeighbors = 0
        
        # path exists
        if len(self.path):                
            # destination reached
            if (self.dest[0] == 'x' and abs(self.dest[1] - x) <= 0.05) or (self.dest[0] == 'y' and abs(self.dest[1] - y) <= 0.05):
                self.path = self.path[1:]
                if len(self.path) == 0 and len(self.unexploredpaths) == 0 and len(self.beacons) == int(self.nBeacons):
                    print('finish')
                    self.has_path = 'stop'
                    self.beacons_path()
                    self.finish()
                    return
                # full path reached
                if len(self.path) == 0:
                    if not self.hasNeighbor(xn, yn):
                        self.checkNeighbors = 5
                else:
                    self.set_destination_and_rotation(x, y)
                self.action = 'rotate'
        elif self.checkNeighbors:
            self.action = 'rotate'
        # path does not exist
        else:
            self.set_path(x, y)
            self.set_destination_and_rotation(x, y)
            self.action = 'rotate'
        
        diff = self.angle_dista(self.rotation, angle)
        # forward
        if self.action == 'forward':
            lpow, rpow = self.forward(x, y, angle, lineSensor)
        # rotate
        else:
            print(Back.BLUE+Fore.BLACK+'ROTATE')
            print(Style.RESET_ALL)
            lpow, rpow = self.rotate(angle)

        print(f'path = {self.path}')
        print(f'lineSensorOriginal = {self.measures.lineSensor}')
        print(f'lineSensorCorrected = {lineSensor}')
        print(f'exploredpath = {self.exploredpath}')
        print(f'unexploredpath = {self.unexploredpaths}')
        print(f'connections = {self.connections}')
        #print(f'destination = {self.dest[0]}, {self.dest[1]}')
        print(f'lpow, rpow = {lpow}, {rpow}')
        self.driveMotors(lpow, rpow)
        self.update_model(lpow, rpow)
    
        self.save_map()

        print("#############################################")

    def getAdjacentPos(self, direction, angle):
        print(Back.GREEN+Fore.BLACK+f'HAS {direction}')
        print(Style.RESET_ALL)
        
        coef = self.get_neightbor_coeficient(direction, angle)
        x, y = self.prev_pos
        if direction != 'front' and abs(self.rotation+180 - angle-180) < 10:
            x = 2*round(x/2)
            y = 2*round(y/2)
            if self.rotation == 0:
                x -= 0.438
            elif self.rotation == -180:
                x += 0.438
            if self.rotation == 90:
                y -=0.438
            elif self.rotation == -90:
                y += 0.438
            self.prev_pos = [x, y]

            print(f'Corrected x: {x}, y: {y}')

        nx = x
        ny = y


       
        if self.rotation == 0:
            nx += 0.438
        elif self.rotation == -180:
            nx -= 0.438
        if self.rotation == 90:
            ny +=0.438
        elif self.rotation == -90:
            ny -= 0.438

        nx = 2*round(nx/2) + 2*coef[0]
        ny = 2*round(ny/2) + 2*coef[1]
        return [nx, ny, coef]


    def forward(self, x, y, angle, lineSensor):
        if (self.dest[0] == 'x' and abs(self.dest[1] - x) <= 0.02) \
            or (self.dest[0] == 'y' and abs(self.dest[1] - y) <= 0.02):
            self.action = 'rotate'
            return 0, 0
        pow = 0
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

        diff = self.angle_dista(self.rotation, angle)
        if abs(x - 2*round(x/2)) <= 0.01 and abs(y - 2*round(y/2)) <= 0.01 and lineSensor.count(0) == 7 and abs(diff) < 1:
            p = [2*round(x/2), 2*round(y/2)]
            if len(self.path):
                self.rem_connection(self.path[0][0], self.path[0][1], 2*round(x/2), 2*round(y/2))
                unexplored_connections = [connection for connection in self.connections if connection[0] ==  p or connection[1] == p]
                if len(unexplored_connections) == 1:
                    self.rem_unexplored(self.path[0][0], self.path[0][1])
                print(f'REMOVED CONNECTION [{self.path[0]}, {p}]')
            self.path = []
            return 0, 0

       
        # centrar o lineSensor
        if ((self.dest[0] == 'x' and abs(self.dest[1] - x) > 0.5) or (self.dest[0] == 'y' and abs(self.dest[1] - y) > 0.5)):
            return self.go(pow, 0.1, lineSensor[3:7].count(1)/4, lineSensor[0:4].count(1)/4)
        # centrar angulo
        else:
            print(Back.BLUE+Fore.WHITE+'ANGLE FORWARD')
            print(Style.RESET_ALL)
            
            return self.go(pow, 2, diff/180, 0)

    def rotate(self, angle):
        
        print(Back.GREEN+Fore.BLACK+f'ROTATION = {self.rotation}, ANGLE = {angle}')

        print(Style.RESET_ALL)
        diff = self.angle_dista(self.rotation, angle)
        if abs(diff) < 0.5 and not self.checkNeighbors:
            self.action = 'forward'
            return 0, 0

        if self.checkNeighbors and abs(diff) < 0.5:
            self.rotation = (self.rotation + 180 + 90)%360 -180
            self.checkNeighbors -= 1
        
        diff = self.angle_dista(self.rotation, angle)
        print(f'DIFF COMPASS {diff}')
        pow = self.pid_Controller.go(0, diff, True)
        return -pow, pow

    # returns the closest non explored point to the current position
    def find_next(self, x, y):
        # pesquisa no mapa em vez da lista
        minimum = math.inf
        target = None
        print(self.unexploredpaths)
        for point in self.unexploredpaths:
            if point != (x,y):
                dist = abs(x-point[0]) + abs(y-point[1])
                if dist < minimum:
                    minimum = dist
                    target = point
        return [target[0], target[1]]

    # based on tree_search module and list of connections, returns the best path from origin to destination
    def get_best_path(self, origin, destination):
        print(f'ORIGIN = {origin}')
        self.pathtoexplore = []

        self.pathtoexplore += SearchTree(SearchProblem(Domain(self.connections), origin, destination), 'a*').search(2000)[0]

        return self.pathtoexplore

    def get_neightbor_coeficient(self, direction, compass):
        calc = [0, 0]
        # up
        if 80 < compass < 100:
            if direction == 'left':
                calc = [-1, 0]
            elif direction == 'front':
                calc = [0, 1]
            else:
                calc = [1, 0]
        # right
        elif -10 < compass < 10:
            if direction == 'left':
                calc = [0, 1]
            elif direction == 'front':
                calc = [1, 0]
            else:
                calc = [0, -1]
        # left
        elif -180 <= compass < -170 or 170 < compass <= 180:
            if direction == 'left':
                calc = [0, -1]
            elif direction == 'front':
                calc = [-1, 0]
            else:
                calc = [0, 1]
        # down
        elif -100 < compass < -80:
            if direction == 'left':
                calc = [1, 0]
            elif direction == 'front':
                calc = [0, -1]
            else:
                calc = [-1, 0]

        return calc

    def add_connection(self, nx, ny, x, y):
        x = 2*round(x/2)
        y = 2*round(y/2)
        if [[x, y], [nx, ny]] not in self.connections and [[nx, ny], [x, y]] not in self.connections:
            self.connections.append([[x, y], [nx, ny]])
            self.write_known_path(x, y, nx, ny)
            print(Back.YELLOW+Fore.BLACK+f'added connection [{x}, {y}] <-> [{nx}, {ny}]')
            print(Style.RESET_ALL)
    
    def rem_connection(self, nx, ny, x, y):
        x = 2*round(x/2)
        y = 2*round(y/2)
        if [[x, y], [nx, ny]] in self.connections:
            self.connections.remove([[x, y], [nx, ny]])
            #self.write_known_path(x, y, nx, ny)
            print(Back.YELLOW+Fore.BLACK+f'added connection [{x}, {y}] <-> [{nx}, {ny}]')
            print(Style.RESET_ALL)
        elif [[nx, ny], [x, y]] in self.connections:
            self.connections.remove([[nx, ny], [x, y]])
            #self.write_known_path(x, y, nx, ny)
            print(Back.YELLOW+Fore.BLACK+f'added connection [{nx}, {ny}] <-> [{x}, {y}]')
            print(Style.RESET_ALL)



    def rem_unexplored(self, x, y):
        nx = round(x)
        ny = round(y)
        if (nx, ny) in self.unexploredpaths:
            self.unexploredpaths.remove((nx, ny))       

    def add_unexplored(self, nx, ny):
        if (nx, ny) != (24, 10) and (nx, ny) not in self.exploredpath:
            self.unexploredpaths.add((nx, ny))
            print(Back.YELLOW+Fore.BLACK+f'added unexplored {nx}, {ny}')
            print(Style.RESET_ALL)

    def write_known_path(self, ix, iy, fx, fy):
        x = int((ix+fx)/2)
        y = int((iy+fy)/2)

        # every odd line
        if self.map[y][x] in [' ', '|', '-']:
            if y % 2 == 1:
                self.map[y][x] = "|"
            # every odd column and even linedist
            # every odd column and even line
            elif x % 2 == 1:
                self.map[y][x] = "-"
            else:
                pass

    def set_path(self, x, y):
        # go for unexplored paths
        x = 2*round(x/2)
        y = 2*round(y/2)
        unexploredcell = self.find_next(x, y)
        print(f'unexploredcell = {unexploredcell}')
        self.path = self.get_best_path([x, y], unexploredcell)
        self.path_map = [[" " for j in range(1,50)] for i in range(1,22)]
        for cell in self.path:
            self.path_map[cell[1]][cell[0]] = 'P'
        self.save_map(self.path_map, "path.txt")
    
    def set_destination_and_rotation(self, x, y):
        horizontal = self.path[0][0] - 2*round(x/2)
        vertical = self.path[0][1] - 2*round(y/2)
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

    def write_unknown_path(self, x, y):
        x = round(x)
        y = round(y)
        
        if self.map[y][x] not in ['-', '|', 'I'] and calc != [0, 0]:
            if y % 2 == 1 and x % 2 == 0:
                self.map[y][x] = "?"
            elif x % 2 == 1 and y % 2 == 0:
                self.map[y][x] = "?"

    def beacons_path(self):
        path = []
        num = int(self.nBeacons)

        if len(self.beacons) == num:
            path = [self.beacons[0]]
            for n in range(0, num-1):

                path += self.get_best_path(self.beacons[n], self.beacons[n+1])

            path += self.get_best_path(self.beacons[num-1], self.beacons[0])
            
            self.save_path(path)
            return path

    def save_path(self, path=None, fout=None):
        if not path:
            path = self.path
        if not fout:
            fout=self.outfile if self.challenge != '4' else self.outfile+'.path'
        s = ""
        for point in path:
            s += str(point[0]-24) + ' ' + str(point[1]-10)
            s += "\n"

        with open(fout, "w+") as f:
            f.truncate(0)
            f.write(s)
            f.close()

    def save_map(self, map=None, fout=None):
        if not map:
            map = self.map
        if not fout:
            fout=self.outfile if self.challenge != '4' else self.outfile+'.map'
        s = ""
        for row in reversed(map):
            for val in row:
                s += str(val)
            s += "\n"

        with open(fout, "w+") as f:
            f.truncate(0)
            f.write(s)
            f.close()
        
    def get_correct_measures(self):
        if self.challenge == '4':
            x = self.prev_pos[0] + self.lin*math.cos(self.prev_angle)
            y = self.prev_pos[1] + self.lin*math.sin(self.prev_angle)
            return [x, y]
        else:
            return [self.measures.x + self.diffpos[0], self.measures.y + self.diffpos[1]]


    def update_model(self, lpow, rpow):
        self.lin = (lpow + rpow)/2
        rot = rpow - lpow
        self.prev_angle = self.prev_angle + rot
        print(f'lin = {self.lin} \nprev_angle = {self.prev_angle*180/math.pi}')

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
