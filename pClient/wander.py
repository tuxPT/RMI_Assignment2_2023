def wanderC1(self):
    linSensor = self.weighted_average()
    lpow, rpow = self.go(0.15, 2, linSensor[0:3].count(
        0)/7, linSensor[4:7].count(0)/7)
    self.driveMotors(lpow, rpow)

def wanderC2(self):
    x, y = self.get_correct_measures()

    self.exploredpath.add((2*round(x/2), 2*round(y/2)))
    self.rem_unexplored(x, y)

    # if there is a path both to the right and to the left
    if self.measures.lineSensor.count('1') == 7:
        coef = self.get_neightbor_coeficient('left')
        nx = x
        ny = y
        if self.rotation == 0:
            nx += 0.438
        elif self.rotation == -180:
            nx -= 0.438
        if self.rotation == 90:
            ny += 0.438
        elif self.rotation == -90:
            ny -= 0.438
        nx = 2*round(nx/2) + 2*coef[0]
        ny = 2*round(ny/2) + 2*coef[1]
        self.add_unexplored(nx, ny)
        self.add_connection(nx, ny)
        coef = self.get_neightbor_coeficient('right')
        nx = x
        ny = y
        if self.rotation == 0:
            nx += 0.438
        elif self.rotation == -180:
            nx -= 0.438
        if self.rotation == 90:
            ny += 0.438
        elif self.rotation == -90:
            ny -= 0.438
        nx = 2*round(nx/2) + 2*coef[0]
        ny = 2*round(ny/2) + 2*coef[1]
        self.add_unexplored(nx, ny)
        self.add_connection(nx, ny)
    # if there is a path to the left
    elif self.measures.lineSensor[0] == '1':
        if self.measures.lineSensor.count('1') > 3:
            coef = self.get_neightbor_coeficient('left')
            nx = x
            ny = y
            if self.rotation == 0:
                nx += 0.438
            elif self.rotation == -180:
                nx -= 0.438
            if self.rotation == 90:
                ny += 0.438
            elif self.rotation == -90:
                ny -= 0.438
            nx = 2*round(nx/2) + 2*coef[0]
            ny = 2*round(ny/2) + 2*coef[1]
            self.add_unexplored(nx, ny)
            self.add_connection(nx, ny)

    # if there is a path to the left
    elif self.measures.lineSensor[6] == '1':
        if self.measures.lineSensor.count('1') > 3:
            coef = self.get_neightbor_coeficient('right')
            nx = x
            ny = y
            if self.rotation == 0:
                nx += 0.438
            elif self.rotation == -180:
                nx -= 0.438
            if self.rotation == 90:
                ny += 0.438
            elif self.rotation == -90:
                ny -= 0.438
            nx = 2*round(nx/2) + 2*coef[0]
            ny = 2*round(ny/2) + 2*coef[1]
            self.add_unexplored(nx, ny)
            self.add_connection(nx, ny)

    if '1' in self.measures.lineSensor[2:5]:
        coef = self.get_neightbor_coeficient('front')
        nx = x
        ny = y
        if self.rotation == 0:
            nx += 0.438
        elif self.rotation == -180:
            nx -= 0.438
        if self.rotation == 90:
            ny += 0.438
        elif self.rotation == -90:
            ny -= 0.438
        nx = 2*round(nx/2) + 2*coef[0]
        ny = 2*round(ny/2) + 2*coef[1]
        # add connection on x,y at border cell
        if (x == 2*round(x/2) and coef[0]) or (y == 2*round(y/2) and coef[1]):
            self.add_unexplored(nx, ny)
            self.add_connection(nx, ny)

    if self.has_path == 'path_finding':
        # path exists
        if len(self.path):
            # destination reached
            if (self.dest[0] == 'x' and self.dest[1] == x) or (self.dest[0] == 'y' and self.dest[1] == y):
                self.path = self.path[1:]
                if len(self.path) == 0 and len(self.unexploredpaths) == 0:
                    print('finish')
                    self.has_path = 'stop'
                    self.finish()
                    return
                # full path reached
                if len(self.path) == 0:
                    self.set_path(x, y)
                self.set_destination_and_rotation(x, y)
        # path does not exist
        else:
            self.set_path(x, y)
            self.set_destination_and_rotation(x, y)

        # forward
        if ((self.dest[0] == 'x' and self.dest[1] != x) or (self.dest[0] == 'y' and self.dest[1] != y)) and abs(self.rotation-self.measures.compass) <= 2:
            lpow, rpow = self.forward(x, y)
        # rotate
        else:
            lpow, rpow = self.rotate()

        self.driveMotors(lpow, rpow)

    if len(self.unexploredpaths) == 0 and len(self.path) == 0:
        self.finish()

    self.save_map()

def wanderC3(self):
    x, y = self.get_correct_measures()

    x = round(x)
    y = round(y)

    if self.measures.ground != -1:
        if [x, y] not in self.beacons:
            self.beacons.append([2*round(x/2), 2*round(y/2)])

    if self.has_path == 'path_finding':
        x, y = self.get_correct_measures()

        self.exploredpath.add((2*round(x/2), 2*round(y/2)))
        self.rem_unexplored(x, y)

        # if there is a path both to the right and to the left
        if self.measures.lineSensor.count('1') == 7:
            coef = self.get_neightbor_coeficient('left')
            nx = x
            ny = y
            if self.rotation == 0:
                nx += 0.438
            elif self.rotation == -180:
                nx -= 0.438
            if self.rotation == 90:
                ny += 0.438
            elif self.rotation == -90:
                ny -= 0.438
            nx = 2*round(nx/2) + 2*coef[0]
            ny = 2*round(ny/2) + 2*coef[1]
            self.add_unexplored(nx, ny)
            self.add_connection(nx, ny)
            coef = self.get_neightbor_coeficient('right')
            nx = x
            ny = y
            if self.rotation == 0:
                nx += 0.438
            elif self.rotation == -180:
                nx -= 0.438
            if self.rotation == 90:
                ny += 0.438
            elif self.rotation == -90:
                ny -= 0.438
            nx = 2*round(nx/2) + 2*coef[0]
            ny = 2*round(ny/2) + 2*coef[1]
            self.add_unexplored(nx, ny)
            self.add_connection(nx, ny)

        # if there is a path to the left
        elif self.measures.lineSensor[0] == '1':
            if self.measures.lineSensor.count('1') > 3:
                coef = self.get_neightbor_coeficient('left')
                nx = x
                ny = y
                if self.rotation == 0:
                    nx += 0.438
                elif self.rotation == -180:
                    nx -= 0.438
                if self.rotation == 90:
                    ny += 0.438
                elif self.rotation == -90:
                    ny -= 0.438
                nx = 2*round(nx/2) + 2*coef[0]
                ny = 2*round(ny/2) + 2*coef[1]
                self.add_unexplored(nx, ny)
                self.add_connection(nx, ny)

        # if there is a path to the left
        elif self.measures.lineSensor[6] == '1':
            if self.measures.lineSensor.count('1') > 3:
                coef = self.get_neightbor_coeficient('right')
                nx = x
                ny = y
                if self.rotation == 0:
                    nx += 0.438
                elif self.rotation == -180:
                    nx -= 0.438
                if self.rotation == 90:
                    ny += 0.438
                elif self.rotation == -90:
                    ny -= 0.438
                nx = 2*round(nx/2) + 2*coef[0]
                ny = 2*round(ny/2) + 2*coef[1]
                self.add_unexplored(nx, ny)
                self.add_connection(nx, ny)

        if '1' in self.measures.lineSensor[2:5]:
            coef = self.get_neightbor_coeficient('front')
            nx = x
            ny = y
            if self.rotation == 0:
                nx += 0.438
            elif self.rotation == -180:
                nx -= 0.438
            if self.rotation == 90:
                ny += 0.438
            elif self.rotation == -90:
                ny -= 0.438
            nx = 2*round(nx/2) + 2*coef[0]
            ny = 2*round(ny/2) + 2*coef[1]

            # add connection on x,y at border cell
            if (x == 2*round(x/2) and coef[0]) or (y == 2*round(y/2) and coef[1]):
                self.add_unexplored(nx, ny)
                self.add_connection(nx, ny)

        if self.has_path == 'path_finding':
            # path exists
            if len(self.path):
                # destination reached
                if (self.dest[0] == 'x' and self.dest[1] == x) or (self.dest[0] == 'y' and self.dest[1] == y):
                    self.path = self.path[1:]
                    if len(self.path) == 0 and len(self.unexploredpaths) == 0 and len(self.beacons) == int(self.nBeacons):
                        print('finish')
                        self.has_path = 'stop'
                        self.beacons_path()
                        self.finish()
                        return
                    # full path reached
                    if len(self.path) == 0:
                        self.set_path(x, y)
                    self.set_destination_and_rotation(x, y)
            # path does not exist
            else:
                self.set_path(x, y)
                self.set_destination_and_rotation(x, y)

            # forward
            if ((self.dest[0] == 'x' and self.dest[1] != x) or (self.dest[0] == 'y' and self.dest[1] != y)) and abs(self.rotation-self.measures.compass) <= 2:
                lpow, rpow = self.forward(x, y)
            # rotate
            else:
                lpow, rpow = self.rotate()
        else:
            lpow, rpow = 0, 0

        self.driveMotors(lpow, rpow)

        #print('rotation = ' + str(self.rotation))
        #print('compass = ' + str(self.measures.compass))
        # self.save_path()
