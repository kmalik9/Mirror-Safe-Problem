"""
Ascent Robotics Mirror-Laser-Safe Problem Solution
Author: Kaustav Malik
"""

import argparse
import operator

from fileReading import *


# parese
parser = argparse.ArgumentParser(description='Ascent Robotics Programming Interview')
parser.add_argument('--input', default='input.txt')
args = parser.parse_args()

#directional constants
UP = 0
DOWN = 1
RIGHT = 2
LEFT = 3

class Safe:

    """Constructor"""
    def __init__(self, rows, cols, m, n):
        self.rows = rows
        self.cols = cols
        self.m = m
        self.n = n
        self.rowMirrors = {}
        self.colMirrors = {}
        self.direction = RIGHT
        self.horizontals = []
        self.verticals = []
        self.intersections = []

    
    """Trace laser path from starting point to find its final position """
    def traverseFwd(self):
        
        #initialize direction and starting position
        #endPosition is used as coordinate variable for segment endpoints
        self.direction = RIGHT
        position = [1,0]
        endPosition = [] 
        
        #minimum and maximum values possible for laser indices
        temp1 = -1
        temp2 = 1000005

        # Checks if first mirror in path exists and traverses accordingly
        if len(self.rowMirrors.get(position[0], [])) > 0:
            endPosition = [position[0], min(x[0] for x in self.rowMirrors.get(position[0], []))]
            self.horizontals.append([position,endPosition])
            position = endPosition
            self.direction = min(x[1] for x in self.rowMirrors.get(position[0], []))
        else:
            endPosition = [1, self.cols]
            self.horizontals.append([position,endPosition])
            return position

        # Until path is fully traversed, updates position and direction based on mirror location and orientation, and laser position and direction
        # Appends horizontal and vertical segments to their respective lists 
        while True:
            if self.direction == RIGHT:
                if len(self.rowMirrors.get(position[0], [])) > 1 and max(x[0] for x in self.rowMirrors.get(position[0], [])) > position[1]:
                    for col,orientation in self.rowMirrors.get(position[0], []):
                        if col>position[1] and col<temp2:
                            temp2 = col
                            if orientation == 0:
                                self.direction = UP
                            else:
                                self.direction = DOWN
                    endPosition = [position[0], temp2]
                    self.horizontals.append([position,endPosition])
                    position = endPosition
                    temp2 = 1000005
                else:
                    endPosition = [position[0], self.cols]
                    self.horizontals.append([position,endPosition])
                    position = endPosition
                    break

            elif self.direction == LEFT:
                if len(self.rowMirrors.get(position[0], [])) > 1 and min(x[0] for x in self.rowMirrors.get(position[0], [])) < position[1]:
                    for col,orientation in self.rowMirrors.get(position[0], []):
                        if col<position[1] and col>temp1:
                            temp1 = col
                            if orientation == 0:
                                self.direction = DOWN
                            else:
                                self.direction = UP
                    endPosition = [position[0], temp1]
                    self.horizontals.append([endPosition,position])
                    position = endPosition
                    temp1 = -1
                else:
                    endPosition = [position[0], 0]
                    self.horizontals.append([endPosition,position])
                    position = endPosition
                    break

            elif self.direction == DOWN:
                if len(self.colMirrors.get(position[1], [])) > 1 and max(x[0] for x in self.colMirrors.get(position[1], [])) > position[0]:
                    for row,orientation in self.colMirrors.get(position[1], []):
                        if row>position[0] and row<temp2:
                            temp2 = row
                            if orientation == 0:
                                self.direction = LEFT
                            else:
                                self.direction = RIGHT
                    endPosition = [temp2,position[1]]
                    self.verticals.append([[position[0],endPosition[0]],position[1]])
                    position = endPosition
                    temp2 = 1000005
                else:
                    endPosition = [self.rows, position[1]]
                    self.verticals.append([[position[0],endPosition[0]],position[1]])
                    position = endPosition
                    break

            elif self.direction == UP:
                if len(self.colMirrors.get(position[1], [])) > 1 and min(x[0] for x in self.colMirrors.get(position[1], [])) < position[0]:
                    for row,orientation in self.colMirrors.get(position[1], []):
                        if row<position[0] and row>temp1:
                            temp1 = row
                            if orientation == 0:
                                self.direction = RIGHT
                            else:
                                self.direction = LEFT
                    endPosition = [temp1, position[1]]
                    self.verticals.append([[endPosition[0],position[0]],position[1]])
                    position = endPosition
                    temp1 = -1
                else:
                    endPosition = [0, position[1]]
                    self.verticals.append([[endPosition[0],position[0]],position[1]])
                    position = endPosition
                    break
        return position

    def traverseBwd(self):

        #initialize direction and starting position
        #endPosition is used as coordinate variable for segment endpoints
        self.direction = LEFT
        position = [self.rows, self.cols]
        endPosition = []
        
        #minimum and maximum values possible for laser indices
        temp1 = -1
        temp2 = 1000005
        
        # Checks if first mirror in path exists and traverses accordingly
        # Updates intersection list if path interects with previously accounted forward path segments
        if len(self.rowMirrors.get(position[0], [])) > 0:
            endPosition = [position[0], max(x[0] for x in self.rowMirrors.get(position[0], []))]

            #looks for intersection in segment dicionaries
            mask = [(el[0][0]<=position[0]) and (el[0][1]>=position[0]) and (el[1]>=endPosition[1]) and (el[1]<=position[1]) for el in self.verticals]
            intersection = [position[0], [self.verticals[i][1] for i in xrange(len(self.verticals)) if mask[i]]]
            if intersection[1]!=[]:
                intersection = [intersection[0], intersection[1][0]] #reformat
                self.intersections.append(intersection)

            #updates position and direction based on mirror
            position = endPosition
            self.direction = max(x[1] for x in self.rowMirrors.get(position[0], []))
            if(self.direction==1):
                self.direction=UP
            else:
                self.direction=DOWN
        
        #if no mirror exists in starting direction, checks for intersection and exits
        else:
            endPosition = [position[0], 0]

            mask = [(el[0][0]<=position[0]) and (el[0][1]>=position[0]) and (el[1]>=endPosition[1]) and (el[1]<=position[1]) for el in self.verticals]
            intersection = [position[0], [self.verticals[i][1] for i in xrange(len(self.verticals)) if mask[i]]]
            if intersection[1]!=[]:
                intersection = [intersection[0], intersection[1][0]] #reformat
                self.intersections.append(intersection)

            position = endPosition


        # Until path is fully traversed, updates position and direction based on mirror location and orientation, and laser position and direction
        # Checks for intersections in path with previously collected forward path segments and stores in list 
        while True:
            if self.direction == RIGHT:
                if len(self.rowMirrors.get(position[0], [])) > 1 and max(x[0] for x in self.rowMirrors.get(position[0], [])) > position[1]:
                    for col,orientation in self.rowMirrors.get(position[0], []):
                        if col>position[1] and col<temp2:
                            temp2 = col
                            if orientation == 0:
                                self.direction = UP
                            else:
                                self.direction = DOWN
                    endPosition = [position[0], temp2]
                    self.findIntersectionRight(position, endPosition)
                    position = endPosition
                    temp2 = 1000005
                else:
                    endPosition = [position[0], self.cols]
                    self.findIntersectionRight(position, endPosition)
                    position = endPosition
                    break

            elif self.direction == LEFT:
                if len(self.rowMirrors.get(position[0], [])) > 1 and min(x[0] for x in self.rowMirrors.get(position[0], [])) < position[1]:
                    for col,orientation in self.rowMirrors.get(position[0], []):
                        if col<position[1] and col>temp1:
                            temp1 = col
                            if orientation == 0:
                                self.direction = DOWN
                            else:
                                self.direction = UP
                    endPosition = [position[0], temp1]
                    self.findIntersectionLeft(position, endPosition)
                    position = endPosition
                    temp1 = -1
                else:
                    endPosition = [position[0], 0]
                    self.findIntersectionLeft(position, endPosition)
                    position = endPosition
                    break

            elif self.direction == DOWN:
                if len(self.colMirrors.get(position[1], [])) > 1 and max(x[0] for x in self.colMirrors.get(position[1], [])) > position[0]:
                    for row,orientation in self.colMirrors.get(position[1], []):
                        if row>position[0] and row<temp2:
                            temp2 = row
                            if orientation == 0:
                                self.direction = LEFT
                            else:
                                self.direction = RIGHT
                    endPosition = [temp2,position[1]]
                    self.findIntersectionDown(position, endPosition)
                    position = endPosition
                    temp2 = 1000005
                else:
                    endPosition = [self.rows, position[1]]
                    self.findIntersectionDown(position, endPosition)
                    position = endPosition
                    break

            elif self.direction == UP:
                if len(self.colMirrors.get(position[1], [])) > 1 and min(x[0] for x in self.colMirrors.get(position[1], [])) < position[0]:
                    for row,orientation in self.colMirrors.get(position[1], []):
                        if row<position[0] and row>temp1:
                            temp1 = row
                            if orientation == 0:
                                self.direction = RIGHT
                            else:
                                self.direction = LEFT
                    endPosition = [temp1, position[1]]
                    self.findIntersectionUp(position, endPosition)
                    position = endPosition
                    temp1 = -1
                else:
                    endPosition = [0, position[1]]
                    self.findIntersectionUp(position, endPosition)
                    position = endPosition
                    break

        #sort intersection list to find lexicographically smallest coordinate
        self.intersections.sort()

    """
    The following four functions find intersections for current segment with previously stored horizontal and vertical segments.
    These functions are called 

    Input: start and end position of current segment
    Output: NA
    Writes intersection values to class's intersection list

    """
    def findIntersectionLeft(self, position, endPosition):
        mask = [(el[0][0]<position[0]) and (el[0][1]>position[0]) and (el[1]>endPosition[1]) and (el[1]<position[1]) for el in self.verticals]
        intersection = [position[0], [self.verticals[i][1] for i in xrange(len(self.verticals)) if mask[i]]]
        if intersection[1]!=[]:
            intersection = [intersection[0], intersection[1][0]] #reformat
            self.intersections.append(intersection)

    def findIntersectionRight(self, position, endPosition):
        mask = [(el[0][0]<position[0]) and (el[0][1]>position[0]) and (el[1]>position[1]) and (el[1]<endPosition[1]) for el in self.verticals]
        intersection = [position[0], [self.verticals[i][1] for i in xrange(len(self.verticals)) if mask[i]]]
        if intersection[1]!=[]:
            intersection = [intersection[0], intersection[1][0]] #reformat
            self.intersections.append(intersection)

    def findIntersectionUp(self, position, endPosition):
        mask = [(el[0][1]<position[1]) and (el[1][1]>position[1]) and (el[0][0]>endPosition[0]) and (el[0][0]<position[0]) for el in self.horizontals]
        intersection = [[self.horizontals[i][0][0] for i in xrange(len(self.horizontals)) if mask[i]], position[1]]
        if intersection[0]!=[]:
            intersection = [intersection[0][0], intersection[1]] #reformat
            self.intersections.append(intersection)

    def findIntersectionDown(self, position, endPosition):
        mask = [(el[0][1]<position[1]) and (el[1][1]>position[1]) and (el[0][0]>position[0]) and (el[0][0]<endPosition[0]) for el in self.horizontals]
        intersection = [[self.horizontals[i][0][0] for i in xrange(len(self.horizontals)) if mask[i]], position[1]]
        if intersection[0]!=[]:
            intersection = [intersection[0][0], intersection[1]] #reformat
            self.intersections.append(intersection)


    """Master function for solving safe"""
    def solve(self):
        position = self.traverseFwd()

        if(position==[self.rows, self.cols]):
            return 0
        else:
            self.traverseBwd()
            if(len(self.intersections)>0):
                return str(len(self.intersections)) + " " + str(self.intersections[0][0]) + " " + str(self.intersections[0][1])
            else:
                return "impossible"



"""
Store mirror information in two seperate dictionaries, one keyed by the row value and the other keyed by the column value.
For each key, the corresponding row/column is stored along with the mirror's orientation

"""
def readMirrors(safe, position, orientation):

    row, col = position

    if safe.rowMirrors.get(row) is None:
        safe.rowMirrors[row] = []
    safe.rowMirrors[row].append((col, orientation))

    if safe.colMirrors.get(col) is None:
        safe.colMirrors[col] = []
    safe.colMirrors[col].append((row, orientation))

    return safe



"""
Use input arguments to parse information on safe
Reads lines of config and parses the information into a list of safes

"""
def parseSafeArgs(config):
    
    safeList = []
    for line in config:
        line_contents = [int(item) for item in line.split(' ')]
        if len(line_contents) == 4:
            rows, cols, m, n = line_contents
            safe = Safe(rows, cols, m, n)
            safeList.append(safe)
            count = 0
        else:
            if count < m:
                safe = readMirrors(safe, line_contents, 0)
            else:
                safe = readMirrors(safe, line_contents, 1)
            count += 1

    return safeList

def main():
    inputConfig = read_input(args.input)
    safeList = parseSafeArgs(inputConfig)
    resultList = []
    for ind, safe in enumerate(safeList):
        result = safe.solve()
        resultStr = "Case {}: {}".format(ind, result)
        print(resultStr)
        resultList.append(resultStr)
    write_to_file(resultList)


if __name__=='__main__':
    main()