import random
import math
import pygame
from scipy import interpolate
import numpy as np
from tester import get_slope
import pickle

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        # window settings
        self.MapWindowName = 'RRT path planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        imp = pygame.image.load("C:\\Users\\musto\\OneDrive\\Desktop\\RRT-4DOF_WORKING\\map.png").convert()
        self.map.blit(imp, (0, 0))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        # Colors
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

 ################DRAW OBSTACLES###########################
    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.Green, (self.start[0],self.start[1]), self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Green, (self.goal[0],self.goal[1]), self.nodeRad + 20, 1)
        ###########################
        #self.drawObs(obstacles)

    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.Red, (node[0],node[1]), 3, 0)


    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)

##################################################################################


class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x, y, v,th) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.v =[]
        self.th =[]
        self.parent = []
        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.v.append(v)
        self.th.append(th)
        self.parent.append(0)
        # the obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum
        # path
        self.goalstate = None
        self.path = []


    #OBSTACLE
    def makeRandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))

        return (uppercornerx, uppercornery)
    
    #OBSTACLE
    def makeobs(self):
        obs = []
        for i in range(0, self.obsNum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))

                if rectang.collidepoint((self.start[0], self.start[1]) or rectang.collidepoint(self.goal[0], self.goal[1])):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs

    #PART 2
    def add_node(self, n, x, y, v, th):
        self.x.insert(n, x)
        self.y.insert(n, y)
        self.v.insert(n, v)
        
        #self.y.append(y)
        self.th.append(th)
    
    #PART 2
    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)
        self.v.pop(n)
        self.th.pop(n)

    #PART 2
    def add_edge(self, parent, child):
        self.parent.insert(child, parent)
    #PART 2
    def remove_edge(self, n):
        self.parent.pop(n)
    #PART 2
    def number_of_nodes(self):
        return len(self.x)
    #PART 2
    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        #ECLUADIAN DISTANCE
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        #print(px,py)
        return (px + py) ** (0.5)

   
    def sample_envir(self):
        
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        v = int(random.uniform(1, 10))
        th = int(random.uniform(9, 13))
       
        return x, y, v, th
    #PART 3 
    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear


    def isSlip_distribution(self, x,y,node_len,v_sec):
        
        slope=get_slope(x,y)
        slip=slope
        if slip<=150:
            return True      
        return False

    def isSlip(self, x,y,v):
        slope=get_slope(x,y)
        slip=slope
        if slip<=150:
            return True      
        return False

    def check_no_slip(self):
        n = self.number_of_nodes() - 1
        n_pr = self.number_of_nodes() - 2
        #print(self.v[n], self.v[n_pr])
        """max_vel=to choose the max velocity between 2 nodes"""
        max_vel=max(self.v[n], self.v[n_pr])
        (x, y,v) = (self.x[n], self.y[n], max_vel)
        if self.isSlip(x,y,v):
            self.remove_node(n)
            return False
        return True

       

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x, y): 
                self.remove_node(n)
                return False
        return True


    def crossSlip(self, x1, x2, y1, y2,v1,v2):
        #ECLUADIAN DISTANCE
       # node_len=  math.hypot(x2 - x1, y2 - y1)
        max_v = max(v1,v2)
        #print(node_len)
        for i in range(0, 11):
            u = i / 10
            x = x1 * u + x2 * (1 - u)
            y = y1 * u + y2 * (1 - u)
            #print("s",abs(x-x1))
            distance=abs(x-x1)
            v_sec=max_v*u
            #print("try", abs(node_len-x1))
            """isSlip_distribution = function to generate slip distribution and check slip"""
            if self.isSlip_distribution(x, y,distance,v_sec):
                return True
        return False
       

    #PART 2 (NEED EDIT ON INTERPOLATION RANGE)
    def crossObstacle(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rectang = obs.pop(0)
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
                if rectang.collidepoint(x, y):
                    return True
        return False
    #PART 2 (TO EXTRACT INFO FOR V)


    def connect(self, n1, n2):
        (x1, y1,v1) = (self.x[n1], self.y[n1],self.v[n1])
        (x2, y2, v2) = (self.x[n2], self.y[n2], self.v[n2])

        #if self.crossObstacle(x1, x2, y1, y2):
        if self.crossSlip(x1, x2, y1, y2, v1, v2):
            self.remove_node(n2)
            #print(n1)
            self.goalFlag=False
            return False
        else:
            self.add_edge(n1, n2)
            return True

    #PART 3 
    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax:
   
            #u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            v1=self.v[nnear]
            v = self.v[nrand]
            #print(v1,v)
            #THE WAY TO ADD POINT B IN THE SAME LINE FROM A TO C 
            (px, py) = (xrand - xnear, yrand - ynear)
            th = math.atan2(py, px)
            
            #THE NEW POINT COORDINATE
            (x, y) = (int(xnear + dmax * math.cos(th)),int(ynear + dmax * math.sin(th)))
            
            self.remove_node(nrand)
        ##############################################
            #THE GOAL TEST
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1] , self.goal[2], self.goal[3])
                self.goalstate = nrand
                self.goalFlag = True
            else:
          
                self.add_node(nrand, x, y, v,th)
########################################################################################
########################################################################################
########################################################################################
    def bias(self, ngoal):
        n = self.number_of_nodes()
          
        self.add_node(n, ngoal[0], ngoal[1], ngoal[2], ngoal[3])
   
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)

        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y, v,th = self.sample_envir()
        max_velo = abs(self.v[-2]-self.v[-1])
        # print(self.v)
        # print(self.v[-1])
        # print(self.v[-2])
        # print(max_velo)
        # input(10000)
        a = 0
        b=  0
        if len(self.th) == 0 and max_velo<= 3:
            a = a+1
            
            self.add_node(n, x, y, v,th)
        else:
            b = b+ 1
            
            self.add_node(n, x, y, v,self.th[-1])
        print(a)
        print(b)
        if self.check_no_slip():
          
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)

        return self.x, self.y, self.parent


    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
  
        return self.goalFlag

#####################################################################

    def getPathCoords(self):
        pathCoords = []
        
        for node in self.path:
            x, y, v,th = (self.x[node], self.y[node], self.v[node], self.th[node])
        
            pathCoords.append((x, y, v,th))
    
        return pathCoords
#########################################################################


 #TODO:
#
    # def cost(self, n):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c

#     def getTrueObs(self, obs):
#         TOBS = []
#         for ob in obs:
#             TOBS.append(ob.inflate(-50, -50))
#         return TOBS


# ###############################################################################################
# def makeRandomRect(self):
#     uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
#     uppercornery = int(random.uniform(0, self.maph - self.obsDim))
#     return (uppercornerx, uppercornery)

# def makeobs(self):
#     obs = []
#     for i in range(0, self.obsNum):
#         rectang = None
#         startgoalcol = True
#         while startgoalcol:
#             upper = self.makeRandomRect()
#             rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))
#             if rectang.collidepoint((self.start[0], self.start[1]) or rectang.collidepoint(self.goal[0], self.goal[1])):
#             #if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
#                 startgoalcol = True
#             else:
#                 startgoalcol = False
#             obs.append(rectang)
#         self.obstacles = obs.copy()
#     return obs








































