from importlib.resources import path
import pygame
from RRTbasePy import RRTGraph
from RRTbasePy import RRTMap
import time

import random
import math
import pygame
from scipy import interpolate
import numpy as np
#import dubins

turning_radius = 1.0
step_size = 0.5
path_all=[]
    
def B_spline(waypoints):
        x_=[]
        y_=[]
        for point in waypoints:
            x_.append(point[0])
            y_.append(point[1])
        tck, *rest = interpolate.splprep([x_,y_])
        u = np.linspace(0,1, num=100)
        smooth= interpolate.splev(u, tck)
        return smooth

def main():
    dimensions =(512,512)
    start=(0,0,1,1)
    goal=(400,400,1,1)
    
    obsdim=30
    obsnum=50
    iteration=0
    t1=0

    pygame.init()
    map=RRTMap(start,goal,dimensions,obsdim,obsnum)
    graph=RRTGraph(start,goal,dimensions,obsdim,obsnum)

    obstacles=graph.makeobs()
    map.drawMap(obstacles)

    t1=time.time()
    while (not graph.path_to_goal()):
        time.sleep(0.005)
        elapsed=time.time()-t1
        t1=time.time()   
        
        if elapsed > 10:
            print('timeout re-initiating the calculations')
            raise

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1
     
    
    smooth_=[]
    paths=graph.getPathCoords()
    path_all.append(paths)
  
    smooth_= B_spline(paths)
    x_smooth, y_smooth= smooth_
    for x2, y2 in zip (x_smooth, y_smooth):
        
        pygame.draw.circle(map.map, (255,9,0), (x2,y2), 3, 0)

    #map.drawPath(graph.getPathCoords())
    print("CHOSE", graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)



if __name__ == '__main__':
    result=False
    while not result:
        try:
            main()
            print(result)
            result=True
        except:
            result=False



























