import pygame

class Robot:
    def __init__(self, startpos, robotImg, width):
        self.m2p=3779.52 #meters 2 pixels
        
        #robot dims       
        self.w=width
        self.x = startpos[0]
        self.y = startpos[1]
        #self.v = startpos[2]
        self.theta=0
        self.vl=0.01*self.m2p #meter/s
        self.vr=0.01*self.m2p
        self.maxspeed = 0.03*self.m2p
        self.minspeed = 0.01*self.m2p
        #graphics
        self.img=pygame.image.load(robotImg)
        self.rotated=self.img
        self.rect = self.rotated.get_rect(center= (self.x, self.y))
     
    def_draw   
        
        