from TverbergPoint.TverbergPoint import *
from centerpoint.Centerpoint import *



class DynamicFaultyRobot:
    def __init__(self, pos):
        self._pos = pos

    def getPos(self):
        return self._pos
