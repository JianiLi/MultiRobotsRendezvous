from TverbergPoint.TverbergPoint import *
from centerpoint.Centerpoint import *

class FaultFreeRobot:
    def __init__(self, pos, sensDist, alpha, velMax):
        self.neighbors = []
        self._pos = pos
        self.sensDist = sensDist
        self.alpha = alpha
        self.velMax = velMax

    def updatePos(self, points):
        self.findNeighbors(points)
        self.getNextMove()

    def getPos(self):
        return self._pos

    def setPos(self, p):
        self._pos = p

    def findNeighbors(self, points):
        self.neighbors = []
        for p in points:
            d = math.sqrt((self._pos.x - p.x) ** 2 + (self._pos.y - p.y) ** 2)
            if d <= self.sensDist:
                self.neighbors.append(p)

    def getSafePoint(self):
        Tverp = TverbergPoint()
        sp = Tverp.getSafePoint(self.neighbors)
        return sp

    def getCenterPoint(self):
        centerPoint = Centerpoint(self.neighbors)
        cp = centerPoint.reduce_then_get_centerpoint()
        return cp

    def getNextMove(self):
        sp = self.getSafePoint()
        #sp = self.getCenterPoint()

        u = self.alpha * (math.sqrt((self._pos.x - sp.x) ** 2 + (self._pos.y - sp.y) ** 2))
        if u > self.velMax:
            u = self.velMax
        if sp.x > self._pos.x:
            theta = math.atan((sp.y - self._pos.y) / (sp.x - self._pos.x))
            x = self._pos.x + u * math.cos(theta)
            y = self._pos.y + u * math.sin(theta)
            self.setPos(Point(x, y))

        elif sp.x < self._pos.x:
            theta = math.atan((sp.y - self._pos.y) / (sp.x - self._pos.x)) - math.pi
            x = self._pos.x + u * math.cos(theta)
            y = self._pos.y + u * math.sin(theta)
            self.setPos(Point(x, y))

        else:
            if sp.y >= self._pos.y:
                y = self._pos.y + u
            else:
                y = self._pos.y + u
            self.setPos(Point(self._pos.x, y))



class FaultyRobot:
    def __init__(self, pos):
        self.pos = pos

    def getPos(self):
        return self.pos
