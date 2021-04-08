from TverbergPoint.TverbergPoint import *
from centerpoint.Centerpoint import *


class FaultFreeRobot:
    def __init__(self, pos, sensDist, alpha, velMax, delta, fixNeighbors=False):
        self.neighbors = []
        self._pos = pos
        self.sensDist = sensDist
        self.alpha = alpha
        self.velMax = velMax
        self.delta = delta
        self.stop = False
        self.fixNeighbors = fixNeighbors
        self.neighborsPos = []

    def updatePos(self, points, method="tver"):
        self.stop = False
        if self.neighbors == [] or not self.fixNeighbors:
            self.findNeighbors(points)
        self.neighborsPos = [points[i] for i in self.neighbors]
        self.getNextMove(method=method)

    def getPos(self):
        return self._pos

    def setPos(self, p):
        self._pos = p

    def findNeighbors(self, points):
        self.neighbors = []
        for i in range(len(points)):
            p = points[i]
            d = math.sqrt((self._pos.x - p.x) ** 2 + (self._pos.y - p.y) ** 2)
            if d <= self.sensDist:
                self.neighbors.append(i)
        self.neighborsPos = [points[i] for i in self.neighbors]

    def getTvbSafePoint(self):
        Tverp = TverbergPoint()
        tvp = Tverp.getTvbPoint(self.neighborsPos)
        sp = Tverp.getSafePoint(self.neighborsPos)
        return sp

    def getCenterSafePoint(self):
        centerPoint = Centerpoint()
        try:
            # cp = centerPoint.getSafeCenterPoint(self.neighborsPos)
            cp = centerPoint.reduce_then_get_centerpoint(self.neighborsPos)
            if cp:
                return cp
            else:
                return self.neighborsPos
        except:
            return self.neighborsPos
        return cp

    def getMedian(self):
        median_x = np.median([p.x for p in self.neighborsPos])
        median_y = np.median([p.y for p in self.neighborsPos])
        return Point(median_x, median_y)


    def getNextMove(self, method):
        if method == "tver":
            sp = self.getTvbSafePoint()
        elif method == "center":
            sp = self.getCenterSafePoint()
        elif method == "median":
            sp = self.getMedian()
        else:
            raise Exception("No such method, please enter either 'tver' or 'center'.")

        u = self.alpha * (math.sqrt((self._pos.x - sp.x) ** 2 + (self._pos.y - sp.y) ** 2))
        if u > self.velMax:
            u = self.velMax

        if abs(u) < self.delta:
            self.stop = True
        else:
            if sp.x > self._pos.x:
                theta = math.atan((sp.y - self._pos.y) / (sp.x - self._pos.x))
                x = self._pos.x + u * math.cos(theta)
                y = self._pos.y + u * math.sin(theta)
                newPos = Point(x, y)
            elif sp.x < self._pos.x:
                theta = math.atan((sp.y - self._pos.y) / (sp.x - self._pos.x)) - math.pi
                x = self._pos.x + u * math.cos(theta)
                y = self._pos.y + u * math.sin(theta)
                newPos = Point(x, y)
            else:
                if sp.y >= self._pos.y:
                    y = self._pos.y + u
                else:
                    y = self._pos.y + u
                newPos = Point(self._pos.x, y)
            self.setPos(newPos)
