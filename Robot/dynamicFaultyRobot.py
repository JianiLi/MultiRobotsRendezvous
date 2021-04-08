from shapely.geometry import Point


class OsciallateFaultyRobot:

    def __init__(self, pos):
        self._pos = pos

    def updatePos(self, step):
        dict = {1: [1, 0], 2: [0, -1], 3: [-1, 0], 0: [0, 1]}
        newPos = Point(self._pos.x + dict[step % 4][0] * 0.1, self._pos.y + dict[step % 4][1] * 0.1)
        self.setPos(newPos)

    def getPos(self):
        return self._pos

    def setPos(self, p):
        self._pos = p


class MoveAwayFaultyRobot:
    def __init__(self, pos):
        self._pos = pos

    def updatePos(self, t):
        move = [0, 0]
        if self._pos.x > 0:
            if self._pos.y > 0:
                move = [1, 1]
            else:
                move = [1, -1]
        else:
            if self._pos.y > 0:
                move = [-1, 1]
            else:
                move = [-1, -1]
        if self._pos.x > 0.9 or self._pos.x < -0.9:
            move[0] = 0
        if self._pos.y > 0.9 or self._pos.y < -0.9:
            move[1] = 0
        newPos = Point(self._pos.x + move[0] * 0.1, self._pos.y + move[1] * 0.1)
        self.setPos(newPos)

    def getPos(self):
        return self._pos

    def setPos(self, p):
        self._pos = p
