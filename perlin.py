import random
import math

class Perlin2D:
    def __init__(self, cellsHigh, cellsWide, seed = 1):
        random.seed(seed)

        self.cellsHigh = cellsHigh
        self.cellsWide = cellsWide
        self.gradients = [
            math.sin(random.random() * math.pi * 2) if i % 2 == 0 else math.cos(random.random() * math.pi * 2)
            for i in range(cellsHigh * cellsWide * 2)]

    def _dot(self, cellX, cellY, vx, vy):
        offset = (cellX + cellY * self.cellsHigh) * 2
        wx = self.gradients[offset]
        wy = self.gradients[offset + 1]
        return wx * vx + wy * vy

    def _lerp(self, a, b, t):
        return a + t * (b - a)

    def _sCurve(self, t):
        return t * t * (3 - 2 * t)

    def getValue(self, x, y):
        xCell = math.floor(x)
        yCell = math.floor(y)
        xFrac = x - xCell
        yFrac = y - yCell

        x0 = xCell
        y0 = yCell
        x1 = 0 if xCell == self.cellsHigh - 1 else xCell + 1
        y1 = 0 if yCell == self.cellsWide - 1 else yCell + 1

        v00 = self._dot(x0, y0, xFrac, yFrac)
        v10 = self._dot(x1, y0, xFrac - 1, yFrac)
        v01 = self._dot(x0, y1, xFrac, yFrac - 1)
        v11 = self._dot(x1, y1, xFrac - 1, yFrac - 1)

        vx0 = self._lerp(v00, v10, self._sCurve(xFrac))
        vx1 = self._lerp(v01, v11, self._sCurve(xFrac))

        return self._lerp(vx0, vx1, self._sCurve(yFrac))
