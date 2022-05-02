
class BresenhamCircle(set):
    def __init__(self, radius, x0=0, y0=0):
        super(BresenhamCircle, self).__init__()
        points = self.__calc_points(radius, x0, y0)
        self.update(points)

    def tolist(self):
        return list(self)

    def __calc_points(self, radius, x0, y0):
        switch = 3 - 2 * radius
        points = []
        x = 0
        y = radius
        while x <= y:
            self.__draw_line(points, -y + x0, x + y0, y + x0, x + y0)
            self.__draw_line(points, -x + x0, y + y0, x + x0, y + y0)
            self.__draw_line(points, -x + x0, -y + y0, x + x0, -y + y0)
            self.__draw_line(points, -y + x0, -x + y0, y + x0, -x + y0)
            if switch < 0:
                switch = switch + 4 * x + 6
            else:
                switch = switch + 4 * (x - y) + 10
                y = y - 1
            x = x + 1

        return points

    def __draw_line(self, points, x0, y0, x1, y1):
        if x1 == x0 and y1 == y0:
            points.append((x1, y1))
            return

        dx = x1 - x0
        dy = y1 - y0

        xsign = 1 if dx > 0 else -1
        ysign = 1 if dy > 0 else -1

        dx = int(abs(dx))
        dy = int(abs(dy))

        if dx > dy:
            xx, xy, yx, yy = xsign, 0, 0, ysign
        else:
            dx, dy = dy, dx
            xx, xy, yx, yy = 0, ysign, xsign, 0

        D = 2*dy - dx
        y = 0

        for x in range(dx + 1):
            points.append((x0 + x*xx + y*yx, y0 + x*xy + y*yy))
            if D >= 0:
                y += 1
                D -= 2*dx
            D += 2*dy
