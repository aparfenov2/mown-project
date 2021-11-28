from threading import RLock


class Frame(object):
    TASK_TYPE_INIT = "init_type"
    TASK_TYPE_PLAN_TO_GOAL = 'plan_to_goal_type'
    TASK_TYPE_COV_PLAN = 'cov_plan_type'

    def __init__(self):
        self._task_plan_poly = None
        self._task_plan_to_goal = None
        self._current_task = None
        self._current_task_type = self.TASK_TYPE_INIT

        self._grid_map_scale = 0.1
        self._grid_map_start_point = (0.0, 0.0)
        self._grid_map_obstacles = set()

        self._robot_radius = 0.1

        self._rlock = RLock()

        self._localization = None

    def lock(self):
        return self._rlock

    def receive_task_polygon_planning(self, message):
        with self._rlock:
            self._task_plan_poly = message
            self._current_task_type = self.TASK_TYPE_COV_PLAN

    def receive_task_to_point_planning(self, message):
        with self._rlock:
            self._task_plan_to_goal = message
            self._current_task_type = self.TASK_TYPE_PLAN_TO_GOAL

    def receive_localization(self, message):
        with self._rlock:
            self._localization = message

    def receive_occupancy_grid_map(self, message):
        with self._rlock:
            width = message.info.width
            height = message.info.height
            self._grid_map_scale = message.info.resolution

            start_pos = message.info.origin.position
            self._grid_map_start_point = (start_pos.x, start_pos.y)

            # for x in range(width):
            #     for y in range(height):
            #         if message.data[x + width * y] == 1:
            #             pass

            obstacles = set()
        
            for y in range(height):
                for x in range(width):
                    if message.data[y + width * x] > 0:
                        obstacles.add((y, x))

                        not_safe_zone = BresenhamCircle(
                            int(self._robot_radius / self._grid_map_scale),
                            y,
                            x
                        )
                        obstacles = obstacles | not_safe_zone
            
            self._grid_map_obstacles = obstacles

    def get_localization(self):
        return self._localization

    def get_grid_map_obstacles(self):
        return self._grid_map_obstacles

    def get_grid_map_scale(self):
        return self._grid_map_scale

    def get_grid_map_start_pos(self):
        return self._grid_map_start_point

    def get_current_task_type(self):
        return self._current_task_type

    def get_coverage_task(self):
        return self._task_plan_poly

    def get_path_task(self):
        return self._task_plan_to_goal


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
