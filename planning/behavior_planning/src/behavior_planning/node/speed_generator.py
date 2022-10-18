class SpeedGenerator:
    def __init__(self, max_speed, axel_len, des_len) -> None:
        self.max_speed = max_speed
        self.axel_len = axel_len
        self.des_len = des_len

    def fill_speeds(self, route, line_ends=False) -> None:
        for point in route.route:
            point.speed = self.max_speed

        delta_ax = self.max_speed / self.axel_len
        for i in range(0, self.axel_len):
            route.route[i].speed = min(i * delta_ax, route.route[i].speed)

        if line_ends:
            delta_des = self.max_speed / self.des_len
            for i in range(1, self.des_len + 1):
                route.route[-i].speed = min(i * delta_des, route.route[-i].speed)
