
class PIDController(object):
    def __init__(self, controller_params):
        self.p = controller_params.get('p', 0.0)
        self.i = controller_params.get('i', 0.0)
        self.d = controller_params.get('d', 0.0)

        self.last_error = 0
        self.integrate_error = 0

        self.max_speed = controller_params.get('max_val', 100.0)
        self.max_integrator = controller_params.get('max_integrator', 100.0)

    def execute(self, error):
        d_error = error - self.last_error
        self.integrate_error += error
        self.last_error = error

        self.integrate_error = self.__bound(self.integrate_error, self.max_integrator)

        steer = self.p * error + self.i * self.integrate_error + self.d * d_error

        steer = self.__bound(steer, self.max_speed)
        return steer

    def reset(self):
        self.last_error = 0
        self.integrate_error = 0

    def __bound(self, val, bound_val):
        return min(bound_val, max(-bound_val, val))