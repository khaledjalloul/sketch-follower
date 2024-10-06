class PID:
    def __init__(self):
        self.kp = 2

    def step(self, x0, x_ss, u_ss):
        out = self.kp * (x_ss - x0)
        return out
