import cvxpy as cp
import numpy as np


class LinearMPC:
    def __init__(self):
        K = 5

        A = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        B = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        Q = 10
        R = 1
        S = 10

        self.x = cp.Variable((3, K + 1))    # x, y, z
        self.u = cp.Variable((3, K))        # dx, dy, dz

        self.x0 = cp.Parameter(3)
        self.xf = cp.Parameter(3)
        self.uf = cp.Parameter(3)

        cost = S * cp.sum_squares(self.x[:, K])
        constraints = [
            self.x[:, 0] == self.x0,
            self.x[:, K] == self.xf,
            self.u[:, K - 1] == self.uf
        ]

        for k in range(K):
            cost += Q * cp.sum_squares(self.x[:, k] - self.xf) + \
                R * cp.sum_squares(self.u[:, k]) + \
                S * cp.sum_squares(self.x[:, K] - self.xf)

            constraints += [self.x[:, k + 1] == A @
                            self.x[:, k] + B @ self.u[:, k],
                            cp.norm(self.x[:, k + 1], 2) <= 3.1,
                            cp.norm(self.u[:, k], "inf") <= 15
                            ]

        self.prob = cp.Problem(cp.Minimize(cost), constraints)

    def step(self, x0, xf, uf):
        self.x0.value = x0
        self.xf.value = xf
        self.uf.value = uf

        self.prob.solve(solver=cp.ECOS)

        return self.u.value, self.x.value


if __name__ == "__main__":
    t_mpc = LinearMPC()

    x0 = np.array([0., 4.5, 0])
    xf = np.array([3., 0, 0])
    uf = np.array([0., 0, 0])

    u, x = t_mpc.step(x0, xf, uf)
    print(x[:, -1])
