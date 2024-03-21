import cvxpy as cp
import numpy as np


class LinearMPC:
    def __init__(self):
        K = 5

        A = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        B = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        Q = 10
        R = 10
        S = 10

        self.x = cp.Variable((4, K + 1))    # x, y, z, pitch
        self.u = cp.Variable((4, K))        # dx, dy, dz, dpitch

        self.x0 = cp.Parameter(4)
        self.xf = cp.Parameter(4)
        self.uf = cp.Parameter(4)

        cost = S * cp.sum_squares(self.x[:, K])
        constraints = [
            self.x[:, 0] == self.x0,
            self.x[:, K] == self.xf,
            self.u[:, K - 1] == self.uf
        ]

        for k in range(K):
            cost += Q * cp.sum_squares(self.x[:, k] - self.xf) + \
                R * cp.sum_squares(self.u[:, k])

            constraints += [self.x[:, k + 1] == A @
                            self.x[:, k] + B @ self.u[:, k],
                            cp.norm(self.x[:, k + 1], 2) <= 4.5
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

    x0 = np.array([0., 4.5, 0, 0])
    xf = np.array([3., 0, 0, 0.3])
    uf = np.array([0., 0, 0, 0])

    u, x = t_mpc.step(x0, xf, uf)
    print(x[:, -1])
