import cvxpy as cp
import numpy as np


class JointSpaceLinearMPC:
    def __init__(self):
        K = 5

        A = np.array([[1, 0, 0, 1, 0, 0],
                      [0, 1, 0, 0, 1, 0],
                      [0, 0, 1, 0, 0, 1],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])

        B = np.array([[0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        Q = 1
        R = 0.1
        S = 1

        self.x = cp.Variable((6, K + 1))    # q1, q2, q3, dq1, dq2, dq3
        self.u = cp.Variable((3, K))        # ddq1, ddq2, ddq3

        self.x0 = cp.Parameter(6)
        self.xf = cp.Parameter(6)
        self.uf = cp.Parameter(3)

        cost = S * cp.sum_squares(self.x[:, K])
        constraints = [
            self.x[:, 0] == self.x0,
            self.x[:, K] == self.xf,
            self.u[:, K - 1] == self.uf
        ]

        for k in range(K):
            cost += Q * cp.sum_squares(self.x[:, k]) + \
                R * cp.sum_squares(self.u[:, k]) + \
                S * cp.sum_squares(self.x[:, K])

            constraints += [self.x[:, k + 1] ==
                            A @ self.x[:, k] + B @ self.u[:, k],
                            cp.norm(self.x[:, k + 1], "inf") <= np.pi,
                            cp.norm(self.u[:, k], "inf") <= np.pi / 2
                            ]

        self.prob = cp.Problem(cp.Minimize(cost), constraints)

    def step(self, x0, xf, uf):
        self.x0.value = x0
        self.xf.value = xf
        self.uf.value = uf

        self.prob.solve(solver=cp.ECOS)

        return self.u.value, self.x.value


class TaskSpaceLinearMPC:
    def __init__(self):
        K = 10

        A = np.array([[1, 0, 1, 0],
                      [0, 1, 0, 1],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        B = np.array([[0, 0],
                      [0, 0],
                      [1, 0],
                      [0, 1],])

        Q = 1
        R = 0.1
        S = 1

        self.x = cp.Variable((4, K + 1))    # x, y, dx, dy
        self.u = cp.Variable((2, K))        # ddx, ddy

        self.x0 = cp.Parameter(4)
        self.xf = cp.Parameter(4)
        self.uf = cp.Parameter(2)

        cost = S * cp.sum_squares(self.x[:, K])
        constraints = [
            self.x[:, 0] == self.x0,
            self.x[:, K] == self.xf,
            self.u[:, K - 1] == self.uf
        ]

        for k in range(K):
            cost += Q * cp.sum_squares(self.x[:, k]) + \
                R * cp.sum_squares(self.u[:, k]) + \
                S * cp.sum_squares(self.x[:, K])

            constraints += [self.x[:, k + 1] == A @
                            self.x[:, k] + B @ self.u[:, k],
                            cp.norm(self.x[:, k + 1], 2) <= 4.6,
                            cp.norm(self.u[:, k], "inf") <= 1.5
                            ]

        self.prob = cp.Problem(cp.Minimize(cost), constraints)

    def step(self, x0, xf, uf):
        self.x0.value = x0
        self.xf.value = xf
        self.uf.value = uf

        self.prob.solve(solver=cp.ECOS)

        return self.u.value, self.x.value


if __name__ == "__main__":
    j_mpc = JointSpaceLinearMPC()

    x0 = np.array([1., 1, 1, 0, 0, 0])
    xf = np.array([1., 2, 3, 0, 0, 0])
    uf = np.array([0., 0, 0])

    u, x = j_mpc.step(x0, xf, uf)
    print(x[:, -1])

    t_mpc = TaskSpaceLinearMPC()

    x0 = np.array([0., 4.5, 0, 0])
    xf = np.array([3., 0, 0, 0])
    uf = np.array([0., 0])

    u, x = t_mpc.step(x0, xf, uf)
    print(x[:, -1])
