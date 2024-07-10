import cvxpy as cp
import numpy as np


class LinearMPC:
    def __init__(self):
        K = 5
        nx = 4
        self.nu = 4

        A = np.eye(nx)
        B = np.eye(self.nu)

        Q = 1
        R = 0.01
        S = 3

        self.x = cp.Variable((nx, K + 1))    # x, y, z, pitch
        self.u = cp.Variable((self.nu, K))        # dx, dy, dz, dpitch

        self.x0 = cp.Parameter(nx)
        self.xf = cp.Parameter(nx)
        self.uf = cp.Parameter(self.nu)

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

    def step(self, x0, x_ss, u_ss):
        self.x0.value = x0
        self.xf.value = x_ss
        self.uf.value = u_ss

        self.prob.solve(solver=cp.ECOS)

        u_out = self.u.value

        if u_out is not None:
            return u_out[:, 0]
        else:
            print("Infeasible")
            return np.zeros(self.nu)


if __name__ == "__main__":
    t_mpc = LinearMPC()

    x0 = np.array([0., 4.5, 0, 0])
    xf = np.array([3., 0, 0, 0.3])
    uf = np.array([0., 0, 0, 0])

    u = t_mpc.step(x0, xf, uf)
    print(u)
