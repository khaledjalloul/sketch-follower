import numpy as np
from kinematics import Kinematics


class Dynamics(Kinematics):
    def __init__(self):
        super().__init__()
        x = 0.5
        y = 2
        z = 0.1

        self.m = 0.3

        self.inertia = (1/12) * self.m * np.array([[z ** 2 + y ** 2, 0, 0],
                                                   [0, x ** 2 + z ** 2, 0],
                                                   [0, 0, y ** 2 + x ** 2]])

        self.n = np.array([0, 0, 1])

        self.prev_J_p1 = np.zeros((3, 3))
        self.prev_J_r1 = np.zeros((3, 3))
        self.prev_J_p2 = np.zeros((3, 3))
        self.prev_J_r2 = np.zeros((3, 3))
        self.prev_J_p3 = np.zeros((3, 3))
        self.prev_J_r3 = np.zeros((3, 3))

    def get_dynamics(self, q, dq):
        r = np.array([0, 0.75, 0, 1])

        Ir_IS1 = (self.T_I0(q) @ r)[0:3]
        J_p1 = np.c_[np.cross(self.n, Ir_IS1),
                     np.zeros((3)),
                     np.zeros((3))]
        J_r1 = np.c_[self.n,
                     np.zeros(3),
                     np.zeros(3)]
        dJ_p1 = (J_p1 - self.prev_J_p1) / 0.1
        dJ_r1 = (J_r1 - self.prev_J_r1) / 0.1
        self.prev_J_p1 = J_p1
        self.prev_J_r1 = J_r1
        M1, b1, g1 = self.get_link_dynamics(q, dq, J_p1, J_r1, dJ_p1, dJ_r1)

        T_I1 = self.T_I0(q) @ self.T_01(q)
        Ir_IS2 = (T_I1 @ r)[0:3]
        J_p2 = np.c_[np.cross(self.n, Ir_IS2),
                     np.cross(self.n, (T_I1[0:3, 0:3] @ r[0:3])),
                     np.zeros(3)]
        J_r2 = np.c_[self.n,
                     self.n,
                     np.zeros(3)]
        dJ_p2 = (J_p2 - self.prev_J_p2) / 0.1
        dJ_r2 = (J_r2 - self.prev_J_r2) / 0.1
        self.prev_J_p2 = J_p2
        self.prev_J_r2 = J_r2
        M2, b2, g2 = self.get_link_dynamics(q, dq, J_p2, J_r2, dJ_p2, dJ_r2)

        T_I2 = T_I1 @ self.T_12(q)
        Ir_IS3 = (T_I2 @ r)[0:3]
        J_p3 = np.c_[np.cross(self.n, Ir_IS3),
                     np.cross(self.n,
                              (T_I1[0:3, 0:3] @ (self.T_12(q) @ r)[0:3])),
                     np.cross(self.n, (T_I2[0:3, 0:3] @ r[0:3]))]
        J_r3 = np.c_[self.n,
                     self.n,
                     self.n]
        dJ_p3 = (J_p3 - self.prev_J_p3) / 0.1
        dJ_r3 = (J_r3 - self.prev_J_r3) / 0.1
        self.prev_J_p3 = J_p3
        self.prev_J_r3 = J_r3
        M3, b3, g3 = self.get_link_dynamics(q, dq, J_p3, J_r3, dJ_p3, dJ_r3)

        M = M1 + M2 + M3
        b = b1 + b2 + b3
        g = g1 + g2 + g3

        return M, b, g

    def get_link_dynamics(self, q, dq, J_p, J_r, dJ_p, dJ_r):
        M = self.m * J_p.T @ J_p + J_r.T @ self.inertia @ J_r

        omega = J_r @ dq
        b = self.m * J_p.T @ dJ_p @ dq + \
            J_r.T @ (self.inertia @ dJ_r @ dq +
                     np.cross(omega, self.inertia @ omega))

        g = - J_p.T @ np.array([0, 0, -9.81 * self.m])

        return M, b, g

    def forward_dyn(self, q, dq, ddq):
        M, b, g = self.get_dynamics(q, dq)

        tau = (M @ ddq).reshape(-1) + b + g

        return tau


if __name__ == "__main__":
    dyn = Dynamics()

    q = np.array([0.5, -0.5, 0.7])
    dq = np.array([0, -0, 0])
    ddq = np.array([0.8, 0.9, 1.3])

    print(dyn.forward_dyn(q, dq, ddq))
