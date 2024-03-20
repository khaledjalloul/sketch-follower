import numpy as np


class Kinematics:
    def __init__(self):
        self.prev_J = np.zeros((6, 2))

    def T_I0(self, q):
        return np.array([[np.cos(q[0]),    -np.sin(q[0]),  0,  0],
                         [np.sin(q[0]),    np.cos(q[0]),   0,  0],
                         [0,               0,              1,  0],
                         [0,               0,              0,  1]])

    def T_01(self, q):
        return np.array([[np.cos(q[1]),    -np.sin(q[1]),  0,  0],
                         [np.sin(q[1]),    np.cos(q[1]),   0,  1.5],
                         [0,               0,              1,  0],
                         [0,               0,              0,  1]])

    def T_1E(self, q):
        return np.array([[1,               0,              0, 0],
                         [0,               1,              0, 1.5],
                         [0,               0,              1, 0],
                         [0,               0,              0, 1]])

    def p(self, q):
        return (self.T_I0(q) @ self.T_01(q) @ self.T_1E(q))[0:3, 3]

    def J(self, q):
        n = np.array([0, 0, 1])

        Ir_IE = self.p(q)

        T_I1 = self.T_I0(q) @ self.T_01(q)
        Ir_1E = self.T_1E(q)[0:3, 3]

        J = np.r_[np.c_[np.cross(n, Ir_IE),   np.cross(n, Ir_1E)],
                  np.c_[n,                    n]]

        return J

    def w(self, q, dq):
        return self.J(q) @ dq

    def dJ(self, q):

        J = self.J(q)
        dJ = (J - self.prev_J) / 0.1
        self.prev_J = J

        return dJ

    def inv_kin_q(self, p_desired, q_initial) -> np.ndarray:
        p_current = self.p(q_initial)
        err = np.linalg.norm(p_desired - p_current)
        i = 0
        q = np.array(q_initial, dtype=np.float64)

        while err > 0.1 and i < 20:
            p_current = self.p(q)
            J_p = self.J(q)[0:3]

            diff = (p_desired - p_current.reshape(-1))
            err = np.linalg.norm(diff)
            dq = (np.linalg.pinv(J_p) @ diff)

            q += dq
            i += 1

        q = q % (2 * np.pi)
        q = np.where(q > np.pi, q - 2 * np.pi, q)

        return q

    def inv_kin_dq(self, w_desired, q) -> np.ndarray:
        J_p = self.J(q)[0:3]
        dq = (np.linalg.pinv(J_p) @ w_desired)

        return dq

    def inv_kin_ddq(self, dw_desired, q, dq) -> np.ndarray:
        J_p = self.J(q)[0:3]
        dJ_p = self.dJ(q)[0:3]

        ddq = np.linalg.pinv(J_p) @ (dw_desired - dJ_p @ dq)

        return ddq


if __name__ == "__main__":

    kin = Kinematics()

    p_desired = np.array([-2., 1, 0])
    w_desired = np.array([1., 0, 0])
    dw_desired = np.array([0.3, 0.5, 0])
    q = [-2, 1]
    dq = [0, 0]

    print(kin.inv_kin_q(p_desired, q))
    print(kin.inv_kin_dq(w_desired, q))
    print(kin.inv_kin_ddq(dw_desired, q, dq))
