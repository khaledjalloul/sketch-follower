import numpy as np
import sympy as sp


class Kinematics:
    def T_I0(self, q):
        return sp.Matrix([[sp.cos(q[0]),    -sp.sin(q[0]),  0,  0],
                          [sp.sin(q[0]),    sp.cos(q[0]),   0,  0],
                          [0,               0,              1,  0],
                          [0,               0,              0,  1]])

    def T_01(self, q):
        return sp.Matrix([[sp.cos(q[1]),    -sp.sin(q[1]),  0,  0],
                          [sp.sin(q[1]),    sp.cos(q[1]),   0,  1.5],
                          [0,               0,              1,  0],
                          [0,               0,              0,  1]])

    def T_12(self, q):
        return sp.Matrix([[sp.cos(q[2]),    -sp.sin(q[2]),  0,  0],
                          [sp.sin(q[2]),    sp.cos(q[2]),   0,  1.5],
                          [0,               0,              1,  0],
                          [0,               0,              0,  1]])

    def T_2E(self, q):
        return sp.Matrix([[1,               0,              0, 0],
                          [0,               1,              0, 1.5],
                          [0,               0,              1, 0],
                          [0,               0,              0, 1]])

    def p_sym(self, q):
        return (self.T_I0(q) @ self.T_01(q) @ self.T_12(q) @ self.T_2E(q))[0:3, 3]

    def J_sym(self, q):
        n = sp.Matrix([0, 0, 1])

        Ir_IE = self.p_sym(q)

        T_I1 = self.T_I0(q) @ self.T_01(q)
        T_1E = self.T_12(q) @ self.T_2E(q)
        Ir_1E = T_I1[0:3, 0:3] @ T_1E[0:3, 3]

        T_I2 = T_I1 @ self.T_12(q)
        Ir_2E = T_I2[0:3, 0:3] @ self.T_2E(q)[0:3, 3]

        J = sp.Matrix([sp.Matrix([n.cross(Ir_IE).T,    n.cross(Ir_1E).T,     n.cross(Ir_2E).T]).T,
                       sp.Matrix([n.T,                 n.T,                  n.T]).T])

        return J

    def p(self, q):
        p = self.p_sym(q)
        return np.array(p).astype(np.float64)

    def J(self, q):
        J = self.J_sym(q)
        return np.array(J).astype(np.float64)

    # def p(self, q_val):
    #     q1, q2, q3 = sp.symbols('q1 q2 q3')
    #     q = sp.Matrix([q1, q2, q3])
    #     p = self.p_sym(q)
    #     p_val = p.subs([(q[i], q_val[i]) for i in range(len(q_val))])
    #     return np.array(p_val).astype(np.float64)

    # def J(self, q_val):
    #     q1, q2, q3 = sp.symbols('q1 q2 q3')
    #     q = sp.Matrix([q1, q2, q3])
    #     J = self.J_sym(q)
    #     J_val = J.subs([(q[i], q_val[i]) for i in range(len(q_val))])
    #     return np.array(J_val).astype(np.float64)

    def w(self, q, dq):
        return self.J(q) @ dq

    def dJ(self, q_val, dq_val):
        t = sp.symbols('t')
        q1, q2, q3 = sp.symbols('q1 q2 q3', cls=sp.Function)
        q = sp.Matrix([q1(t), q2(t), q3(t)])

        J = self.J_sym(q)
        dJ = sp.Derivative(J).doit()

        dJ_val = dJ.subs([(sp.Derivative(q[i]), dq_val[i])
                         for i in range(len(dq_val))])
        dJ_val = dJ_val.subs([(q[i], q_val[i]) for i in range(len(q_val))])

        return np.array(dJ_val).astype(np.float64)

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
        dJ_p = self.dJ(q, dq)[0:3]

        ddq = np.linalg.pinv(J_p) @ (dw_desired - dJ_p @ dq)

        return ddq

    # def get_dynamics(self, q, dq):
    #     m = 0.3
    #     x = 0.5
    #     y = 2
    #     z = 0.1

    #     n = np.array([0, 0, 1])

    #     intertia = (1/12) * m * np.array([[z ** 2 + y ** 2, 0, 0],
    #                                       [0, x ** 2 + z ** 2, 0],
    #                                       [0, 0, y ** 2 + x ** 2]])

    #     T_I0 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0,  0],
    #                      [np.sin(q[0]),  np.cos(q[0]),  0,  0],
    #                      [0,             0,             1,  0],
    #                      [0,             0,             0,  1]])

    #     Ir_IS1 = T_I0 @ np.array([0, 0.75, 0])

    #     J1_p = np.c_[np.cross(n, Ir_IS1),
    #                  np.cross(n, Ir_IS1),
    #                  np.cross(n, Ir_IS1)]
    #     J1_r = np.c_[n, n, n]

    #     M1 = m * J1_p.T @ J1_p + J1_r.T @ intertia @ J1_r

    #     # b1 = m * J1_p.T @ self.


if __name__ == "__main__":

    kin = Kinematics()

    p_desired = np.array([-2., 1, 0])
    w_desired = np.array([1., 0, 0])
    dw_desired = np.array([0.3, 0.5, 0])
    q = [-2, 1, 0]
    dq = [0, 0, 0]

    print(kin.inv_kin_q(p_desired, q))
    print(kin.inv_kin_dq(w_desired, q))
    print(kin.inv_kin_ddq(dw_desired, q, dq))
