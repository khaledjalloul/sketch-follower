import numpy as np
import sympy as sp
from kinematics import Kinematics


class Dynamics(Kinematics):
    def __init__(self):
        x = 0.5
        y = 2
        z = 0.1

        self.m = 0.3

        self.inertia = (1/12) * self.m * sp.Matrix([[z ** 2 + y ** 2, 0, 0],
                                                    [0, x ** 2 + z ** 2, 0],
                                                    [0, 0, y ** 2 + x ** 2]])

        self.n = sp.Matrix([0, 0, 1])

    def get_dynamics(self, t, q, dq):
        r = sp.Matrix([0, 0.75, 0, 1])

        Ir_IS1 = (self.T_I0(q) @ r)[0:3, :]
        J_p1 = sp.Matrix([self.n.cross(Ir_IS1).T,
                          sp.zeros(1, 3),
                          sp.zeros(1, 3)]).T
        J_r1 = sp.Matrix([self.n.T,
                          sp.zeros(1, 3),
                          sp.zeros(1, 3)]).T
        M1, b1, g1 = self.get_link_dynamics(t, q, dq, J_p1, J_r1)

        T_I1 = self.T_I0(q) @ self.T_01(q)
        Ir_IS2 = (T_I1 @ r)[0:3, :]
        J_p2 = sp.Matrix([self.n.cross(Ir_IS2).T,
                          self.n.cross((T_I1[0:3, 0:3] @ r[0:3, :])).T,
                          sp.zeros(1, 3)]).T
        J_r2 = sp.Matrix([self.n.T,
                          self.n.T,
                          sp.zeros(1, 3)]).T
        M2, b2, g2 = self.get_link_dynamics(t, q, dq, J_p2, J_r2)

        T_I2 = T_I1 @ self.T_12(q)
        Ir_IS3 = (T_I2 @ r)[0:3, :]
        J_p3 = sp.Matrix([self.n.cross(Ir_IS3).T,
                          self.n.cross(
                              (T_I1[0:3, 0:3] @ (self.T_12(q) @ r)[0:3, :])).T,
                          self.n.cross((T_I2[0:3, 0:3] @ r[0:3, :])).T]).T
        J_r3 = sp.Matrix([self.n.T,
                          self.n.T,
                          self.n.T]).T
        M3, b3, g3 = self.get_link_dynamics(t, q, dq, J_p3, J_r3)

        M = M1 + M2 + M3
        b = b1 + b2 + b3
        g = g1 + g2 + g3

        return M, b, g

    def get_link_dynamics(self, t, q, dq, J_p, J_r):

        dJ_p = sp.Derivative(J_p).doit()
        dJ_r = sp.Derivative(J_r, t).doit()

        if dJ_r == 0:
            dJ_r = sp.Matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

        M = self.m * J_p.T @ J_p + J_r.T @ self.inertia @ J_r

        omega = J_r @ dq
        b = self.m * J_p.T @ dJ_p @ dq + \
            J_r.T @ (self.inertia @ dJ_r @ dq +
                     omega.cross(self.inertia @ omega))

        g = - J_p.T @ sp.Matrix([0, 0, -9.81 * self.m])

        return M, b, g

    def forward_dyn(self, q_val, dq_val, ddq):

        # print("hi")
        t = sp.symbols('t')
        q1, q2, q3 = sp.symbols('q1 q2 q3', cls=sp.Function)

        q = sp.Matrix([q1(t), q2(t), q3(t)])
        dq = sp.Derivative(q).doit()

        M, b, g = self.get_dynamics(t, q, dq)

        tau = M @ ddq.reshape(3, 1) + b + g

        tau_func = sp.lambdify(
            (q1(t), q2(t), q3(t),
             sp.Derivative(q1(t)),
             sp.Derivative(q2(t)),
             sp.Derivative(q3(t))), tau, "numpy")
        
        tau_val = tau_func(*q_val, *dq_val)
        
        # tau_val = tau.subs([
        #     *[(dq[i], dq_val[i]) for i in range(len(q_val))],
        #     *[(q[i], q_val[i]) for i in range(len(q_val))]
        # ])
        
        return np.array(tau_val).astype(np.float64)


if __name__ == "__main__":
    dyn = Dynamics()

    q = np.array([0.5, -0.5, 0.7])
    dq = np.array([0.5, -0.5, 0])
    ddq = np.array([0.8, 0.9, 1.3])

    print(np.array(dyn.forward_dyn(q, dq, ddq)).astype(np.float64))
