import numpy as np
from typing import Tuple
import cvxpy as cp
import do_mpc
import casadi as ca
import sympy as sp


class Kinematics:
    def __init__(self, q_init=None):
        if q_init is not None:
            self.q = q_init
        else:
            self.q = np.array([0., 0., 0.])

        self.n = np.array([0., 0., 1.])

    def get_robot_state(self, q=None) -> Tuple[np.ndarray, np.ndarray]:
        if q is None:
            q = self.q

        T_I0 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0,  0],
                        [np.sin(q[0]),  np.cos(q[0]),  0,  0],
                        [0,             0,             1,  0],
                        [0,             0,             0,  1]])

        T_01 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0,  0],
                        [np.sin(q[1]),  np.cos(q[1]),  0,  1.5],
                        [0,             0,             1,  0],
                        [0,             0,             0,  1]])

        T_12 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0,  0],
                        [np.sin(q[2]),  np.cos(q[2]),  0,  1.5],
                        [0,             0,             1,  0],
                        [0,             0,             0,  1]])

        T_2E = np.array([[1,            0,             0,  0],
                        [0,             1,             0,  1.5],
                        [0,             0,             1,  0],
                        [0,             0,             0,  1]])

        T_I1 = T_I0 @ T_01
        T_I2 = T_I1 @ T_12
        T_IE = T_I2 @ T_2E

        T_1E = T_12 @ T_2E

        Ir_IE = T_IE[0:3, 3]
        Ir_1E = T_I1[0:3, 0:3] @ T_1E[0:3, 3]
        Ir_2E = T_I2[0:3, 0:3] @ T_2E[0:3, 3]

        J = np.r_[np.c_[np.cross(self.n, Ir_IE),    np.cross(self.n, Ir_1E), np.cross(self.n, Ir_2E)],
                  np.c_[self.n,                     self.n,                  self.n]]

        return Ir_IE, J

    def inv_kin(self, p_desired: np.ndarray) -> np.ndarray:
        p_current, _ = self.get_robot_state()
        err = np.linalg.norm(p_desired - p_current)
        i = 0

        while err > 0.1 and i < 100:
            p_current, J = self.get_robot_state()
            p_J = J[0:3]

            diff = (p_desired - p_current)
            err = np.linalg.norm(diff)
            qd = (np.linalg.pinv(p_J) @ diff)

            self.q += qd
            i += 1

        self.q = self.q % (2 * np.pi)
        self.q = np.where(self.q > np.pi, self.q - 2 * np.pi, self.q)

        return self.q

    def inv_kin_qp(self, p_desired: np.ndarray) -> np.ndarray:
        dq = cp.Variable(3)
        J_param = cp.Parameter((3, 3))
        diff_param = cp.Parameter(3)
        # q_param = cp.Parameter(3)

        constraints = []

        prob = cp.Problem(cp.Minimize(cp.sum_squares(
            J_param @ dq - diff_param)), constraints)

        q_param = self.q
        p_current, _ = self.get_robot_state()
        err = np.linalg.norm(p_desired - p_current)
        i = 0

        while err > 0.1 and i < 100:
            p_current, J = self.get_robot_state()
            J_param.value = J[0:3]

            diff_param.value = (p_desired - p_current)
            err = np.linalg.norm(diff_param.value)
            prob.solve(warm_start=True, solver='ECOS_BB')

            self.q += dq.value

            i += 1

        self.q = self.q % (2 * np.pi)
        self.q = np.where(self.q > np.pi, self.q - 2 * np.pi, self.q)

        return self.q


if __name__ == "__main__":

    kin = Kinematics()
    p_desired = np.array([-2., 1, 0])

    print(kin.inv_kin(p_desired))
    print(kin.inv_kin_qp(p_desired))
