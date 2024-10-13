import numpy as np
import cvxpy as cp


class Kinematics:
    def __init__(self):
        self.prev_J = np.zeros((6, 4))
        self.dt = 0.1

    def T_I0(self, q):  # base -> base_arm_slot
        return np.array([[np.cos(q[0]),   -np.sin(q[0]),  0,              0],
                         [np.sin(q[0]),   np.cos(q[0]),   0,              0],
                         [0,              0,              1,              0.6],
                         [0,              0,              0,              1]])

    def T_01(self, q):  # base_arm_slot -> arm_link_1
        return np.array([[np.cos(q[1]),   0,              np.sin(q[1]),   0],
                         [0,              1,              0,              0],
                         [-np.sin(q[1]),  0,              np.cos(q[1]),   0.8],
                         [0,              0,              0,              1]])

    def T_12(self, q):  # arm_link_1 -> arm_link_2
        return np.array([[np.cos(q[2]),   0,              np.sin(q[2]),   0],
                         [0,              1,              0,              0],
                         [-np.sin(q[2]),  0,              np.cos(q[2]),   2],
                         [0,              0,              0,              1]])

    def T_23(self, q):  # arm_link_2 -> end_effector_link
        return np.array([[np.cos(q[3]),   0,              np.sin(q[3]),   0],
                         [0,              1,              0,              0],
                         [-np.sin(q[3]),  0,              np.cos(q[3]),   2],
                         [0,              0,              0,              1]])

    def T_3E(self, q):  # end_effector_link -> end_effector
        return np.array([[1,              0,              0,              0.6],
                         [0,              1,              0,              0],
                         [0,              0,              1,              -0.7],
                         [0,              0,              0,              1]])

    def p(self, q):
        return (self.T_I0(q) @ self.T_01(q) @ self.T_12(q) @ self.T_23(q) @ self.T_3E(q))

    def J(self, q):
        T_3E = self.T_3E(q)
        T_2E = self.T_23(q) @ T_3E
        T_1E = self.T_12(q) @ T_2E
        T_0E = self.T_01(q) @ T_1E

        R_I0 = self.T_I0(q)[0:3, 0:3]
        R_I1 = R_I0 @ self.T_01(q)[0:3, 0:3]
        R_I2 = R_I1 @ self.T_12(q)[0:3, 0:3]
        R_I3 = R_I2 @ self.T_23(q)[0:3, 0:3]

        I_r0E = R_I0 @ T_0E[0:3, 3]
        I_r1E = R_I1 @ T_1E[0:3, 3]
        I_r2E = R_I2 @ T_2E[0:3, 3]
        I_r3E = R_I3 @ T_3E[0:3, 3]

        n0 = np.array([0, 0, 1])
        n1 = R_I1 @ np.array([0, 1, 0])
        n2 = R_I2 @ np.array([0, 1, 0])
        n3 = R_I3 @ np.array([0, 1, 0])

        J = np.r_[np.c_[np.cross(n0, I_r0E), np.cross(n1, I_r1E),   np.cross(n2, I_r2E),    np.cross(n3, I_r3E)],
                  np.c_[n0,                  n1,                    n2,                     n3]]

        return J

    def w(self, q, dq):
        return self.J(q) @ dq

    def dJ(self, q):
        J = self.J(q)
        dJ = (J - self.prev_J) / self.dt
        self.prev_J = J

        return dJ

    def rot_matrix_to_vector(self, R):
        theta = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)

        n = np.array([R[2, 1] - R[1, 2],
                      R[0, 2] - R[2, 0],
                      R[1, 0] - R[0, 1]])
        den = np.sin(theta)
        if den == 0:
            den = 0.0001
        n = (1 / (2 * den)) * n

        return theta * n

    def inv_kin_q(self, p_desired, q_initial) -> np.ndarray:

        current_pose = self.p(q_initial)
        current_position = current_pose[0:3, 3]
        current_pitch = self.rot_matrix_to_vector(current_pose[0:3, 0:3])[1]

        diff = p_desired - np.r_[current_position, current_pitch]
        err = np.linalg.norm(diff)

        i = 0
        q = np.array(q_initial, dtype=np.float64)

        while err > 0.1 and i < 20:
            J = self.J(q)[[0, 1, 2, 4], :]

            current_pose = self.p(q)
            current_position = current_pose[0:3, 3]
            current_pitch = self.rot_matrix_to_vector(
                current_pose[0:3, 0:3])[1]

            diff = p_desired - np.r_[current_position, current_pitch]
            err = np.linalg.norm(diff)

            dq = (np.linalg.pinv(J, 0.1) @ diff)

            q += dq
            i += 1

        q = q % (2 * np.pi)
        q = np.where(q > np.pi, q - 2 * np.pi, q)

        return q

    def inv_kin_dq(self, w_desired, q, target=None) -> np.ndarray:
        J = self.J(q)

        if target == "position":
            J = J[0:3]
        elif target == "position_and_pitch":
            J = J[[0, 1, 2, 4], :]

        dq = np.linalg.pinv(J, 0.1) @ w_desired

        return dq

    def inv_kin_ddq(self, dw_desired, q, dq) -> np.ndarray:
        J = self.J(q)[[0, 1, 2, 4], :]
        dJ = self.J(q)[[0, 1, 2, 4], :]

        ddq = np.linalg.pinv(J, 0.1) @ (dw_desired - dJ @ dq)

        return ddq


if __name__ == "__main__":

    kin = Kinematics()

    p_desired = np.array([3., 0, 1, 0])
    w_desired = np.array([1., 0, 0, 0])
    dw_desired = np.array([0.3, 0.5, 0, 0])
    q = [0, 0, 0, 0]
    dq = [0, 0, 0, 0]

    print(kin.inv_kin_q(p_desired, q))
    print(kin.inv_kin_dq(w_desired, q))
    print(kin.inv_kin_ddq(dw_desired, q, dq))
