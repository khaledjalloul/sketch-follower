import numpy as np
import cvxpy as cp


class DeePC:
    def __init__(self):
        self.T_prev = 6
        self.T_fut = 15
        self.L = self.T_prev + self.T_fut
        self.T = 30000

        self.ny = 4
        self.nu = 4

        self.Q = np.eye(self.ny * self.T_fut)
        self.R = np.eye(self.nu * self.T_fut) * 0.1
        self.slack_cost = 1000

        self.y_past = np.zeros((self.ny, self.T_prev))
        self.u_past = np.zeros((self.nu, self.T_prev))

    def construct_hankel_matrices(self, kin, sim):
        u_sim = 80 * np.random.randn(self.nu, self.T)
        y_sim = np.zeros((self.ny, self.T))

        for i in range(self.T):
            u = u_sim[:, i]

            sim.joint_publisher.publish(u)

            current_pose = kin.p(sim.q)
            current_position = current_pose[0:3, 3]
            current_pitch = kin.rot_matrix_to_vector(current_pose[0:3, 0:3])[1]

            y_sim[:, i] = np.r_[current_position, current_pitch]

        self.num_hankel_columns = self.T - self.L + 1

        self.Hu = np.zeros((self.nu * self.L, self.num_hankel_columns))
        self.Hy = np.zeros((self.ny * self.L, self.num_hankel_columns))

        for i in range(self.num_hankel_columns):
            self.Hu[:, i] = u_sim[:, i : i + self.L].T.reshape(-1)
            self.Hy[:, i] = y_sim[:, i : i + self.L].T.reshape(-1)

        print(self.Hu)
        print(self.Hy)

    def step(self, x0, x_ss, u_ss):
        self.y_past = np.append(self.y_past[:, 1:], x0)

        U_p = self.Hu[: self.nu * self.T_prev]
        U_f = self.Hu[-self.nu * self.T_fut :]
        Y_p = self.Hy[: self.ny * self.T_prev]
        Y_f = self.Hy[-self.ny * self.T_fut :]

        u_past = self.u_past.T.reshape(-1)
        y_past = self.y_past.T.reshape(-1)

        x_ss = np.tile(x_ss, self.T_fut)
        u_ss = np.tile(u_ss, self.T_fut)

        g = cp.Variable(self.num_hankel_columns)
        u = cp.Variable(self.nu * self.T_fut)
        y = cp.Variable(self.ny * self.T_fut)
        s = cp.Variable(self.nu * self.T_fut)

        cost = (
            cp.quad_form(y - x_ss, self.Q)
            + cp.quad_form(u - u_ss, self.R)
            + cp.norm1(s)
        )

        constraints = [
            U_p @ g == u_past,
            U_f @ g == u,
            Y_p @ g == y_past,
            Y_f @ g == y,
            cp.norm_inf(u) <= 3,
            s >= 0,
        ]

        prob = cp.Problem(cp.Minimize(cost), constraints)

        result = prob.solve()
        u_out = np.zeros(self.nu)

        if not np.isinf(result):
            u_out = u.value[: self.nu]

        self.u_past = np.append(self.u_past[:, 1:], u_out)
        return u_out
