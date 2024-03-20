import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import do_mpc

model = do_mpc.model.Model("discrete")

x = model.set_variable(var_type='_x', var_name='x', shape=(2, 1))
u = model.set_variable(var_type='_u', var_name='u', shape=(2, 1))

A = np.array([[1, 0], [0, 1]])
B = np.array([[1, 0], [0, 1]])
Q = np.array([[1, 0], [0, 1]])
R = np.array([[1, 0], [0, 1]])

model.set_rhs('x', A @ x + B @ u)

model.setup()

mpc = do_mpc.controller.MPC(model)
setup = {
    'n_horizon': 20,
    't_step': 0.1,
    'store_full_solution': True,
    "nlpsol_opts": {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
}
mpc.set_param(**setup)

mpc.set_objective(
    lterm=x.T @ Q @ x,
    mterm=x.T @ Q @ x,
)
mpc.set_rterm(
    rterm=u.T @ R @ u
)

mpc.bounds['upper', '_x', 'x'] = 5
mpc.bounds['lower', '_x', 'x'] = -5
mpc.bounds['upper', '_u', 'u'] = 10
mpc.bounds['lower', '_u', 'u'] = -10

mpc.setup()

x0 = np.array([[3, -3]]).T
u0 = np.array([0, 0])

mpc.x0 = x0
mpc.set_initial_guess()

for i in range(20):
    u0 = mpc.make_step(x0)
    x0 = A @ x0 + B @ u0

    print(x0, u0)
    print("------------")
