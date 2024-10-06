#!/usr/bin/env python3

import numpy as np
import rclpy

from sketch_follower.model.kinematics import Kinematics
from sketch_follower.control.pid import PID
from sketch_follower.control.linear_mpc import LinearMPC
from sketch_follower.control.deepc import DeePC
from sketch_follower.ros.ros_interface import ROSInterface

from std_msgs.msg import Float64

np.set_printoptions(suppress=True)
np.set_printoptions(precision=3)

kin = Kinematics()
sim = ROSInterface()

# controller = PID()
controller = LinearMPC()
# controller = DeePC()

desired_z = 0
desired_pitch = 0

print("Resetting to home position...")

err = sim.q_reset - sim.q

while np.linalg.norm(err) > 0.1:
    err = sim.q_reset - sim.q
    dq = 2 * err

    for i in range(4):
        msg = Float64(data=dq[i])
        sim.joint_publishers[i].publish(msg)

    sim.r.sleep()

print("Position reset.")

# controller.construct_hankel_matrices(kin, sim)

while rclpy.ok():
    sim.r.sleep()

    if sim.desired_position is None:
        for i in range(4):
            sim.joint_publishers[i].publish(0)
        continue

    current_pose = kin.p(sim.q)
    current_position = current_pose[0:3, 3]
    current_pitch = kin.rot_matrix_to_vector(current_pose[0:3, 0:3])[1]

    x0 = np.r_[current_position, current_pitch]
    x_ss = np.r_[sim.desired_position, desired_z, desired_pitch]
    u_ss = [0, 0, 0, 0]

    w_desired = controller.step(x0, x_ss, u_ss)

    dq = kin.inv_kin_dq(w_desired, sim.q)
    # dq = kin.inv_kin_dq_pos(w_desired[0:3], sim.q)

    # print(
    #     f"Target: {np.r_[sim.desired_position, desired_z, desired_pitch]}\nDesired: {np.r_[current_position, current_pitch]}\n----")

    for i in range(4):
        msg = Float64(data=dq[i])
        sim.joint_publishers[i].publish(msg)
