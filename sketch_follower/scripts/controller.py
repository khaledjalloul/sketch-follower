#!/usr/bin/env python3

import numpy as np
import rospy

from kinematics import Kinematics
from linear_mpc import LinearMPC
from ros_interface import ROSInterface

np.set_printoptions(suppress=True)
np.set_printoptions(precision=3)

kin = Kinematics()
sim = ROSInterface()

r = rospy.Rate(10)

method = 'pid'
# method = 'mpc'

desired_z = 0
desired_pitch = 0

kp = 2

print("Resetting to home position...")

err = sim.q_reset - sim.q

while np.linalg.norm(err) > 0.1:
    err = sim.q_reset - sim.q
    dq = kp * err

    for i in range(4):
        sim.joint_publishers[i].publish(dq[i])

    r.sleep()

print("Position reset.")

if method == 'pid':
    while not rospy.is_shutdown():
        r.sleep()
        if sim.desired_position is None:
            for i in range(4):
                sim.joint_publishers[i].publish(0)
            continue

        current_pose = kin.p(sim.q)
        current_position = current_pose[0:3, 3]
        current_pitch = kin.rot_matrix_to_vector(current_pose[0:3, 0:3])[1]

        w_desired = kp * (np.r_[sim.desired_position, desired_z, desired_pitch] -
                          np.r_[current_position, current_pitch])

        dq = kin.inv_kin_dq(w_desired, sim.q)
        # dq = kin.inv_kin_dq_pos(w_desired[0:3], sim.q)

        print(
            f"Target: {np.r_[sim.desired_position, desired_z, desired_pitch]}\nDesired: {np.r_[current_position, current_pitch]}\n----")

        for i in range(4):
            sim.joint_publishers[i].publish(dq[i])

elif method == 'mpc_task_space':
    mpc = LinearMPC()

    while not rospy.is_shutdown():
        r.sleep()
        if sim.desired_position is None:
            for i in range(4):
                sim.joint_publishers[i].publish(0)
            continue

        current_pose = kin.p(sim.q)
        current_position = current_pose[0:3, 3]
        current_pitch = kin.rot_matrix_to_vector(current_pose[0:3, 0:3])[1]

        print(
            f"Target: {np.r_[sim.desired_position, desired_z, desired_pitch]}\nDesired: {np.r_[current_position, current_pitch]}\n----")

        w_star, p_star = mpc.step(np.r_[current_position, current_pitch],
                                  np.r_[sim.desired_position,
                                        desired_z, desired_pitch],
                                  [0, 0, 0, 0])

        if w_star is not None:
            dq = kin.inv_kin_dq(w_star[:, 0], sim.q)

            for i in range(4):
                sim.joint_publishers[i].publish(dq[i])
        else:
            print("Unfeasible")
            for i in range(4):
                sim.joint_publishers[i].publish(0)
