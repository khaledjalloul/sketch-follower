import numpy as np
import rospy

from kinematics import Kinematics
from dynamics import Dynamics
# from linear_mpc import LinearMPC
from ros_interface import ROSInterface

kin = Dynamics()
sim = ROSInterface()

r = rospy.Rate(10)

# method = 'pid_joint_space'
method = 'pid_task_space'
# method = 'mpc_joint_space'
# method = 'mpc_task_space'


if method == 'pid_joint_space':
    kp = 1
    kd = 0.1

    while not rospy.is_shutdown():
        q_desired = kin.inv_kin_q(sim.p_desired, sim.q)

        ddq = kp * (q_desired - sim.q) + kd * (0 - sim.dq)
        tau = kin.forward_dyn(sim.q, sim.dq, ddq)

        print(ddq, tau)

        sim.joint_0.publish(ddq[0])
        sim.joint_1.publish(ddq[1])
        sim.joint_2.publish(ddq[2])

        r.sleep()

elif method == 'pid_task_space':
    kp = 1
    kd = 0.1

    while not rospy.is_shutdown():
        p_current = kin.p(sim.q).reshape(-1)
        w_current = kin.w(sim.q, sim.dq)[0:3]
        dw = kp * (sim.p_desired - p_current) + kd * (0 - w_current)

        ddq = kin.inv_kin_ddq(dw, sim.q, sim.dq)
        ddq = np.where(ddq > 10, 10, np.where(ddq < -10, -10, ddq))

        tau = kin.forward_dyn(sim.q, sim.dq, ddq)

        print(ddq, tau)

        sim.joint_0.publish(ddq[0])
        sim.joint_1.publish(ddq[1])
        sim.joint_2.publish(ddq[2])

        r.sleep()

# else:
#     mpc = LinearMPC()

# while not rospy.is_shutdown():

#     q = kin.inv_kin(p_desired)

#     ddq, x = mpc.step(x0=joint_poses.reshape(-1),
#                       xf=np.r_[q, [0, 0, 0]].reshape(-1),
#                       uf=[0, 0, 0])

#     if ddq is not None:
#         # ddq *= 100
#         # ddq = np.where(ddq > 5, 5, ddq)
#         # ddq = np.where(ddq < -5, -5, ddq)
#         # print(q, x[0:3, -1], ddq[:, 0])
#         print(ddq.T)
#         print("\n")
#         print(x.T)
#         print("--------")
#         # ddq = kp * ddq + kd * (ddq - prev_error)
#         # prev_error = ddq
#         joint_0.publish(ddq[0, 0])
#         joint_1.publish(ddq[1, 0])
#         joint_2.publish(ddq[2, 0])
#     else:
#         print("Unfeasible")
#         joint_0.publish(0)
#         joint_1.publish(0)
#         joint_2.publish(0)

#     r.sleep()


# kp = 4
# kd = 2
# ki = 0.01

# r = rospy.Rate(10)
# prev_err = 0
# while not rospy.is_shutdown():
#     err = q_desired - joint_poses[0]
#     qdd = kp * err + kd * (0 - joint_poses[1]) + ki * prev_err
#     print(err, qdd, ki * (err + prev_err))
#     prev_err += err

#     joint_0.publish(qdd[0])
#     joint_1.publish(qdd[1])
#     joint_2.publish(qdd[2])

#     r.sleep()
