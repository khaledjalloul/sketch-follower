import numpy as np
import rospy

from kinematics import Kinematics
from linear_mpc import LinearMPC
from ros_interface import ROSInterface

kin = Kinematics()
sim = ROSInterface()

r = rospy.Rate(10)

# method = 'pid_joint_space'
method = 'pid_task_space'
# method = 'mpc_task_space'


if method == 'pid_joint_space':

    while not rospy.is_shutdown():
        q_desired = kin.inv_kin_q(sim.p_desired, sim.q)
        dq = q_desired - sim.q

        print(dq)

        sim.joint_0.publish(dq[0])
        sim.joint_1.publish(dq[1])
        sim.joint_2.publish(dq[2])

        r.sleep()

elif method == 'pid_task_space':
    kp = 2

    while not rospy.is_shutdown():
        p_current = kin.p(sim.q).reshape(-1)
        w_current = kin.w(sim.q, sim.dq)[0:3]

        w_desired = kp * (sim.p_desired - p_current)

        dq = kin.inv_kin_dq(w_desired, sim.q)
        dq = np.where(dq > 3, 3, np.where(dq < -3, -3, dq))
        print(dq)

        sim.joint_0.publish(dq[0])
        sim.joint_1.publish(dq[1])
        sim.joint_2.publish(dq[2])

        r.sleep()

elif method == 'mpc_task_space':
    mpc = LinearMPC()

    while not rospy.is_shutdown():
        p_current = kin.p(sim.q).reshape(-1)
        p_desired = sim.p_desired

        w_star, p_star = mpc.step(p_current, p_desired, [0, 0, 0])

        if w_star is not None:
            print(p_current, p_desired, sim.q)
            print(w_star[:, 0])
            dq = kin.inv_kin_dq(w_star[:, 0], sim.q)

            print(dq)

            sim.joint_0.publish(dq[0])
            sim.joint_1.publish(dq[1])
            sim.joint_2.publish(dq[2])
        else:
            print("Unfeasible")
            sim.joint_0.publish(0)
            sim.joint_1.publish(0)
            sim.joint_2.publish(0)
        r.sleep()
