import rospy
import numpy as np
from typing import Tuple

from kinematics import Kinematics

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Pose2D

rospy.init_node("controller")

joint_poses = np.zeros((2, 3))


def joint_states_cb(data: JointState):
    global joint_poses
    joint_poses[0, :] = data.position
    if len(data.velocity) != 0:
        joint_poses[1, :] = data.velocity


joint_states = rospy.topics.Subscriber(
    "/sketch_follower/joint_states", JointState, callback=joint_states_cb)

kin = Kinematics()
kin.q = joint_poses[0]
p_desired = np.zeros(3)
q_desired = kin.inv_kin(p_desired)


def cursor_cb(data: Pose2D):
    global p_desired, q_desired
    p_desired[0] = data.x
    p_desired[1] = data.y
    q_desired = kin.inv_kin(p_desired)


cursor = rospy.topics.Subscriber(
    "/cursor_position", Pose2D, callback=cursor_cb)

joint_0 = rospy.topics.Publisher(
    "/sketch_follower/joint_0_effort_controller/command", Float64, queue_size=10)
joint_1 = rospy.topics.Publisher(
    "/sketch_follower/joint_1_effort_controller/command", Float64, queue_size=10)
joint_2 = rospy.topics.Publisher(
    "/sketch_follower/joint_2_effort_controller/command", Float64, queue_size=10)


kp = 3
kd = 1

r = rospy.Rate(10)

while not rospy.is_shutdown():
    qdd = kp * (q_desired - joint_poses[0]) + kd * (0 - joint_poses[1])
    print(q_desired, joint_poses[0], qdd)

    joint_0.publish(qdd[0])
    joint_1.publish(qdd[1])
    joint_2.publish(qdd[2])

    r.sleep()
