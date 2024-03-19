import rospy
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Pose2D


class ROSInterface:
    def __init__(self):
        rospy.init_node("controller")

        self.q = np.zeros(3)
        self.dq = np.zeros(3)
        self.p_desired = np.zeros(3)

        joint_states = rospy.topics.Subscriber(
            "/sketch_follower/joint_states", JointState, callback=self.joint_states_cb)

        cursor = rospy.topics.Subscriber(
            "/cursor_position", Pose2D, callback=self.cursor_cb)

        self.joint_0 = rospy.topics.Publisher(
            "/sketch_follower/joint_0_effort_controller/command", Float64, queue_size=10)
        self.joint_1 = rospy.topics.Publisher(
            "/sketch_follower/joint_1_effort_controller/command", Float64, queue_size=10)
        self.joint_2 = rospy.topics.Publisher(
            "/sketch_follower/joint_2_effort_controller/command", Float64, queue_size=10)

    def joint_states_cb(self, data: JointState):
        self.q[:] = data.position

        self.q = self.q % (2 * np.pi)
        self.q = np.where(self.q > np.pi, self.q - 2 * np.pi, self.q)

        if len(data.velocity) != 0:
            self.dq[:] = data.velocity

    def cursor_cb(self, data: Pose2D):
        self.p_desired[0] = data.x
        self.p_desired[1] = data.y
