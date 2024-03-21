import rospy
import numpy as np
from typing import List

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Pose2D
from kinematics import Kinematics


class ROSInterface:
    def __init__(self):
        rospy.init_node("controller")

        control_mode = rospy.get_param('/sketch_follower/control_mode')

        self.kin = Kinematics()

        self.q = np.zeros(4)
        self.dq = np.zeros(4)
        self.q_reset = [0, -0.5, 2, -1.5]

        self.desired_position = None

        joint_states = rospy.topics.Subscriber(
            "/sketch_follower/joint_states", JointState, callback=self.joint_states_cb)

        cursor = rospy.topics.Subscriber(
            "/sketch_follower/cursor_position", Pose2D, callback=self.cursor_cb)

        self.cursor_feedback = rospy.topics.Publisher(
            "/sketch_follower/eef_position", Pose2D, queue_size=10)

        self.joint_publishers: List[rospy.topics.Publisher] = []

        for i in range(4):
            self.joint_publishers.append(rospy.topics.Publisher(
                f"/sketch_follower/joint_{i}_{control_mode}_controller/command", Float64, queue_size=10))

    def joint_states_cb(self, data: JointState):
        self.q[1] = data.position[0]
        self.q[2] = data.position[1]
        self.q[0] = data.position[2]
        self.q[3] = data.position[3]

        self.q = self.q % (2 * np.pi)
        self.q = np.where(self.q > np.pi, self.q - 2 * np.pi, self.q)

        p = self.kin.p(self.q)[0:3, 3]
        self.cursor_feedback.publish(Pose2D(x=p[0], y=p[1]))

        if len(data.velocity) != 0:
            self.dq[1] = data.velocity[0]
            self.dq[2] = data.velocity[1]
            self.dq[0] = data.velocity[2]
            self.dq[3] = data.velocity[3]

    def cursor_cb(self, data: Pose2D):
        if self.desired_position is None:
            self.desired_position = np.array([data.x, data.y])
        else:
            self.desired_position[0] = data.x
            self.desired_position[1] = data.y
