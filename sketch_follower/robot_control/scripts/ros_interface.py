import rospy
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Pose2D
from kinematics import Kinematics

class ROSInterface:
    def __init__(self):
        rospy.init_node("controller")

        control_mode = rospy.get_param('/sketch_follower/control_mode')

        self.kin = Kinematics()
        
        self.q = np.zeros(3)
        self.dq = np.zeros(3)
        self.p_desired = np.array([0, 4.5, 0])

        joint_states = rospy.topics.Subscriber(
            "/sketch_follower/joint_states", JointState, callback=self.joint_states_cb)

        cursor = rospy.topics.Subscriber(
            "/sketch_follower/cursor_position", Pose2D, callback=self.cursor_cb)

        self.cursor_feedback = rospy.topics.Publisher(
            "/sketch_follower/eef_position", Pose2D, queue_size=10)

        self.joint_0 = rospy.topics.Publisher(
            f"/sketch_follower/joint_0_{control_mode}_controller/command", Float64, queue_size=10)
        self.joint_1 = rospy.topics.Publisher(
            f"/sketch_follower/joint_1_{control_mode}_controller/command", Float64, queue_size=10)
        self.joint_2 = rospy.topics.Publisher(
            f"/sketch_follower/joint_2_{control_mode}_controller/command", Float64, queue_size=10)

    def joint_states_cb(self, data: JointState):
        self.q[:] = data.position

        self.q = self.q % (2 * np.pi)
        self.q = np.where(self.q > np.pi, self.q - 2 * np.pi, self.q)

        p = self.kin.p(self.q)
        self.cursor_feedback.publish(Pose2D(x = p[0], y = p[1]))
        
        if len(data.velocity) != 0:
            self.dq[:] = data.velocity

    def cursor_cb(self, data: Pose2D):
        self.p_desired[0] = data.x
        self.p_desired[1] = data.y
