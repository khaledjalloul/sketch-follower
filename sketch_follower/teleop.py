from pynput import keyboard
import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from sketch_follower.ros.ros_interface import ROSInterface
from sketch_follower.model.kinematics import Kinematics

sim = ROSInterface()
kin = Kinematics()

teleop_node = Node("teleop")

axes = np.zeros(3, dtype=np.float32)
vel = 3.0


def set_axis(index, sign):
    global axes
    if axes[index] != sign:
        axes[index] = sign


def on_press(key):
    try:
        if key == keyboard.Key.up:
            set_axis(0, 1)
        elif key == keyboard.Key.down:
            set_axis(0, -1)
        elif key == keyboard.Key.left:
            set_axis(1, 1)
        elif key == keyboard.Key.right:
            set_axis(1, -1)
        elif key.char == "q":
            set_axis(2, 1)
        elif key.char == "e":
            set_axis(2, -1)
    except AttributeError:
        pass


def on_release(key):
    if key == keyboard.Key.esc:
        return False
    try:
        if key == keyboard.Key.up:
            set_axis(0, 0)
        elif key == keyboard.Key.down:
            set_axis(0, 0)
        elif key == keyboard.Key.left:
            set_axis(1, 0)
        elif key == keyboard.Key.right:
            set_axis(1, 0)
        elif key.char == "q":
            set_axis(2, 0)
        elif key.char == "e":
            set_axis(2, 0)
    except AttributeError:
        pass


with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    print("Press ESC to stop.")

    while rclpy.ok():
        desired_w = axes * vel
        dq = kin.inv_kin_dq(desired_w, sim.q, target="position")

        msg = Float64MultiArray(data=dq)
        sim.joint_publisher.publish(msg)

        sim.r.sleep()

    listener.join()
