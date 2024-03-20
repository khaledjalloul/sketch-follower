#!/usr/bin/env python3

from tkinter import *
import rospy

from geometry_msgs.msg import Pose2D

FRAME_HEIGHT = 600
FRAME_WIDTH = 300

def map_coordinates(x, y, reverse = False):
    if not reverse:
        robot_x = (x * 2 / FRAME_WIDTH) + 0.5
        robot_y = ((FRAME_HEIGHT - y) * 4 / FRAME_HEIGHT) - 2
        return robot_x, robot_y
    else:
        frame_x = (x - 0.5) * FRAME_WIDTH / 2
        frame_y = - ((y + 2) * FRAME_HEIGHT / 4 - FRAME_HEIGHT)
        return frame_x, frame_y


def eef_cb(data):
    frame_x, frame_y = map_coordinates(data.x, data.y, True)
    frame.create_oval(frame_x-1,frame_y-1,frame_x+1,frame_y+1, fill="#FF9900")

rospy.init_node("cursor_publisher")
rospy.set_param('move_robot', False)
pub = rospy.Publisher("/sketch_follower/cursor_position", Pose2D, queue_size=10)
sub = rospy.Subscriber("/sketch_follower/eef_position", Pose2D, eef_cb)

root = Tk()
root.geometry('400x700')
root.title('Cursor Publisher')

frame_coordinates = StringVar()
robot_coordinates = StringVar()

frame_coordinates.set("Frame coordinates: 0, 0")
robot_coordinates.set("Robot coordinates: 0, 0")

frame_coordinates_label = Label(root, textvariable=frame_coordinates)
frame_coordinates_label.pack(pady=10)

robot_coordinates_label = Label(root, textvariable=robot_coordinates)
robot_coordinates_label.pack()

frame = Canvas(root, height=FRAME_HEIGHT, width=FRAME_WIDTH, background="white")
frame.pack(pady=20)

i = 0
recordMotion = False
waypoints = []

def motion(event):
    global i
    i+=1
    if (i >= 10 and recordMotion):
        x, y = event.x, event.y
        frame_coordinates.set(f'Frame coordinates: {x}, {y}')
        robot_x, robot_y = map_coordinates(x, y)
        pose = Pose2D(x=robot_x, y=robot_y)
        pub.publish(pose)
        robot_coordinates.set(f'Robot coordinates: {round(robot_x, 5)}, {round(robot_y, 5)}')
        i = 0
        frame.create_oval(x-1,y-1,x+1,y+1, fill="#0099FF")

def mouseDown(event):
    global recordMotion
    recordMotion = True
    frame.delete("all")

def mouseUp(event):
    global recordMotion
    recordMotion = False
    rospy.set_param('move_robot', True)


frame.bind('<Motion>', motion)
frame.bind('<ButtonPress-1>', mouseDown)
frame.bind('<ButtonRelease-1>', mouseUp)

root.mainloop()

