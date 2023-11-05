#!/usr/bin/env python3
import rospy
from rv6l_3d_msgs.msg import RV6L_positions
from subscriber_cam import uwe

ichbinder = uwe()

rospy.wait_for_message("/darknet_ros_3d/bounding_boxes/", RV6L_positions, timeout=None)
print("[Objektklasse, horizontal, vertikal, Tiefe]: ", ichbinder.object_pos_cam)

