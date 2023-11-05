#!/usr/bin/env python3

import rospy
from rv6l_3d_msgs.msg import RV6L_positions

class uwe:
    def __init__(self):
        #rospy.init_node('cam_subscriber_Node', anonymous=None)
        rospy.Subscriber("/darknet_ros_3d/bounding_boxes", RV6L_positions, self.callback_pos)
        
    def callback_pos(self, data):
        for box in data.bounding_boxes:
            self.object_pos_cam = [box.Class, box.h, box.w, box.d]

if __name__ == '__main__':
    pos_cam = uwe()
    rospy.wait_for_message("/darknet_ros_3d/bounding_boxes/", RV6L_positions, timeout=None)
    print("[Objektklasse, vertikal, horizontal, Tiefe]: ", pos_cam.object_pos_cam)
