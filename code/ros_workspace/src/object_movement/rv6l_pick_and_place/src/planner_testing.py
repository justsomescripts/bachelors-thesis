#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

from bauplan_auswertung import get_position
from bauplan_auswertung import hoehenauswertung

#from subscriber_kinect_data_copy import callback

#from darknet_ros_msgs.msg import BoundingBox

import sys
import copy
import rospy
import moveit_commander
#import move_group
import moveit_msgs.msg
import geometry_msgs.msg

#from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

TCP = 0.29 #FLOAT

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "rv6l_planning_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        
        move_group.set_planner_id("RRTstar")

        print("new planner ID", move_group.get_planner_id)


        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("rv6l_pick_place Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("rv6l_pick_place End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("rv6l_pick_place Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("rv6l_pick_place Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_ref_pose(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group


        ## BEGIN_SUB_TUTORIAL plan_to_ref_pose
        ##
        ## Planning to a reference pose
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        ref_pose_goal = geometry_msgs.msg.Pose()
        ref_pose_goal.orientation.x = 0
        ref_pose_goal.orientation.y = 0.7071067811865464
        ref_pose_goal.orientation.z = 0
        ref_pose_goal.orientation.w = 0.7071067811865464
        ref_pose_goal.position.x = 1.01
        ref_pose_goal.position.y = 0
        ref_pose_goal.position.z = 1.73

        move_group.set_pose_target(ref_pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(ref_pose_goal, current_pose, 0.01)


    def go_to_pose_goal(self, posx, posy, posz):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0
        pose_goal.orientation.x = 1
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.position.x = posx
        pose_goal.position.y = posy
        pose_goal.position.z = posz

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL


    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        #self.move_group.set_end_effector_link("rv6l_greifer_sauger")       ###Aus dem internet
    
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the RV6L Sauger. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'RV6L_Greifer_Sauger'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "RV6L_Greifer_Sauger"
        touch_links = robot.get_link_names(grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


def main():
    try:
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt Python Interface Pick and Place Demo")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        tutorial = MoveGroupPythonInterfaceTutorial()

    
        get_position()                  #funtionsaufruf für Bauplanposition
        heightinfo = hoehenauswertung()
        position = get_position()       # deklarieren von positions mit wertrückgabe von funktion (type tuple)

        #pos1x = float(position[0][0])      #pos2x = float(position[1][0])      #pos3x = float(position[2][0])
        #pos1y = float(position[0][1])      #pos2y = float(position[1][1])      #pos3y = float(position[2][1])
        #pos1z = float(position[0][2])      #pos2z = float(position[1][2])      #pos3z = float(position[2][2])

        ################################## Höhe, die zur anpeilung des spezifischen Blocks addiert werden muss (hälfte der Blockhöhe)
        height1 = float(position[5][2])/2           # ACHTUNG, RICHTIGE HÖHE benutzen
        print(height1)
        height2 = float(position[6][2])/2           
        print(height2)
        height3 = float(position[7][2])/2
        print(height3)
        height4 = float(position[8][1])/2           # [x][1] weil ein Cylinder nur 2 Maße hat
        print(height4)
        height5 = float(position[9][1])/2           # [x][1] weil ein Cylinder nur 2 Maße hat
        print(height5)

        ################################## Abfrage niedrigste Position
        heightinfo = hoehenauswertung()

        ################################## Hardcode Positon Objekte (aus hoehenauswertung())

        pos1_gesucht = heightinfo[2]
        pos2_gesucht = heightinfo[5]
        pos3_gesucht = heightinfo[8]
        pos4_gesucht = heightinfo[11]
        pos5_gesucht = heightinfo[14]

        pos1_min_z = heightinfo[1]
        pos2_min_z = heightinfo[4]
        pos3_min_z = heightinfo[7]
        pos4_min_z = heightinfo[10]
        pos5_min_z = heightinfo[13]

        object1_min_z = heightinfo[0]
        object2_min_z = heightinfo[3]
        object3_min_z = heightinfo[6]
        object4_min_z = heightinfo[9]
        object5_min_z = heightinfo[12]

        
        #tutorial.go_to_ref_pose()
        ######################## Bewegung zum 1. gesuchten Objekt
        input("press Enter to move")
        tutorial.go_to_pose_goal(float(pos1_gesucht[0]), float(pos1_gesucht[1]), (float(pos1_gesucht[2]) + height1 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos1_gesucht[0]), float(pos1_gesucht[1]), (float(pos1_gesucht[2]) + height1 + TCP))
        # attach
        ######################## Bewegung zum Ziel
        input("press Enter to move")
        tutorial.go_to_pose_goal(float(pos1_min_z[0]), float(pos1_min_z[1]), (float(pos1_min_z[2]) + height1 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos1_min_z[0]), float(pos1_min_z[1]), (float(pos1_min_z[2]) + height1 + TCP))
        # detach
        tutorial.go_to_pose_goal(float(pos1_min_z[0]), float(pos1_min_z[1]), (float(pos1_min_z[2]) + height1 + TCP + 0.15))
        #tutorial.go_to_ref_pose()
        ######################## Bewegung zum 2. gesuchten Objekt

        tutorial.go_to_pose_goal(float(pos2_gesucht[0]), float(pos2_gesucht[1]), (float(pos2_gesucht[2]) + height2 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos2_gesucht[0]), float(pos2_gesucht[1]), (float(pos2_gesucht[2]) + height2 + TCP))
        # attach
        ######################## Bewegung zum Ziel
        
        tutorial.go_to_pose_goal(float(pos2_min_z[0]), float(pos2_min_z[1]), (float(pos2_min_z[2]) + height2 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos2_min_z[0]), float(pos2_min_z[1]), (float(pos2_min_z[2]) + height2 + TCP))
        # detach
        tutorial.go_to_pose_goal(float(pos2_min_z[0]), float(pos2_min_z[1]), (float(pos2_min_z[2]) + height2 + TCP + 0.15))
        #tutorial.go_to_ref_pose()
        ######################## Bewegung zum 3. gesuchten Objekt

        tutorial.go_to_pose_goal(float(pos3_gesucht[0]), float(pos3_gesucht[1]), (float(pos3_gesucht[2]) + height3 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos3_gesucht[0]), float(pos3_gesucht[1]), (float(pos3_gesucht[2]) + height3 + TCP))
        # attach
        ######################## Bewegung zum Ziel

        tutorial.go_to_pose_goal(float(pos3_min_z[0]), float(pos3_min_z[1]), (float(pos3_min_z[2]) + height3 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos3_min_z[0]), float(pos3_min_z[1]), (float(pos3_min_z[2]) + height3 + TCP))
        # detach
        tutorial.go_to_pose_goal(float(pos3_min_z[0]), float(pos3_min_z[1]), (float(pos3_min_z[2]) + height3 + TCP + 0.15))
        #tutorial.go_to_ref_pose()

        ######################## Bewegung zum 4. gesuchten Objekt

        tutorial.go_to_pose_goal(float(pos4_gesucht[0]), float(pos4_gesucht[1]), (float(pos4_gesucht[2]) + height4 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos4_gesucht[0]), float(pos4_gesucht[1]), (float(pos4_gesucht[2]) + height4 + TCP))
        # attach
        ######################## Bewegung zum Ziel

        tutorial.go_to_pose_goal(float(pos4_min_z[0]), float(pos4_min_z[1]), (float(pos4_min_z[2]) + height4 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos4_min_z[0]), float(pos4_min_z[1]), (float(pos4_min_z[2]) + height4 + TCP))
        # detach
        tutorial.go_to_pose_goal(float(pos4_min_z[0]), float(pos4_min_z[1]), (float(pos4_min_z[2]) + height4 + TCP + 0.15))
        #tutorial.go_to_ref_pose()

        ######################## Bewegung zum 5. gesuchten Objekt

        tutorial.go_to_pose_goal(float(pos5_gesucht[0]), float(pos5_gesucht[1]), (float(pos5_gesucht[2]) + height5 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos5_gesucht[0]), float(pos5_gesucht[1]), (float(pos5_gesucht[2]) + height5 + TCP))
        # attach
        ######################## Bewegung zum Ziel

        tutorial.go_to_pose_goal(float(pos5_min_z[0]), float(pos5_min_z[1]), (float(pos5_min_z[2]) + height5 + TCP + 0.15))
        tutorial.go_to_pose_goal(float(pos5_min_z[0]), float(pos5_min_z[1]), (float(pos5_min_z[2]) + height5 + TCP))
        # detach
        tutorial.go_to_pose_goal(float(pos5_min_z[0]), float(pos5_min_z[1]), (float(pos5_min_z[2]) + height5 + TCP + 0.15))
        #tutorial.go_to_ref_pose()
        

        #input("============ Press `Enter` to attach a Box to the robot ...")
        #tutorial.attach_box()                                                           #findet grasping group nicht
        #input("============ Press `Enter` to detach the box from the robot ...")
        #tutorial.detach_box()



        print("============ Python pick and place complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()

