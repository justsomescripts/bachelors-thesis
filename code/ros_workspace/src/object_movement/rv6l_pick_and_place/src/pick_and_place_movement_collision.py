#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

from bauplan_auswertung import get_position
from bauplan_auswertung import hoehenauswertung

from subscriber_cam import uwe
from rv6l_3d_msgs.msg import RV6L_positions


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint

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

class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface_demo", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  
        ## This interface can be used to plan and execute motions:
        group_name = "rv6l_planning_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        
        move_group.set_planner_id("RRTstar")          # Festlegen des OMPL Planungsalgorithmus

        print("new planner ID", move_group.get_planner_id)


        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,)


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

        move_group = self.move_group

        ## Planning to a reference pose
        ##
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        ref_pose_goal = geometry_msgs.msg.Pose()
        ref_pose_goal.orientation.x = 0
        ref_pose_goal.orientation.y = 0.7071067811865464
        ref_pose_goal.orientation.z = 0
        ref_pose_goal.orientation.w = 0.7071067811865464
        ref_pose_goal.position.x = 1.01
        ref_pose_goal.position.y = 0
        ref_pose_goal.position.z = 1.53

        move_group.set_pose_target(ref_pose_goal)

        ## Call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(ref_pose_goal, current_pose, 0.01)

    def go_to_scan_pose(self):

        move_group = self.move_group

        ## Planning to a scan pose (only wih camera system)
        ##
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        scan_pose_goal = geometry_msgs.msg.Pose()
        scan_pose_goal.orientation.x = 0
        scan_pose_goal.orientation.y = 0.7071067811865464
        scan_pose_goal.orientation.z = 0
        scan_pose_goal.orientation.w = 0.7071067811865464
        scan_pose_goal.position.x = 0.89
        scan_pose_goal.position.y = 0
        scan_pose_goal.position.z = 1.56                     # z setzt sich zusammen aus: 0.9m Tischhöhe und 0.56m über dem Tisch

        move_group.set_pose_target(scan_pose_goal)

        ## Call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(scan_pose_goal, current_pose, 0.01)


    def go_to_pose_goal(self, posx, posy, posz):
        
        move_group = self.move_group

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
        move_group.clear_pose_targets()
     
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def go_to_scan(self, posx, posy, posz):
        
        move_group = self.move_group

        ## Planning to the specific scan of an object (only with camera system)
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        scan_goal = geometry_msgs.msg.Pose()
        scan_goal.orientation.x = 0
        scan_goal.orientation.y = 0.7071067811865464
        scan_goal.orientation.z = 0
        scan_goal.orientation.w = 0.7071067811865464
        scan_goal.position.x = posx
        scan_goal.position.y = posy
        scan_goal.position.z = posz

        move_group.set_pose_target(scan_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()
     
        current_pose = self.move_group.get_current_pose().pose
        return all_close(scan_goal, current_pose, 0.01)


    def display_trajectory(self, plan):

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

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


    def execute_plan(self, plan):

        move_group = self.move_group

        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        box_name = self.box_name
        scene = self.scene

        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this py, we call this function after adding,
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

def main():
    try:
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt Python Interface Pick and Place Demo")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")

        py = MoveGroupPythonInterface()

        ## Abfrage niedrigste Position
        heightinfo = hoehenauswertung()

        ## Deklarieren von positions mit Wertrückgabe von funktion get_position() (type tuple)
        position = get_position()
        #pos1x = float(position[0][0])      #pos2x = float(position[1][0])      #pos3x = float(position[2][0])
        #pos1y = float(position[0][1])      #pos2y = float(position[1][1])      #pos3y = float(position[2][1])
        #pos1z = float(position[0][2])      #pos2z = float(position[1][2])      #pos3z = float(position[2][2])

        ################################## Höhe, die zur Anpeilung des spezifischen Blocks addiert werden muss (hälfte der Blockhöhe)
        height1 = float(position[5][2])/2           # box_g
        #print(height1)
        height2 = float(position[6][2])/2           # box_r1
        #print(height2)
        height3 = float(position[7][2])/2           # box_r2
        #print(height3)
        height4 = float(position[8][1])/2           # cyl_b1    [x][1] weil ein Cylinder nur 2 Maße hat
        #print(height4)
        height5 = float(position[9][1])/2           # cyl_b2    [x][1] weil ein Cylinder nur 2 Maße hat
        #print(height5)


        ## Hardcode Ursprung Objekte (aus hoehenauswertung())
        ## und Informationen über Bezeichnung und Zelposition mit minimaler Höhe
        pos1_gesucht = heightinfo[2]
        pos2_gesucht = heightinfo[5]
        pos3_gesucht = heightinfo[8]
        pos4_gesucht = heightinfo[11]
        pos5_gesucht = heightinfo[14]                       #19.5

        pos1_min_z = heightinfo[1]
        pos2_min_z = heightinfo[4]
        pos3_min_z = heightinfo[7]
        pos4_min_z = heightinfo[10]
        pos5_min_z = heightinfo[13]
        
        input("<ENTER> Scanposition")
        py.go_to_scan_pose()
        input("<ENTER> erster Block")
        py.go_to_pose_goal(1.43, -0.4, (0.92 + height2 + TCP))

        input("<ENTER> zweiter Block")
        py.go_to_pose_goal(1.34, 0.25, (0.98 + height2 + TCP))

        
        print("============ Python pick and place Demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()

