#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

import pickle

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


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
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

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
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
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
        
        self.table_height = 0.4
        self.num_box = 0

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
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
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

    def set_environment(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        table_pos = geometry_msgs.msg.PoseStamped()
        table_pos.header.frame_id = self.robot.get_planning_frame()
        table_pos.pose.position.x = 0.6
        table_pos.pose.position.y = 0.0
        table_pos.pose.position.z = self.table_height/2

        pw = geometry_msgs.msg.PoseStamped()
        pw.header.frame_id = self.robot.get_planning_frame()


        box_name = "table"
        scene.add_box("table", table_pos, size=(0.8, 1.5, self.table_height))
        
        self.box_name = box_name
        # scene.add_plane("wall", pw,)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    


    def add_box(self, pose=[0,0,0,0,0,0,0], timeout=4):
        
        box_name = self.box_name
        scene = self.scene
        self.num_box += 1

        box_pos = geometry_msgs.msg.PoseStamped()
        box_pos.header.frame_id = self.robot.get_planning_frame()
        box_pos.pose.position.x = pose[0]
        box_pos.pose.position.y = pose[1]
        box_pos.pose.position.z = pose[2] + self.table_height
        box_pos.pose.orientation.x = pose[3]
        box_pos.pose.orientation.y = pose[4]
        box_pos.pose.orientation.z = pose[5]
        box_pos.pose.orientation.w = pose[6]
        box_name = f"box_{self.num_box}"

        scene.add_box(box_name, box_pos, size=(0.1, 0.1, pose[2]*2))
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    
    




def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )

        tutorial = MoveGroupPythonInterfaceTutorial()
        input("============ Press `Enter` to add obstacles ...")
        
        tutorial.set_environment()

        tutorial.add_box(pose=[0.5,0 ,0.2, 0,0,0,0])
        tutorial.add_box(pose=[0.5,0.4 ,0.2, 0,0,0,0])
        tutorial.add_box(pose=[0.5,-0.4 ,0.2, 0,0,0,0])

        # print(tutorial.scene.get_known_object_names())
        
        # 현재 joint 값 얻기
        # print(tutorial.move_group.get_current_joint_values())



        js = sensor_msgs.msg.JointState()
        js.header.frame_id = tutorial.robot.get_planning_frame()
        js.header.stamp = rospy.Time.now()
        
        js.name = tutorial.move_group.get_active_joints()
        js.position = [0.0, -0.78, 0.0, -0.35, 0.31, 1.57, 0.78]
        

        print("get_joints:  ",tutorial.move_group.get_joints())
        print("get_active_joints:   ",tutorial.move_group.get_active_joints())
        print("get_random_joint_values:  ", tutorial.move_group.get_random_joint_values())
        print("get_joint_value_target:  ", tutorial.move_group.get_joint_value_target())
        print("get_current_state:  ", tutorial.move_group.get_current_state().joint_state.name)

        rs_man = moveit_msgs.msg.RobotState()
        rs_man.joint_state = js
        
        print("get_current_state:  ", tutorial.move_group.get_current_state())
        # tutorial.move_group.set_start_state(rs_man)
        # print("================change state state =====================")
        # print("get_current_state:  ", tutorial.move_group.get_current_state())
        # tutorial.move_group.set_start_state_to_current_state()

        tutorial.move_group.clear_pose_targets()

        tutorial.move_group.set_joint_value_target([0.0, -0.78, 0.0, -0.35, 0.31, 1.57, 0.78])

        plan = tutorial.move_group.plan()
        
        plan = plan[1]

        planned_position = [plan.joint_trajectory.points[i].positions for i in range(len(plan.joint_trajectory.points))]
        
        print(planned_position[0])
        print(planned_position[-1])
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()