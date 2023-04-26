#!/usr/bin/env python

import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import pickle
import argparse
import numpy as np
# from .utils.moveit_utils import PlanningSceneModifier

"""
move_group.get_current_joint_values() : 7개 joint 값 얻기
move_group.get_current_pose() : goal_dict['env_name']['pose'] 내부 자료형

success == False 가 뜰 경우 rviz를 실행시키는 터미널에서 다음과 같은 오류 발생.
    [ERROR] [1682426633.427900241]: panda_arm/panda_arm: 
    Unable to sample any valid states for goal tree

"""

class PlanningSceneModifier():
    def __init__(self, robot, scene):
        self._robot = robot
        self._scene = scene
        self._obstacles = None

    def set_obstacles_info(self, obstacles):
        self._obstacles = obstacles
    
    def create_environment(self):
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = self._robot.get_planning_frame()
        table_pose.pose.position.x = 0.6
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.15

        objects_name = self._scene.get_known_object_names()
        for i in objects_name:
            self._scene.remove_world_object(i)
        
        rospy.sleep(1)

        print("Adding table...")
        self._scene.add_box('table', table_pose, (0.6, 1.2, 0.3))
        
        rospy.sleep(1)
    
    def create_obstacles(self, pose_dict):
        for name in pose_dict.keys():
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = self._robot.get_planning_frame()
            pose.pose.position.x = pose_dict[name][0]
            pose.pose.position.y = pose_dict[name][1]
            pose.pose.position.z = pose_dict[name][2] + self._obstacles[name]['z_offset']
            
            # pose_dict -> NO orientation info ? !!!
            # Maybe the orientation is fixed and positions are changed.
            if self._obstacles[name]['orientation'] is not None:
                pose.pose.orientation.x = self._obstacles[name]['orientation'][0]
                pose.pose.orientation.y = self._obstacles[name]['orientation'][1]
                pose.pose.orientation.z = self._obstacles[name]['orientation'][2]
                pose.pose.orientation.w = self._obstacles[name]['orientation'][3]

            self._scene.add_box(name, pose, size=self._obstacles[name]['size'])
            print("Add ", name)
        rospy.sleep(1)
        print("Obstacles in the current scene: ")
        print(self._scene.get_known_object_names())

    def remove_obstacles(self):
        for name in self._obstacles.keys():
            self._scene.remove_world_object(name)

def create_scene_obstacle_dict(name, size, orientation, z_offset):
    obs_dict = {}
    obs_dict['name'] = name
    obs_dict['size'] = size
    obs_dict['orientation'] = orientation
    obs_dict['z_offset'] = z_offset

    return obs_dict

def go_to_goal_pose(move_group, goal_pose):
    move_group = move_group

    goal_pose = geometry_msgs.msg.Pose()
    goal_pose.position.x = goal_pose[0]
    goal_pose.position.y = goal_pose[1]
    goal_pose.position.z = goal_pose[2]
    
    goal_pose.orientation.w = goal_pose[3]

    move_group.set_pose_target(goal_pose)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose

def sample_cartesian_pose():
    x = np.random.uniform(low=0.45, high=0.75, size=1)[0]
    y = np.random.uniform(low=0.15, high=0.25, size=1)[0]
    z = np.random.uniform(low=0.4, high=0.6, size=1)[0]
    w = np.random.uniform(low=0.0, high=3.14, size=1)[0]

    return [x,y,z,w]
    
def sample_joint_goal():
    # NotImplimentedError
    x = np.random.uniform(low=0.45, high=0.75, size=1)[0]
    y = np.random.uniform(low=0.15, high=0.25, size=1)[0]
    z = np.random.uniform(low=0.4, high=0.6, size=1)[0]
    w = np.random.uniform(low=0.0, high=3.14, size=1)[0]

    return 



def main(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planning_generation', anonymous=True)

    MAX_TIME = 300
    path_dict = {}
    path_file = args.path_data_path + args.path_data_file

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene._scene_pub = rospy.Publisher(
        'planning_scene', moveit_msgs.msg.PlanningScene, queue_size=0)
    move_group = moveit_commander.MoveGroupCommander('panda_arm')
    move_group.set_planning_time(MAX_TIME)


    # ROS를 이용할 경우 현재 파일 실행 경로가 home/leh 로 나온다.    
    # print("Current Path : ",  os.getcwd())
    with open('ws_moveit/src/panda_moveit_experiment/env/train_environments.pkl', 'rb') as env_f:
        env_dict = pickle.load(env_f)


    scene_modifier = PlanningSceneModifier(robot, scene)
    scene_modifier.set_obstacles_info(env_dict['obs_dict'])
    scene_modifier.create_environment()

    robot_state = robot.get_current_state()
    print("robot state: ",robot_state.joint_state.position)

    envs = env_dict['pose_dict'].keys()
    done = False
    iter = 0
    print("Testing Envs : ")
    print(envs)
    print("====================================")
    print(robot_state.joint_state)

    while (not rospy.is_shutdown() and not done):
        for i, name in enumerate(envs):
            print('Env iteration number: ' + str(i))
            print('Env name: ' + str(name))

            scene_modifier.remove_obstacles()
            new_pose = env_dict['pose_dict'][name]
            scene_modifier.create_obstacles(new_pose)
            print("Loaded new pose and create obstacles\n")

            path_dict[name] = {}
            path_dict[name]['path'] = []
            path_dict[name]['cost'] = []
            path_dict[name]['times'] = []
            path_dict[name]['total'] = 0
            path_dict[name]['feasibility'] = 0

            rospy.sleep(1)

            go_to_goal_pose(move_group=move_group)
    
    



    

def make_goal_dict(args):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planning_generation', anonymous=True)

    MAX_TIME = 5
    goal_dict = {}
    # goal_file = args.path_data_path + args.path_data_file
    goal_file = 'ws_moveit/src/panda_moveit_experiment/env/train_environments_goals.pkl'

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene._scene_pub = rospy.Publisher(
        'planning_scene', moveit_msgs.msg.PlanningScene, queue_size=0)
    move_group = moveit_commander.MoveGroupCommander('panda_arm')
    move_group.set_planning_time(MAX_TIME)


    # ROS를 이용할 경우 현재 파일 실행 경로가 home/leh 로 나온다.    
    # print("Current Path : ",  os.getcwd())
    with open('ws_moveit/src/panda_moveit_experiment/env/train_environments.pkl', 'rb') as env_f:
        env_dict = pickle.load(env_f)


    scene_modifier = PlanningSceneModifier(robot, scene)
    scene_modifier.set_obstacles_info(env_dict['obs_dict'])
    scene_modifier.create_environment()

    robot_state = robot.get_current_state()
    print("robot state: ",robot_state.joint_state.position)

    envs = env_dict['pose_dict'].keys()
    done = False
    iter = 0
    print("Testing Envs : ")
    print(envs)
    MAX_ITER = 100

    while (not rospy.is_shutdown() and not done):
        for i, name in enumerate(envs):
            # print('Env iteration number: ' + str(i))
            # print('Env name: ' + str(name))

            scene_modifier.remove_obstacles()
            new_pose = env_dict['pose_dict'][name]
            scene_modifier.create_obstacles(new_pose)
            # print("Loaded new pose and create obstacles\n")

            goal_dict[name] = {}
            goal_dict[name]['configuration_space'] = []
            goal_dict[name]['cartesian_space'] = []
            
            NUM_ITER = 0
            while (NUM_ITER < MAX_ITER):
                goal_pose = geometry_msgs.msg.Pose()
                goal_pose.position.x  = np.random.uniform(low=0.45, high=0.75, size=1)[0]
                goal_pose.position.y = np.random.uniform(low=-0.25, high=-0.15, size=1)[0]
                goal_pose.position.z = np.random.uniform(low=0.4, high=0.6, size=1)[0]
                
                # goal_pose.orientation.x = 1
                # goal_pose.orientation.y = 1
                # goal_pose.orientation.z = 1
                goal_pose.orientation.w = np.random.uniform(low=0.0, high=3.14, size=1)[0]

                move_group.set_pose_target(goal_pose)
                success = move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                current_pose = move_group.get_current_pose().pose
                

                if success:
                    goal_dict[name]['configuration_space'].append(move_group.get_current_joint_values())
                    goal_dict[name]['cartesian_space'].append(move_group.get_current_pose())
                    
                    NUM_ITER+=1
                
                print('(i, num_iter, success): {}, {}, {}'.format(i,NUM_ITER, success))

            with open('ws_moveit/src/panda_moveit_experiment/env/goal_example' + "_" + name + ".pkl", "wb") as goal_ff:
                pickle.dump(goal_dict[name], goal_ff)

            with open(goal_file,'wb') as goal_f:
                pickle.dump(goal_dict, goal_f)

        print("==========================================")
        print("==========================================")
        print("=============== D O N E ==================")
        print("==========================================")
        
        with open(goal_file,'wb') as goal_f:
            pickle.dump(goal_dict, goal_f)

        print("============= SAVE COMPLETE ===============")
                

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()

        parser.add_argument('--path_data_path', type=str, default='ws_moveit/src/panda_moveit_experiment/data/')
        parser.add_argument('--path_data_file', type =str, default='path_data_sample')
    
        args = parser.parse_args()

        # main(args)
        make_goal_dict(args)
    except rospy.ROSInterruptException:
        pass