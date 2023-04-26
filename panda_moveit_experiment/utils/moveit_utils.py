#!/usr/bin/env python

import sys
import copy
import rospy
import pickle
# import moveit_commander
# import moveit_msgs.msg
import geometry_msgs.msg


def create_scene_obstacle_dict(name, size, orientation, z_offset):
    obs_dict = {}
    obs_dict['name'] = name
    obs_dict['size'] = size
    obs_dict['orientation'] = orientation
    obs_dict['z_offset'] = z_offset

    return obs_dict



def save_env_dict():
    obs_dict = {}
    pose_dict = {}
    env_dict = {}

    temp_1 = create_scene_obstacle_dict('box_1',[0.1, 0.1, 0.4], orientation=None, z_offset=0.3)
    temp_2 = create_scene_obstacle_dict('box_2',[0.1, 0.1, 0.4], orientation=None, z_offset=0.3)
    temp_3 = create_scene_obstacle_dict('box_3',[0.1, 0.1, 0.4], orientation=None, z_offset=0.3)
    
    name_1 = temp_1['name']
    name_2 = temp_2['name']
    name_3 = temp_3['name']

    del temp_1['name']
    del temp_2['name']
    del temp_3['name']

    obs_dict[name_1] = temp_1
    obs_dict[name_2] = temp_2
    obs_dict[name_3] = temp_3

    for i in range(60):
        temp = {}
        temp[name_1] = [0.6, -0.4, 0.2]
        temp[name_2] = [0.45 + 0.005 * i, 0.0, 0.2] # 0.45 ~ 0.75 (0.005 m/step)
        temp[name_3] = [0.6, 0.4, 0.2]

        pose_dict[f'train_env_{i}'] = temp
    
    env_dict['obs_dict'] = obs_dict
    env_dict['pose_dict'] = pose_dict

    with open('../env/train_environments.pkl','wb') as f:
        pickle.dump(env_dict, f, protocol=pickle.HIGHEST_PROTOCOL)


       

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

save_env_dict()