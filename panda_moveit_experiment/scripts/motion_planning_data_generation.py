import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import pickle

"""
1. 적당한 초기위치 및 목표 선정
2. RRT* 와 같은 알고리즘 선정
3. 모션 플레닝 경로 저장 
    - 환경 번호
    - 경로
    - 경로 길이
    - ...
4. 
"""



_colors = {}

def _init_colors(name, rgba):

    rgba = [x/225.0 for x in rgba]

    color = moveit_msgs.msg.ObjectColor()
    color.id = name
    color.color.r = rgba[0]
    color.color.g = rgba[1]
    color.color.b = rgba[2]
    color.color.a = rgba[3]

    _colors[name] = color

def pub_color(scene):
    p = moveit_msgs.msg.PlanningScene()
    p.is_diff = True
    for color in _colors.values():
        p.object_colors.append(color)
    scene._scene_pub.publish(p) # 이 함수가 뭔지를 모르겠다.

def create_environment(robot, scene):
    
    # Set plane pose 
    plane_pos = geometry_msgs.msg.PoseStamped()
    plane_pos.header.frame_id = robot.get_planning_frame()
    
    # Set table pose
    table_pos = geometry_msgs.msg.PoseStamped()
    table_pos.header.frame_id = robot.get_planning_frame()
    table_pos.pose.position.x = 0.6
    table_pos.pose.position.y = 0.0
    table_pos.pose.position.z = 0.2

    # Remove previous environment objects
    objects = scene.get_known_object_names()
    if objects:
        for i in objects:
            scene.remove_world_object(i)

    rospy.sleep(1)

    # Set a new environemt
    scene.add_box("table", table_pos, (0.8, 1.5, 0.4))
    # scene.add_plane("plane", plane_pos)

    rospy.sleep(1)

    table_color = [105, 105, 105, 1]
    plane_color = [255, 0, 0, 1]

    _init_colors('table',table_color)
    # _init_colors('plane',plane_color)

    # pub_color(scene)


class ScenceGenerator():
    def __init__(self, obstacles):
        self._obstacles = obstacles

        self._robot = None
        self._scene = None
    
    def setup_scene(self, robot, scene, move_group):
        self._robot = robot
        self._scene = scene
        self._move_group = move_group




def main():
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node("motion_planning", anonymous=True) # input 애매

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    scene._scene_pub = rospy.Publisher(
        'planning_scene', moveit_msgs.msg.PlanningScene, queue_size=0)
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    create_environment(robot, scene)
    
    max_time = 300
    move_group.set_planng_time(max_time)

    path_dict = {}
    path_file = "data/path_data_example"






if __name__ == "__main__":
    main()
