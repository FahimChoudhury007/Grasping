#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
from tf.transformations import quaternion_from_matrix
from tf.transformations import rotation_matrix
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

from grasping import *


def request_move(request, pos, quat):
    request.ik_request.pose_stamped.pose.position.x = pos[0]
    request.ik_request.pose_stamped.pose.position.y = pos[1]
    request.ik_request.pose_stamped.pose.position.z = pos[2]
    request.ik_request.pose_stamped.pose.orientation.x = quat[0]
    request.ik_request.pose_stamped.pose.orientation.y = quat[1]
    request.ik_request.pose_stamped.pose.orientation.z = quat[2]
    request.ik_request.pose_stamped.pose.orientation.w = quat[3]

def execute_move(compute_ik, request):
    try:
        response = compute_ik(request)        
        print(response)
        group = MoveGroupCommander("right_arm")
        group.set_pose_target(request.ik_request.pose_stamped)

        plan = group.plan()
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        
        if user_input == 'y':
            group.execute(plan[1])
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def grasp_to_base(grasp_matrix):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        r = rospy.Rate(10) # 10hz
        trans_base_to_gripper = tfBuffer.lookup_transform("base", "right_gripper_tip", rospy.Time(), rospy.Duration(10))

        obj_to_grasp = PoseStamped() 
        q = quaternion_from_matrix(grasp_matrix)
        print(q)

        obj_to_grasp.pose.position.x = 0
        obj_to_grasp.pose.position.y = 0
        obj_to_grasp.pose.position.z = 0
        obj_to_grasp.pose.orientation.x = q[0]
        obj_to_grasp.pose.orientation.y = q[1]
        obj_to_grasp.pose.orientation.z = q[2]
        obj_to_grasp.pose.orientation.w = q[3]

        grasp_in_base = do_transform_pose(obj_to_grasp, trans_base_to_gripper)
        return grasp_in_base


def main():
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open()

    index = 2
    grasp_matrix, vertices, quality_value = run("nozzle", "robust", index=index)
    print(grasp_matrix)

    if isinstance(grasp_matrix, type(None)):
        print(f"Matrix None is index: {index}")

    else:
        while not rospy.is_shutdown():
            input('Press [ Enter ]: ')

            request = GetPositionIKRequest()
            request.ik_request.group_name = "right_arm"

            link = "right_gripper_tip"

            request.ik_request.ik_link_name = link
            request.ik_request.pose_stamped.header.frame_id = "base"

            grasp_pose = grasp_to_base(grasp_matrix)

            grasp_pos = [grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z]
            grasp_down_pos = [grasp_pose.pose.position.x, grasp_pose.pose.position.y, -.02]
            grasp_new_pos = [grasp_pose.pose.position.x - 0.05, grasp_pose.pose.position.y + 0.2, 0.04]

            grasp_quat = [grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y, grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w]

            request_move(request, grasp_pos, grasp_quat)
            execute_move(compute_ik, request)

            request_move(request, grasp_down_pos, grasp_quat)
            execute_move(compute_ik, request)

            right_gripper.close()
            
            request_move(request, grasp_pos, grasp_quat)
            execute_move(compute_ik, request)

            request_move(request, grasp_new_pos, grasp_quat)
            execute_move(compute_ik, request)

            right_gripper.open()
        
# Python's syntax for a main() method
if __name__ == '__main__':
    main()
