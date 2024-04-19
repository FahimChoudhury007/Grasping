#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import SolvePositionFK, SolvePositionFKRequest

def fk_service_client():
    service_name = "ExternalTools/right/PositionKinematicsNode/FKService"
    fk_service_proxy = rospy.ServiceProxy(service_name, SolvePositionFK)
    fk_request = SolvePositionFKRequest()
    joints = JointState()
    # YOUR CODE HERE
<<<<<<< HEAD:lab5/src/ik/src/fk.py
    joints.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
    joint_input = [float(angle) for angle in input('Enter a comma-separated list of 7 joint angles:\n').split(', ')]
=======
    joints.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
    joint_input = []
>>>>>>> aa9b04b6cf97288181c99efd3651a32a1e76f75a:lab5/fk.py
    joints.position = joint_input
    # Add desired pose for forward kinematics
    fk_request.configuration.append(joints)
    # Request forward kinematics from base to "right_hand" link
    fk_request.tip_names.append('right_hand')

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = fk_service_proxy(fk_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 

    # Check if result valid
    if (response.isValid[0]):
        rospy.loginfo("SUCCESS - Valid Cartesian Solution Found")
        rospy.loginfo("\nFK Cartesian Solution:\n")
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)
    else:
        rospy.logerr("INVALID JOINTS - No Cartesian Solution Found.")
        return 

    print('Final end effector position: ', response)
    return True


def main():
    rospy.init_node("fk_service_client")

    fk_service_client()

if __name__ == '__main__':
    main()
