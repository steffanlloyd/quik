"""
@file quik_service_helpers.py
@author Steffan Lloyd (steffan.lloyd@nibio.no)
@brief Defines several python helper functions to make and parse service requests to
the services defined by the node in this package, ros_kinematics_service_node, which defines
fk_service (for forward kinematics), ik_service (for inverse kinematics), and 
jacobian_service (for computing the jacobian).

@date 2024-11-23

@copyright Copyright (c) 2024
"""
from quik.srv import FKService, IKService, JacobianService
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

BREAKREASON_MAP = {
    0: 'Tolerance reached',
    1: 'Minimum step size is reached',
    2: 'Max iterations reached',
    3: 'Gradient failed to improve'
}

def fk_make_request(q):
    """ Makes a FKService request from a numpy array q"""
    req = FKService.Request() 
    req.q = q.tolist()
    return req

def fk_parse_response(response):
    """ Parses an FKService response into numpy arrays for quat and d"""
    if response == None:
        return {'quat', None, 'd', None}

    return {
        'quat': np.array([response.pose.orientation.x, response.pose.orientation.y, response.pose.orientation.z, response.pose.orientation.w]),
        'd': np.array([response.pose.position.x, response.pose.position.y, response.pose.position.z])
    }

def jacobian_make_request(q):
    """ Makes a JacobianService request from a numpy array q"""
    req = JacobianService.Request() 
    req.q = q.tolist()
    return req

def jacobian_parse_response(response):
    """ Parse the response data from the jacobian service """
    if response == None:
        return None
    jacobian_flat = np.array(response.jacobian)
    dof = len(jacobian_flat) // 6
    return jacobian_flat.reshape((6, dof))

def ik_make_request(quat_des, d_des, q_0):
    """ Makes an IKService request from numpy arrays for desired orientation
        quat_des, position d_des, and intial joint angle guesses q_guess
    """
    quat = Quaternion()
    quat.x = quat_des[0]; quat.y = quat_des[1]; quat.z = quat_des[2]; quat.w = quat_des[3]
    pnt = Point()
    pnt.x = d_des[0]; pnt.y = d_des[1]; pnt.z = d_des[2]
    pose = Pose()
    pose.orientation = quat; pose.position = pnt
    req = IKService.Request()
    req.target_pose = pose
    req.q_0 = q_0.tolist()
    return req

def ik_parse_response(response):
    """ Parses an IKService response into a numpy dictionary """

    if response == None:
        return {
        'q_star': None,
        'e_star': None,
        'iter': None,
        'break_reason': None,
        'success': False
    }
    return {
        'q_star': np.array(response.q_star),
        'e_star': np.array(response.e_star),
        'iter': response.iter,
        'break_reason': BREAKREASON_MAP.get(response.break_reason, 'Unknown break reason'),
        'success': response.success
    }