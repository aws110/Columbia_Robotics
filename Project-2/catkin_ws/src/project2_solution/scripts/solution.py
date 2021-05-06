#!/usr/bin/env python

"""
This is a skeleton code.
"""

import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg


def message_from_transform(T):
   
    msg = geometry_msgs.msg.Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    tr = tf.transformations.translation_from_matrix(T)

    msg.translation.x = tr[0]
    msg.translation.y = tr[1]
    msg.translation.z = tr[2]

    msg.rotation.x = q[0]
    msg.rotation.y = q[1]
    msg.rotation.z = q[2]
    msg.rotation.w = q[3]
    return msg

#########################################################################
# Code to calculate angle between two n dimensional vectors was taken directly from the link below.
# https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def matrix_by_vector_multiplication(matrix,vector):
    """Multiplication of matrix by vector"""
    vector.append(1)
    return [sum([vector[x]*matrix[n][x] for x in range(len(vector))]) for n in range(len(matrix))]


############################################################################

def publish_transforms():
    
    ### Object Section ###########################
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    
    #Calculate Transformation matrix of object for later calculations
    R1_matrix = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79))
    T1_matrix = tf.transformations.translation_matrix((0.0, 1.0, 1.0))
    #Full Transform Matrix
    Trans1_matrix = tf.transformations.concatenate_matrices(R1_matrix, T1_matrix)  
    
    #Apply rotation and translation info to object
    object_transform.transform = message_from_transform(Trans1_matrix)
    br.sendTransform(object_transform)
    ### End Object Section ########################
    
    ### Robot Section #################################
    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    
    #Calculate the rotation and translation matrices for later calculations
    R2_matrix = tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(1.5,(0.0, 0.0, 1.0)))
    T2_matrix = tf.transformations.translation_matrix((0.0, -1.0, 0.0))  
    #Full Transform Matrix
    Trans2_matrix = tf.transformations.concatenate_matrices(R2_matrix, T2_matrix)                                                 
    
    #Quaternion of the rotation
    q2 = tf.transformations.quaternion_about_axis(1.5,(0.0,0.0,1.0))
    
    #Apply rotation and translation to the transform of the robot
    robot_transform.transform = message_from_transform(Trans2_matrix)
    
    br.sendTransform(robot_transform)
    ### End Robot Section #########################################
    
    ### Camera Section #####################################
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    
    #Translation Matrix
    T3_matrix = tf.transformations.translation_matrix((0.0, 0.1, 0.1))
    
    ####Rotation Calculation ############################
    run_once = False
    if not run_once:
        run_once = True
        R3_matrix = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0))
        Trans3_matrix = tf.transformations.concatenate_matrices(T3_matrix, R3_matrix)
    
        # origin of object_frame in world
        p_object_world = tf.transformations.translation_from_matrix(Trans1_matrix)
        p_list = p_object_world.tolist()
    
        # p_object_robot in robot_frame
        p_object_robot = matrix_by_vector_multiplication(tf.transformations.inverse_matrix(Trans2_matrix), p_list)
        #create homogenous array
        p_object_robot = p_object_robot[:(len(p_object_robot) - 1)]
    
        #p_object_robot in camera fram
        p_object_camera = matrix_by_vector_multiplication(tf.transformations.inverse_matrix(Trans3_matrix), p_object_robot)
        #create homogenous array
        p_object_camera = p_object_camera[:(len(p_object_camera) - 1)]
        
        #x-axis
        x = [1,0,0]
        
        #angle between camera and x-axis
        angle = angle_between(x, p_object_camera)
        
        normal = np.cross(x, p_object_camera) 
    
        
    R3_matrix = tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(angle, normal))
    
    Trans3_matrix = tf.transformations.concatenate_matrices(T3_matrix, R3_matrix)
    
    camera_transform.transform = message_from_transform(Trans3_matrix)
    
    br.sendTransform(camera_transform)
    ### End Camera Section #####################################

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
