<<<<<<< HEAD
#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    br.sendTransform(object_transform)

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    br.sendTransform(robot_transform)
 
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    br.sendTransform(camera_transform)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
=======
#!/usr/bin/env python  
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    rm1 = tf.transformations.euler_matrix(0.79, 0.0, 0.79)
    t1 = np.dot(rm1, np.array([0,1,1,1]).T)
    # print rm2
    q1 = tf.transformations.quaternion_from_euler(0.79, 0, 0.79)
    object_transform.transform.rotation.x = q1[0]
    object_transform.transform.rotation.y = q1[1]
    object_transform.transform.rotation.z = q1[2]
    object_transform.transform.rotation.w = q1[3]
    object_transform.transform.translation.x = t1[0]
    object_transform.transform.translation.y = t1[1]
    object_transform.transform.translation.z = t1[2]
    br.sendTransform(object_transform)

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    rm2 = tf.transformations.euler_matrix(0,0,1.5)
    t2 = np.dot(rm2, np.array([0,-1,0,1]).T)
    q2 = tf.transformations.quaternion_about_axis(1.5, (0, 0, 1))
    robot_transform.transform.rotation.x = q2[0]
    robot_transform.transform.rotation.y = q2[1]
    robot_transform.transform.rotation.z = q2[2]
    robot_transform.transform.rotation.w = q2[3]
    robot_transform.transform.translation.x = t2[0]
    robot_transform.transform.translation.y = t2[1]
    robot_transform.transform.translation.z = t2[2]
    
    br.sendTransform(robot_transform)
    
    tm2 = np.copy(rm2)
    tm2[:,3] = t2
    tm3 = np.identity(4)
    tm3[:,3] = np.array([0, 0.1, 0.1, 1]).T
    # t1_rev = -t1
    # t1_rev[3]=1
    # new_point = np.dot(t3, np.dot(rm2, t1_rev))
    # new_point[:3]*= -1
    # print new_point
    tm3_rev = np.copy(tm3)
    tm3_rev[:,3] = np.array([0,-0.1,-0.1,1]).T
    tm2_rev = tf.transformations.inverse_matrix(tm2)
    n_p = np.dot(tm3_rev, np.dot(tm2_rev, t1))
    print n_p
    rot_ax = np.cross(np.array([1,0,0]).T, n_p[:3])
    print rot_ax
    # normed_rot_ax = rot_ax/np.sqrt(np.dot(rot_ax.T,rot_ax))
    alpha = np.arccos(np.dot(np.array([1,0,0]).T, n_p[:3])/np.sqrt(np.dot(n_p[:3].T, n_p[:3])))
    print alpha
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    camera_transform.transform.translation.x = 0
    camera_transform.transform.translation.y = 0.1
    camera_transform.transform.translation.z = 0.1
    q3 = tf.transformations.quaternion_from_euler(0, 1.5+np.arctan2(n_p[1], n_p[0]), 1.5+np.arctan2(n_p[2], n_p[0]),'sxzy')
    # q3 = tf.transformations.quaternion_from_euler(0, 0, 0)
    q3 = tf.transformations.quaternion_about_axis(alpha,rot_ax)
    camera_transform.transform.rotation.x = q3[0]
    camera_transform.transform.rotation.y = q3[1]
    camera_transform.transform.rotation.z = q3[2]
    camera_transform.transform.rotation.w = q3[3]
    br.sendTransform(camera_transform)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
>>>>>>> 14b9b4c3924666c12a1e287398368ceb37278cd6
