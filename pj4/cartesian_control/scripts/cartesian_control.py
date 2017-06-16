#!/usr/bin/env python

import math
import numpy as np
import time
from threading import Thread, Lock

import rospy
import tf

from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

def S_matrix(w):
    S = np.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S

# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    """
    
    joint_transforms: a list containing the transforms of all the joints with respect to the base frame. In other words, joint_transforms[i] will contain the transform from the base coordinate frame to the coordinate frame of joint i.
    b_T_ee_current: current transform from the base frame to the end-effector
    b_T_ee_desired: desired transform from the base frame to the end-effector

In addition, the parameters below are relevant only if you are also choosing to implement null-space control:

    red_control: boolean telling you when the Cartesian Controller should take into account the secondary objective for null-space control. This is only set to True when the user interacts with the control marker dedicated to the first joint of the robot.
    q_current: list of all the current joint positions
    q0_desired: desired position of the first joint to be used as the secondary objective for null-space control. Again, the goal of the secondary, null-space controller is to make the value of the first joint be as close as possible to q0_desired, while not affecting the pose of the end-effector.

    """
    num_joints = len(joint_transforms)
    dq = np.zeros((num_joints,1))
    #-------------------- Fill in your code here ---------------------------  
    ee_T_b_current = tf.transformations.inverse_matrix(b_T_ee_current)
    
    dx_matrix = np.dot(ee_T_b_current, b_T_ee_desired)
    # dx = rotation_from_matrix(dx_matrix)

    proportion = 0.1
    dx_vel_trans = proportion * tf.transformations.translation_from_matrix(dx_matrix)
    dx_vel_rot_angle, dx_vel_rot_ax = rotation_from_matrix(dx_matrix)
    dx_vel_rot_angle *= proportion
    # dx_vel = np.array([dx_vel]).T
    # if dx_vel>0.5:
    #     dx_vel = 0.5
    for i in range(len(dx_vel_trans)):
        if dx_vel_trans[i]>0.1:
            dx_vel_trans[i]=0.1
    if dx_vel_rot_angle>0.1:
        dx_vel_rot_angle = 0.1
    dx_vel_matrix = tf.transformations.rotation_matrix(dx_vel_rot_angle, dx_vel_rot_ax)
    dx_vel_matrix[:3,3] = dx_vel_trans
    v_matrix = np.dot(b_T_ee_current, dx_vel_matrix)
    v = np.zeros(6)
    v[:3] = tf.transformations.translation_from_matrix(v_matrix)
    v[3:] = tf.transformations.euler_from_matrix(v_matrix)
    v = np.array([v]).T

    J = np.zeros((6,num_joints))
    J_i = np.zeros(6)
    for (i, joint_t) in enumerate(joint_transforms):
        # J_i = calculate from joint_t and b_T_ee_current
        inv = tf.transformations.inverse_matrix(joint_t)
        atb = tf.transformations.translation_from_matrix(inv)
        [x, y, z] = atb.T
        s_t_ab = np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
            ])
       
        J_i[:3] = np.dot(-inv[:3,:3], s_t_ab)[:,2]
        J_i[3:] = np.array([inv[:3,2]])
        # J.concatinate J_temp[:,5]
        J[:,i] = J_i

    epsilon = 0.1
    J_plus = np.linalg.pinv(J, epsilon)

    if red_control:
        dq_r = np.zeros(num_joints)
        dq += dq_r
    dq += J_plus.dot(v)

    #scaling
    for i in range(len(dq)):
        if dq[i]>0.1:
            dq[i]=0.1
    #----------------------------------------------------------------------
    return dq
    
def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                  t.translation.y,
                                                  t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = np.dot(trans,rot)
    return T

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = np.array(matrix, dtype=np.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = np.linalg.eig(R33.T)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = np.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = np.linalg.eig(R)
    i = np.where(abs(np.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (np.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class CartesianControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        #Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()        
        
    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.velocity = dq
        else:            
            msg.velocity = np.zeros(7)
        self.mutex.release()
        self.pub_vel.publish(msg)
        
    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = np.array([0,0,1])
        x = np.array([1,0,0])
        dot = np.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = np.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = np.dot(trans_matrix, rot_matrix)
            current_joint_T = np.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = np.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 np.asarray(current_joint.axis))
                next_link_T = np.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)
        
if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
