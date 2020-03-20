#!/usr/bin/env python

import numpy
import random
import sys
import math

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def discretize(self,r,s,step):
        
       
        d=numpy.subtract(r,s) 
        dt=numpy.linalg.norm(d)
        np=numpy.true_divide(d,dt*step)
        R=[]
        R.append(s)
        t=s
        i=1
        while i<math.floor(dt*step):
          t=numpy.add(np,t)
          R.append(t)
          i=i+1
        R.append(r)
        return R

    def is_segment_valid(self,r,s):
        R=self.discretize(r,s,10) 
        Q=[]
        for i in range(len(R)):
          if self.is_state_valid(R[i])==True:
             Q.append(R[i])
          else:
             break
      
        if len(Q)==0:
          return False
        else:
          return True,Q[-1]
        
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
        q_current=self.q_current
        a=ee_goal
        tr=tf.transformations.translation_matrix((a.translation.x,a.translation.y,a.translation.z))
        R=tf.transformations.quaternion_matrix((a.rotation.x,a.rotation.y,a.rotation.z,a.rotation.w))
        T_goal=numpy.dot(tr,R)
        q_goal=self.IK(T_goal)
        
        RRT_List=[]
        RRT_List.append(q_current)
        index=[0]
        
        p=[]
        begin = rospy.get_rostime().secs
        now = rospy.get_rostime().secs
        #############RRT List####################
        while now-begin<300:
         shortest=9999999
         r=numpy.random.uniform(low=-math.pi,high=math.pi,size=self.num_joints)
         
         for i in range(len(RRT_List)):
            
            if self.is_segment_valid(r,RRT_List[i])==False:
              break
            else:
              [z,p]=self.is_segment_valid(r,RRT_List[i])
              dist=numpy.linalg.norm(p)
              
              if dist<shortest: #shortest length
                 shortest=dist
                 j=i
            parent_id=i+1
##############if_segment_valid add to RRT List and append parent index###############
         if len(p)!=0:
          
          RRT_List.append(p)
          index.append(j) 
         
          if self.is_segment_valid(p,q_goal)!=False:
              [z,t]=self.is_segment_valid(p,q_goal)
            
              checker=0
             
              for i in range(len(t)):
                if abs(t[i]-p[i])<0.05:
                  checker=checker+1
  
              if checker==7:
                RRT_List.append(q_goal)
                index.append(parent_id)
                
                break
         now=rospy.get_rostime().secs
        
       ############trajectory###############
 
        pos_list=[]
        pos_list.append(q_goal)
        z=index[-1] #last
        while z!=0:
         pos_list.append(RRT_List[z])
         z=index[z]
        pos_list.append(q_current)
        pos_list.reverse()
  
        #########Shorten##################################################################################
        pos_l=[]
        o=pos_list[0]
        pos_l.append(o)
        l=0
        while True:
         
         if l>=(len(pos_list)-2):
           break
         if self.is_segment_valid(pos_list[l+2],pos_list[l])==False:
           pos_l.append(pos_list[l+1])
           l=l+1
       
         else:
           [z,u]=self.is_segment_valid(pos_list[l+2],pos_list[l])
           a=pos_list[l+2]
           checker=0
           for i in range(len(u)):
              if abs(u[i]-a[i])<0.01:
                 checker=checker+1
  
           if checker==7:
                pos_l.append(a)
                l=l+2
           else:
                pos_l.append(pos_list[l+1])
                l=l+1
         
        
        checker=0
        ql_1=pos_list[-1]
        ql_2=pos_l[-1]
        for i in range(len(ql_1)):
          if abs(ql_1[i]-ql_2[i]==0):
            checker=checker+1
        if checker!=7:
         pos_l.append(ql_1)
        pos_list=pos_l
              
        #########Discretization########
        q_list=[]
        q_list.append(pos_list[0])
        for i in range(len(pos_list)-1):

          Z=self.discretize(pos_list[i+1],pos_list[i],3)
           
          for j in range(len(Z)):
             q_list.append(Z[j])
###############Trajectory Execution##############
        joint_trajectory_msg=JointTrajectory()
        for i in range(len(q_list)):
          joint_trajectorypoint_msg=JointTrajectoryPoint() #publish
          joint_trajectorypoint_msg.positions=list(q_list[i])
          joint_trajectory_msg.points.append(joint_trajectorypoint_msg)
          
        joint_trajectory_msg.joint_names=self.joint_names
        self.pub.publish(joint_trajectory_msg)


         
           
          
######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

