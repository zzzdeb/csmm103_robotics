#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

def listener():
  	rospy.init_node('two_int_listener',anonymous=True)
 	sub = rospy.Subscriber('two_ints', TwoInts,callback)
   
   	rospy.spin()

def callback(msg):
 	rospy.log_info(rospy.get_caller_id()+'I heard %i', msg.a+msg.b)
 	


if __name__=="__main__":
 	try:
   		listener()
	except rospy.ROSInterruptException:
		pass