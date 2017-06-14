#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

class Summer(object):
	def __init__(self):
		self.sub = rospy.Subscriber('two_ints', TwoInts, self.callback)
		self.pub = rospy.Publisher('sum', Int16, queue_size=10)
		self.listener()

		
	def listener(self):
		rospy.init_node('two_int_listener',anonymous=True)

		rospy.spin()

	def callback(self,msg):
		rospy.loginfo(rospy.get_caller_id()+'I heard %i %i', msg.a, msg.b)
		summe = msg.a + msg.b
		self.pub.publish(summe)
 	


if __name__=="__main__":
 	try:
   		Summer()
	except rospy.ROSInterruptException:
		pass