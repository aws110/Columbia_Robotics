#!/usr/bin/env python
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

def callback(data):
    rospy.loginfo(data)
    pub = rospy.Publisher('sum', Int16, queue_size=10)
    total = data.a + data.b
    rospy.loginfo(total)
    pub.publish(total)
    
def listener():
    rospy.init_node('solution')
    rospy.Subscriber("two_ints", TwoInts, callback)

    rospy.spin()

if __name__=="__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
