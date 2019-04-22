#!/usr/bin/env python
import rospy
from ras_lab1_msgs.msg import PWM
from std_msgs.msg import Header
from std_msgs.msg import Int32

def open_loop_controller():
  pub = rospy.Publisher('kobuki/pwm', PWM)
  rospy.init_node('open_loop_controller', anonymous=True)
  rate = rospy.Rate(10) 
  
  while not rospy.is_shutdown():
    msg = PWM()
    h = Header()
    h.stamp = rospy.Time.now()  
    msg.header = h   
    msg.PWM1 = 255
    msg.PWM2 = 255
    rospy.loginfo(msg)
    pub.publish(msg)
    rate.sleep()
    
if __name__ == '__main__':
  open_loop_controller()