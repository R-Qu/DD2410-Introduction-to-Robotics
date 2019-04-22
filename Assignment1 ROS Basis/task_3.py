#!/usr/bin/env python
import rospy
import math
from ras_lab1_msgs.msg import PWM           #publisher
from ras_lab1_msgs.msg import Encoders      #subscriber
from ras_lab1_msgs.msg import ADConverter
from std_msgs.msg import *

ch1 = 0
ch2 = 0
pwm1 = 70
pwm2 = 70
delta_encoder1 = 0
delta_encoder2 = 0
int_error1 = 0
int_error2 = 0


def callback_encoder(msg):
    global delta_encoder1, delta_encoder2
    delta_encoder1 = msg.delta_encoder1
    delta_encoder2 = msg.delta_encoder2
    
def callback_ADC(msg):
    global ch1, ch2
    ch1 = msg.ch1
    ch2 = msg.ch2
    rospy.loginfo("ch1:{},ch2:{}".format(ch1,ch2))

def main():
    rospy.init_node("cartesian_controller")
    rospy.Subscriber("kobuki/encoders", Encoders, callback_encoder)
    rospy.Subscriber("kobuki/adc", ADConverter, callback_ADC)

    pub = rospy.Publisher("kobuki/pwm", PWM, queue_size = 10)
    rate = rospy.Rate(10)
    header = std_msgs.msg.Header()
    pub.publish(header,70, 70)

    
    Kp = 2
    Ki = 1
    alpha = 0.1
    b = 0.115
    r = 0.0325
    while not rospy.is_shutdown():

        # desired
        v = 2
        w = alpha*(ch1-ch2)
        desired_1 = v+b*w
        desired_2 = v-b*w

        # estimated
        estimated_1 = (delta_encoder1*2*math.pi*10)/360
        estimated_2 = (delta_encoder2*2*math.pi*10)/360

        error1 = desired_1 - estimated_1
        error2 = desired_2 - estimated_2
        global int_error1, int_error2
        int_error1 = int_error1 + error1 * 0.1
        int_error2 = int_error2 + error2 * 0.1
        global pwm1, pwm2
        pwm1 = pwm1 + Kp*error1 + Ki*int_error1
        pwm2 = pwm2 + Kp*error2 + Ki*int_error2
        pub.publish(header,pwm1, pwm2)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()