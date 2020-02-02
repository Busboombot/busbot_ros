#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from time import sleep
from math import copysign, ceil, sqrt


v_max = [15e3 for _ in range(6)]
vec_v_max = sqrt(sum( e**2 for e in v_max ))


def callback(data, memo):
    
    m = sqrt(sum( e**2 for e in data.axes ))
    
    print m
    

def listener():

    rospy.init_node('motion')

    #rate = rospy.Rate(10) # In messages per second

    pub = None # rospy.Publisher('motion/segment_command', MotionCommand, queue_size=4)

    memo = {
        'pub': pub, 
        'last_message' : None,
        'freq_map': None, 
        'last_velocities': [0]*6
    }

    rospy.Subscriber("joy", Joy, callback, memo)

    #rospy.Timer(rospy.Duration(.05), lambda event: timed_callback(event, memo))

    rospy.spin()

if __name__ == '__main__':
    listener()