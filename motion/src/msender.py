#!/usr/bin/env python


import rospy
from stepper.msg import MoveCommand
from random import randint
    
dt = .1 # Length of segments, in seconds
    
def send_motion():
    
    v_max =  rospy.get_param("/stepper/v_max")
    
    x_max = v_max*dt
    
    pub = rospy.Publisher('/stepper/cmd', MoveCommand, queue_size=10)
    rospy.init_node('msender', anonymous=True)
    rate = rospy.Rate(5) # 10hz

    i = 0;
    l = [0, 1500, 1000, -2000,1000, -500, 0]
    while not rospy.is_shutdown():
        rate.sleep()
        x = [ l[(i+j)%len(l)]*10 for j in range(6) ]
        print(i, x)
        m = MoveCommand(movetype=MoveCommand.ABSOLUTE, t=.1, x=x)
        pub.publish(m)
        
        i += 1
        if(i == len(l)*2):
            rospy.signal_shutdown("")
            break

        
        
if __name__ == '__main__':
    try:
        send_motion()
    except rospy.ROSInterruptException:
        pass