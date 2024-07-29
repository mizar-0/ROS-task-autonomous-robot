#!/usr/bin/env python3

import rospy

#the contoller node of ros pid package accepts Float64 messages
from std_msgs.msg import Float64        
from sensor_msgs.msg import LaserScan

def deviation(scan : LaserScan):
    ranges = scan.ranges
    range_max = scan.range_max
    range_min = scan.range_min


    dL = ranges[0]
    #ranges[0] and ranges[-1] corresponds to minimum and maximum scan angle for the laser
    dR = ranges[-1]                 

    #these variable values must be preserved through successive callbacks
    global old_dL                   
    global old_dR


    #IGNORING VALUES OUTSIDE RANGE

    '''   THIS CODE DOESNT WORK FOR SOME REASON
    if dL in [float('inf'), float("-inf")]:                  
        dL = old_dL

    if dR in [float('inf'), float("-inf")]:
        dR = old_dR
    '''

    if dL >=range_max or dL <= range_min:
        dL = old_dL 
    
    if dR >=range_max or dR <= range_min:
        dR = old_dR
    
    deviation = dL-dR
    
    old_dL = dL
    old_dR = dR

    #deviation from the centre is calculated as the difference of distances to each wall
                                        
    #rospy.loginfo("deviation: " + str(deviation))
    
    msg = Float64()
    msg.data = deviation
    pub.publish(msg)

    
if __name__ == "__main__":
    rospy.init_node("deviation_calculator")
    old_dev=0
    sub = rospy.Subscriber("laser_scan", LaserScan, callback=deviation)
    pub = rospy.Publisher("deviation", Float64, queue_size=10)

    #the latch=True attribute keeps the message on the topic until another message is published to it
    #the alternative to this would be to include pub_sp.publish() in the callback
    pub_sp = rospy.Publisher("setpoint", Float64, queue_size=10, latch=True)            
    setpoint = Float64()                                                                
    setpoint.data = 0
    pub_sp.publish(setpoint)

    rospy.loginfo("Deviation node is live")
    rospy.spin()
