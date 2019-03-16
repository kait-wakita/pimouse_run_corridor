#!/usr/bin/env python
import rospy, math
from sensor_msgs.msg import LaserScan

def callback(d):

# #    rospy.loginfo(rospy.get_caller_id()+": I heard %s", d.ranges[50])

# #     rospy.loginfo("R50:%s R60:%s", d.ranges[50],d.ranges[60])

    # rospy.loginfo("size %s", len(d.ranges))
    rospy.loginfo("angle %.2f...%.2f(%.4frad,%sp), range:%.2f...%.2f, time:%.2f  |  %.2f %.2f %.2f %.2f %.2f",
                  d.angle_min, d.angle_max, d.angle_increment, len(d.ranges),
                  d.range_min, d.range_max, d.scan_time,
                  range_v(d, d.ranges[128]),
                  range_v(d, d.ranges[256]),
                  range_v(d, d.ranges[384]),
                  range_v(d, d.ranges[512]),
                  range_v(d, d.ranges[640]))



def range_v(d, r):
    if r < d.range_min:
        return(0.0)
    elif r > d.range_max:
        return(d.range_max)
    elif math.isnan(r):
        return(d.range_max)
    else:
        return(r)
        
    
def listener():

    rospy.init_node('range-listener', anonymous=True)

    sub = rospy.Subscriber("scan", LaserScan, callback)

    # simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
    
