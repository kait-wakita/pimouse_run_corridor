#!/usr/bin/env python
import rospy, math
from sensor_msgs.msg import LaserScan

class RangeListen():
    def __init__(self):
        rospy.Subscriber("scan", LaserScan, self.callback)
        

    def callback(self, m):
        rospy.loginfo("angle %.2f...%.2f(%.4frad,%sp), range:%.2f...%.2f, time:%.2f  |  %.2f %.2f %.2f %.2f %.2f",
                      m.angle_min, m.angle_max, m.angle_increment, len(m.ranges),
                      m.range_min, m.range_max, m.scan_time,
                      self.range_v(m, m.ranges[128]),
                      self.range_v(m, m.ranges[256]),
                      self.range_v(m, m.ranges[384]),
                      self.range_v(m, m.ranges[512]),
                      self.range_v(m, m.ranges[640]))

        
    def range_v(self, d, r):
        if r < d.range_min:
            return(0.0)
        elif r > d.range_max:
            return(d.range_max)
        elif math.isnan(r):
            return(d.range_max)
        else:
            return(r)
 


if __name__ == '__main__':
    rospy.init_node('range-listener', anonymous=True)
    
    RangeListen()
    
    # simply keeps python from exiting until this node is stopped
    rospy.spin()

    
