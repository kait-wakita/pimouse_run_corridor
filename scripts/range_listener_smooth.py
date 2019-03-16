#!/usr/bin/env python
import rospy, math
from sensor_msgs.msg import LaserScan

class RangeListen():
    def __init__(self):
        rospy.Subscriber("scan", LaserScan, self.callback)

        self.left = 1.0
        self.front_l = 1.0
        self.front = 1.0
        self.front_r = 1.0
        self.right = 1.0
        self.left = 1.0
        

    def callback(self, m):
        self.left = self.range_v(m, m.ranges[512], self.left)
        self.front_l = self.range_v(m, m.ranges[414], self.front_l)
        self.front = self.range_v(m, m.ranges[384], self.front)
        self.front_r = self.range_v(m, m.ranges[354], self.front_r)
        self.right = self.range_v(m, m.ranges[256], self.right)

        rospy.loginfo("angle %.2f...%.2f(%.4frad,%sp), range:%.2f...%.2f, time:%.2f  |  %.2f %.2f %.2f %.2f %.2f",
                      m.angle_min, m.angle_max, m.angle_increment, len(m.ranges),
                      m.range_min, m.range_max, m.scan_time,
                      self.left, self.front_l, self.front, self.front_r, self.right)

        
    def range_v(self, m, r, r1):
        return(r)
        if r < m.range_min:
            return(r1)
        elif r > m.range_max:
            return(r1)
        elif math.isnan(r):
            return(r1)
        else:
            return(r)
 


if __name__ == '__main__':
    rospy.init_node('range-listener', anonymous=True)
    
    RangeListen()
    
    # simply keeps python from exiting until this node is stopped
    rospy.spin()

    
