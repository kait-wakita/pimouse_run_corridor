#!/usr/bin/env python
import rospy, copy, math, time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import LaserScan


class WallAround():
    def __init__(self):
        rospy.Subscriber("scan", LaserScan, self.callback)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
        self.left = 0.09
        self.front_l = 0.09
        self.front = 0.09
        self.front_r = 0.09
        self.right = 0.09
        
        
    def callback(self, m):
        self.left = self.range_v(m, m.ranges[512], self.left)
        self.front_l = self.range_v(m, m.ranges[414], self.front_l)
        self.front = self.range_v(m, m.ranges[384], self.front)
        self.front_r = self.range_v(m, m.ranges[354], self.front_r)
        self.right = self.range_v(m, m.ranges[256], self.right)
#        print(m.ranges[256:512])
        
#        rospy.loginfo("left front rihgt | %.2f  %.2f  %.2f",
#                      self.left, self.front, self.right)



    def wall_front(self): 
        return self.front < 0.2 or self.front_r < 0.2

    def too_right(self):
        return self.right < 0.2
    
    def too_left(self):
        return self.left < 0.2

    def no_left(self):
        return self.left > 1.0

    
    def range_v(self, m, r, r1):
        if r < m.range_min:
            return(r1)
        elif r > m.range_max:
            return(r1)
        elif math.isnan(r):
            return(r1)
        else:
            return(r)
 
    def range_err(self, m, r):
        if r < m.range_min:
            return(1)
        elif r > m.range_max:
            return(1)
        elif math.isnan(r):
            return(1)
        else:
            return(0)
 

    def run(self):
        rate = rospy.Rate(10)     # 10Hz
        data = Twist()

        data.linear.x = 0.2
        data.angular.z = 0.0
        
        while not rospy.is_shutdown():

            if self.wall_front():
               data.linear.x = 0.0
               time.sleep(2.0)
            else:
               data.linear.x = 0.2

            if self.too_right():
                data.angular.z = math.pi
            elif self.too_left():
                data.angular.z = - math.pi
            elif self.no_left():
                data.angular.z = 0.4 * math.pi
            else:
                e = self.left - 0.3
                data.angular.z = e * math.pi / 1.2
            rospy.loginfo("Left Front Right Vx Az | %.2f %.2f %.2f %.2f %.2f  %.2f %.2f",
                           self.left, self.front_l, self.front, self.front_r, self.right,
                           data.linear.x, data.angular.z)

            self.cmd_vel.publish(data)
            rate.sleep()



            
            
if __name__ == '__main__':
    rospy.init_node('wall_around')

    time.sleep(5.0)

    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()

    WallAround().run()
    

    
