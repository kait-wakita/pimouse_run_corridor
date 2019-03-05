#!/usr/bin/env python
import rospy, copy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import LaserScan


class WallAround():
    def __init__(self):
        rospy.Subscriber("scan", LaserScan, self.callback)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist,queue_size=1)
        self.front = 0.0
        self.right = 0.0
        self.left = 0.0
        
        
    def callback(self, m):
        self.front = self.range_v(m, m.ranges[384])
        self.right = self.range_v(m, m.ranges[256])
        self.left = self.range_v(m, m.ranges[512])
        print(m.ranges[256:512])
        
#        rospy.loginfo("left front rihgt | %.2f  %.2f  %.2f",
#                      self.left, self.front, self.right)



    def wall_front(self): 
        return self.front < 0.2

    def too_right(self):
        return self.right < 0.1

    def too_left(self):
        return self.left < 0.1

    def no_left(self):
        return self.left > 1.0

    def range_v(self, d, r):
        if r < d.range_min:
            return(0.0)
        elif r > d.range_max:
            return(d.range_max)
        elif math.isnan(r):
            return(d.range_max)
        else:
            return(r)
 

    def run(self):
        rate = rospy.Rate(100)
        data = Twist()

        data.linear.x = 0.2
        data.angular.z = 0.0
        
        while not rospy.is_shutdown():

            if self.wall_front():
               data.linear.x = 0.0
            else:
               data.linear.x = 0.2

            if self.too_right():
                data.angular.z = math.pi
            elif self.too_left():
                data.angular.z = - math.pi
            elif self.no_left():
                data.angular.z = 0.4 * math.pi
            else:
                e = self.left - 0.2
                data.angular.z = e * math.pi / 2.0
            # rospy.loginfo("Left Front Right Vx Az | %.2f %.2f %.2f  %.2f %.2f",
            #               self.left, self.front, self.right,
            #               data.linear.x, data.angular.z)




            self.cmd_vel.publish(data)
            rate.sleep()



            
            
if __name__ == '__main__':
    rospy.init_node('wall_around')


    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()
    WallAround().run()
    

    
