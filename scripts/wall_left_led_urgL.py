#!/usr/bin/env python
import rospy,copy, math, time
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from sensor_msgs.msg import LaserScan

class WallAround():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist,queue_size=1)

        self.led_values=LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.led_callback)
        self.range_values = [0] * 726
        rospy.Subscriber("scan", LaserScan, self.urg_callback)

    def led_callback(self,m):
        self.led_values = m

    def urg_callback(self,m):
        self.range_values = m.ranges
        
    def wall_front(self, ls): 
        return ls.left_forward > 50 or ls.right_forward > 50

    def too_right(self, ls):
        return ls.right_side > 50

    def too_left(self, ls):
        return ls.left_side > 50


    def run(self):
        rate = rospy.Rate(10)
        data = Twist()

        data.linear.x = 0.3
        data.angular.z = 0.0
        
        while not rospy.is_shutdown():
            if self.wall_front(self.led_values):
                data.angular.z = - math.pi
            elif self.too_right(self.led_values):
                data.angular.z = math.pi
            elif self.too_left(self.led_values):
                data.angular.z = - math.pi
            else:
                e = 50 - self.led_values.left_side
                data.angular.z = e * math.pi / 180.0

            # rospy.loginfo("Left Z |  %.2f  %.2f",
            #               self.led_values.left_side,
            #               data.angular.z)

            print(list(self.range_values))

            self.cmd_vel.publish(data)
            rate.sleep()


            
if __name__ == '__main__':
    rospy.init_node('wall_around')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on',Trigger).call()



    WallAround().run()
    

    
