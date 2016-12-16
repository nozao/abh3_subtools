#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from math import sin,cos,pi

from abh3.msg import abh3Vel
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class abh3Converter:
    def __init__(self):
#        rospy.loginfo("conv init")
        rospy.init_node('abh3_conv', anonymous=False)
        rospy.on_shutdown(self.shutdown)
#        rospy.loginfo("start conv")
        driver_item = rospy.get_param('~driver/item', 'abh3')
        driver_name = rospy.get_param('~driver/name', 'no1')

        self.subAB = rospy.Subscriber('/%s/%s/vin/ab' % (driver_item, driver_name), abh3Vel, self.abh3velAB_handler)
        self.pubAB_YX = rospy.Publisher('/%s/%s/vout/ab_yx' % (driver_item, driver_name), abh3Vel, queue_size=1)
        self.pubAB_XA = rospy.Publisher('/%s/%s/vout/ab_xa' % (driver_item, driver_name), Twist, queue_size=1)

        self.subYX = rospy.Subscriber('/%s/%s/vin/yx' % (driver_item, driver_name), abh3Vel, self.abh3velYX_handler)
        self.pubYX_AB = rospy.Publisher('/%s/%s/vout/yx_ab' % (driver_item, driver_name), abh3Vel, queue_size=1)
        self.pubYX_XA = rospy.Publisher('/%s/%s/vout/yx_xa' % (driver_item, driver_name), Twist, queue_size=1)

        self.subXA = rospy.Subscriber('/%s/%s/vin/xa' % (driver_item, driver_name), Twist, self.abh3velXA_handler)
        self.pubXA_AB = rospy.Publisher('/%s/%s/vout/xa_ab' % (driver_item, driver_name), abh3Vel, queue_size=1)
        self.pubXA_YX = rospy.Publisher('/%s/%s/vout/xa_yx' % (driver_item, driver_name), abh3Vel, queue_size=1)

        self.rate = float(rospy.get_param('~machine/rateNum', 1.0)) / float(rospy.get_param('~machine/rateDen', 10.0))
        self.wheel = float(rospy.get_param('~machine/wheel', 0.11))
        self.width = float(rospy.get_param('~machine/width', 0.57))
        #+< for odometry calculate
        self.last_cmd = rospy.Time.now()
        self.o_rate = 10.0
        self.timeout = 1.0
        self.t_delta = rospy.Duration(0,1.0/self.o_rate)
        self.t_next = rospy.Time.now()+ self.t_delta
        self.base_width = self.width
        self.base_frame_id = 'base_link'
        self.odom_frame_id = 'odom_frame_id'
        self.v_left = 0                 # current setpoint velocity
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                     # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()    # time for determining dx/dy
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=5)

        self.odomBroadcaster = TransformBroadcaster()
        #+> for odometry calculate
    def shutdown(self):
        rospy.loginfo("conv shutdown")

        pass

    def abh3velAB_handler(self, msg):
#        rospy.loginfo("velAB_handler in")
        velYX = abh3Vel()
        velXA = Twist()

        velYX.velAY = (msg.velAY + msg.velBX) / 2
        velYX.velBX = msg.velAY - velYX.velAY

        velXA.linear.x = velYX.velAY / 60 * self.rate * self.wheel * pi
        velXA.angular.z = -velYX.velBX / 60 * self.rate * self.wheel * pi / (self.width / 2)
        #+< for odometry calculate
        self.last_cmd = rospy.Time.now()
        self.dx = velXA.linear.x        # m/s
        self.dr = velXA.angular.z       # rad/s
#        rospy.loginfo("delta,next,dx")
#        rospy.loginfo(self.t_delta)
#        rospy.loginfo(self.t_next)
#        rospy.loginfo("linear speed[dx:%d]",self.dx)
        self.update()
        #+> for odometry calculate
        self.pubAB_YX.publish(velYX)
        self.pubAB_XA.publish(velXA)
        #+< for odometry calculate
    def update(self):
#        rospy.loginfo("odom update")

        now = rospy.Time.now()
        if now > self.t_next:
#            rospy.loginfo("loop in")
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            x = cos(self.th)*self.dx*elapsed
            y = -sin(self.th)*self.dx*elapsed
            self.x += cos(self.th)*self.dx*elapsed
            self.y += sin(self.th)*self.dx*elapsed
            self.th += self.dr*elapsed
            # publish or perish
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th/2)
            quaternion.w = cos(self.th/2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

            if now > (self.last_cmd + rospy.Duration(self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0

            self.t_next = now + self.t_delta
        #+> for odometry calculate
#        rospy.loginfo("odom loopout")


    def abh3velYX_handler(self, msg):
        velAB = abh3Vel()
        velXA = Twist()

        velAB.velAY = msg.velAY + msg.velBX
        velAB.velBX = msg.velAY - msg.velBX

        velXA.linear.x = msg.velAY / 60 * self.rate * self.wheel * pi
        velXA.angular.z = -msg.velBX / 60 * self.rate * self.wheel * pi / (self.width / 2)

        self.pubYX_AB.publish(velAB)
        self.pubYX_XA.publish(velXA)

    def abh3velXA_handler(self, msg):
        velAB = abh3Vel()
        velYX = abh3Vel()

        velYX.velAY = msg.linear.x * 60 / (self.rate * self.wheel * pi)
        velYX.velBX = -msg.angular.z * 60 / (self.rate * self.wheel * pi) * (self.width / 2)

        velAB.velAY = velYX.velAY + velYX.velBX
        velAB.velBX = velYX.velAY - velYX.velBX

        self.pubXA_AB.publish(velAB)
        self.pubXA_YX.publish(velYX)


if __name__ == '__main__':
    try:
        rospy.loginfo("conv bootup")
        manager = abh3Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
            pass
