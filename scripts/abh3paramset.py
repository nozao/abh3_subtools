#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from abh3.srv import abh3Com

class abh3StatusDumper:
    def __init__(self):

        rospy.init_node('abh3_paramconfig', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.driver_item = rospy.get_param('~driver/item', 'abh3')
        self.driver_name = rospy.get_param('~driver/name', 'main')

        rospy.wait_for_service('/%s/%s/srv' % (self.driver_item, self.driver_name))
        self.service("SERVO OFF OFF")
        self.dump("CS")
        self.dump("RV")
        self.dump("RC")
        self.dump("TU")
        self.dump("US")
        self.dump("TD")
        self.dump("DS")
        self.dump("KP")
        self.dump("KI")
        print 'Would you like to start setting A and B to the same value?'
        key_input = raw_input('(y/n): ')
        if 'y' in key_input :
            self.setSend("CS",(raw_input('CS(指令テーブル番号)[0]>>')))
            self.setSend("RV",(raw_input('RV(速度指令・速度制限)[0]>>')))
            self.setSend("RC",(raw_input('RC(電流指令・電流制限)[200]>>')))
            self.setSend("TU",(raw_input('TU(加速時定数)>>[1]')))
            self.setSend("US",(raw_input('US(加速S時定数)>>[5]')))
            self.setSend("TD",(raw_input('TD(減速時定数)>>[1]')))
            self.setSend("DS",(raw_input('DS(減速S時定数)>>[5]')))
            self.setSend("KP",(raw_input('KP(比例ゲイン)>>[500]')))
            self.setSend("KI",(raw_input('KI(積分ゲイン)>>[250]')))

    def setSend(self,pre_cmd,set_val):
        try:
            service = rospy.ServiceProxy('/%s/%s/srv' % (self.driver_item, self.driver_name), abh3Com)
            cmd_A = 'TBL 0 ' + pre_cmd + ' A ' + set_val
            cmd_B = 'TBL 0 ' + pre_cmd + ' B ' + set_val
#            print cmd_A
            res_A = service(cmd_A)
            res_B = service(cmd_B)
            print 'Param Cmd ' + pre_cmd + ' (A B)| ' + res_A.response + " " + res_B.response
        except rospy.ServiceException, e:
            print('Service call failed: %s' % e)
    def dump(self,pre_cmd):
        try:
            service = rospy.ServiceProxy('/%s/%s/srv' % (self.driver_item, self.driver_name), abh3Com)
            cmd_A = 'TBL 0 ' + pre_cmd + ' A'
            cmd_B = 'TBL 0 ' + pre_cmd + ' B'
            res_A = service(cmd_A)
            res_B = service(cmd_B)
            print 'Param Cmd ' + pre_cmd + ' (A B)| ' + res_A.response + " " + res_B.response
        except rospy.ServiceException, e:
            print('Service call failed: %s' % e)
    def service(self, cmd):
        try:
            service = rospy.ServiceProxy('/%s/%s/srv' % (self.driver_item, self.driver_name), abh3Com)
            print 'Sended cmd ' + cmd
            res = service(cmd)
        except rospy.ServiceException, e:
            print('Service call failed: %s' % e)

    def shutdown(self):
        pass

if __name__ == '__main__':
    try:
        manager = abh3StatusDumper()
    except rospy.ROSInterruptException:
            pass
