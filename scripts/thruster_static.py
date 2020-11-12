#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pigpio
import time
import rospy
from std_msgs.msg import Int64

class Thruster():
    zero_point_duty = 750000

    def __init__(self):
        self.cmd_duty = 0.0 #Command duty
        self.cur_duty = 0.0 #Current duty
        self.dif_duty = 0.0 #Difference of duty
        self.flag = 0.0

        self.gpio_pin = 13
        self.duration = 0.1

        self.pi = pigpio.pi()
        self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)

    def init_thrusetr(self):
        self.pi.hardware_PWM(self.gpio_pin, 500, self.zero_point_duty)
        self.cur_duty = self.zero_point_duty
        time.sleep(self.duration)
        #rosparam.set_param("/thruster/flag_vel", "2")

    def main_loop(self):
        self.init_thrusetr()
        while not rospy.is_shutdown():
            self.flag = rospy.get_param('/thruster/flag_vel')
            self.thruster_velocity()
        rospy.on_shutdown(self.safety_shutdown())

    def thruster_velocity(self):
        if self.flag == 1:
            self.cmd_duty = 750000
            self.rampup()
            print("flag 1, Thrust 0kfg, duty=%d" % self.cmd_duty)
        elif self.flag == 2:
            self.cmd_duty = 826000
            self.rampup()
            print("flag 2, Thrust 1.0kfg, duty=%d" % self.cmd_duty)
        elif self.flag == 3:
            self.cmd_duty = 868000
            self.rampup()
            print("flag 3, Thrust 2.0kfg, duty=%d" % self.cmd_duty)
        elif self.flag == 4:
            self.cmd_duty = 900000
            self.rampup()
            print("flag 4, Thrust 3.0kfg, duty=%d" % self.cmd_duty)
        elif self.flag == 5:
            self.cmd_duty = 931000
            self.rampup()
            print("flag 5, Thrust 4.0kfg, duty=%d" % self.cmd_duty)
        elif self.flag == 6:
            self.cmd_duty = 786000
            self.rampup()
            print("flag 6, Thrust 0.25kfg, duty=%d" % self.cmd_duty)
        elif self.flag == 7:
            self.cmd_duty = 800000
            self.rampup()
            print("flag 7, Thrust 0.5kfg, duty=%d" % self.cmd_duty)
        elif self.flag == 8:
            self.cmd_duty = 812500
            self.rampup()
            print("flag 8, Thrust 0.75kfg, duty=%d" % self.cmd_duty)
        self.pi.hardware_PWM(self.gpio_pin, 500, self.cmd_duty)
        self.cur_duty = self.cmd_duty

    def rampup(self):
        step = 25
        self.dif_duty = self.cmd_duty - self.cur_duty#Discriminant
        print ("self.dif_duty = %d" % self.dif_duty)
        if self.dif_duty > 0: #rampup
            print("Rampup started")
            for i in range(0, self.dif_duty+step, step):
                self.cmd_duty = self.cur_duty + i
                self.pi.hardware_PWM(self.gpio_pin, 500, self.cmd_duty)
                print("duty = %d" % self.cmd_duty)
            print("Rampup finished")
        elif self.dif_duty < 0: #Down
            print("Down started")
            for j in range(0, abs(self.dif_duty)+step, step):
                self.cmd_duty = self.cur_duty - j
                self.pi.hardware_PWM(self.gpio_pin, 500, self.cmd_duty)
                print("duty = %d" % self.cmd_duty)
            print("Down finished")

        elif self.dif_duty == 0:
            print("No need rampup  and down")

    def safety_shutdown(self):
        self.cmd_duty = self.zero_point_duty
        self.rampup()
        self.pi.hardware_PWM(self.gpio_pin, 500, self.cmd_duty)
        print "Done safety shutdown!"

    def emergency_shutdown(self):
        self.cmd_duty = self.zero_point_duty
        self.pi.hardware_PWM(self.gpio_pin, 500, self.cmd_duty)
        print "Done emergency shutdown!"

if __name__ == '__main__':
    try:
        rospy.init_node('thruster_static', anonymous=True)
        usm = Thruster()
        usm.main_loop()

    except rospy.ROSInterruptException:
        usm.safety_shutdown()
