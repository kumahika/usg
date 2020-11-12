#!/usr/bin/env python
# coding: utf-8
import pigpio
import time
import rospy
import numpy as np
import getpass
# import matplotlib.pyplot as plt
from std_msgs.msg import Int64
from std_msgs.msg import Float64

def getNearestValue(list, num):
    """
    概要: リストからある値に最も近い値を返却する関数
    @param list: データ配列
    @param num: 対象値
    @return 対象値に最も近い値
    """

    # リスト要素と対象値の差分を計算し最小値のインデックスを取得
    idx = np.abs(np.asarray(list) - num).argmin()
    return list[idx]

def callback(msg):
    global duty
    global step
    global pub
    global pi
    global gpio_pin
    control_flag = msg.data
    thrust_msg = Float64()
    if control_flag == 1 and duty < 950000: # 500 [Hz] 75% (pwm 1500)
        duty += step
        print("increment, duty=%d" % duty)
        thrust_msg.data =duty2thrust(duty)
        print thrust_msg.data
        pub.publish(thrust_msg)
    elif control_flag == 2 and duty > 550000:
        duty -= step
        print("decrement, duty=%d" % duty)
        pub.publish(duty2thrust(duty))
    elif control_flag == 0:
        print("OFF")
        duty = 750000
        pub.publish(duty2thrust(duty))
    pi.hardware_PWM(gpio_pin, 500, duty)

def duty2thrust(duty):
    pwm = 0.002 * duty
    thrust = pwm2thrust(pwm)
    print "pwm is ", pwm , "thrust is ", thrust, "kgf" 
    return thrust

def pwm2thrust(pwm):
    global data_pwm
    global data_thrust
    thrust = data_thrust[data_pwm.index(getNearestValue(data_pwm,pwm))]

    # plt.scatter(data_pwm, data_thrust, label='origin data')
    # res_thrust=np.polyfit(data_pwm, data_thrust, 3)
    # y_thrust=np.round(np.poly1d(res_thrust)(data_pwm), decimals=1)
    # plt.plot(data_pwm, y_thrust, label='3d')
    # plt.legend()
    # plt.show()
    return thrust

def read_csv():
    user = getpass.getuser()
    folder = "/home/" + user + "/catkin_ws/src/uuv_manipulator/scripts/" 
    name = "thrust"
    volt = "14v"
    filename = folder + name + volt + ".csv"
    rawdata = np.loadtxt(filename,delimiter = ',')
    return rawdata

def init():
    global duty
    global step
    global data
    global data_pwm
    global data_thrust
    global pi
    global gpio_pin
    gpio_pin = 13
    norm_pwm = 55
    step = 10000/2
    duty =  750000 + (0.4*norm_pwm-20)*10000
    print("duty%d" % duty)
    duration = 0.1
    data = read_csv()
    data_pwm = []
    data_thrust = []
    for i in range (0,201):
        data_pwm.append(data[i,0])
        data_thrust.append(round(data[i,1],1))
    pi = pigpio.pi()
    pi.set_mode(gpio_pin, pigpio.OUTPUT)
    pi.hardware_PWM(gpio_pin, 500, 750000)
    time.sleep(duration)

def main_loop():
    global pub
    rospy.init_node('thruster_dynamic', anonymous=True)
    rospy.Subscriber('thruster/pwm', Int64, callback)
    pub = rospy.Publisher('thrust', Float64, queue_size=10)
    rospy.spin()

def safety_shutdown(self):
    global pi
    global gpio_pin
    pi.hardware_PWM(gpio_pin, 500, 750000)
    print "Done safety shutdown!"

if __name__ == '__main__':
    try:
        init()
        main_loop()
    except rospy.ROSInterruptException:
        safety_shutdown()
