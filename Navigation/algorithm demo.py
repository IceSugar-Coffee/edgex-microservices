#!/usr/bin/env python
# -*-coding:UTF-8 -*-

# 模块引入
# 引入需要的一些模块，如rospy，发布速度信息的Twist，激光雷达数据的Laserscan，一些数学公式math。
import rospy
from geometry_msgs.msg import Twist
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import LaserScan
import math


# 雷达结点
# 定义一个函数capturer用来接收并返回雷达的相关数据。
# wait_for_message是一个对于稳定输出源很好用的接收函数，它不需要回调函数，是一个直接赋值的功能。
# capture lidar data
def capturer():
    ldata = rospy.wait_for_message("/scan", LaserScan, timeout=None)  # catch lidar datas
    return ldata


# 引力斥力函数（速度发布器）
# 根据对x，y（理论上应该是distance和θ）判定来给机器人赋予不同的值，来调制小车的位姿以及姿态。
# 都是大量的if-else语句，逐个分析即可，趋势与前面原理所述一致。
# publish the cmd to the car
def cmd_pub(data):
    p_gain_vel = 1.15
    p_gain_ang = 2.5
    vel_msg = Twist()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    x_pillar, y_pillar = position_cal(data)
    print(x_pillar, y_pillar)
    # repel
    if 0.325 > x_pillar > 0:
        vel_msg.linear.x = 0.01 / x_pillar
        vel_msg.angular.z = (y_pillar * p_gain_ang)
    # attraction
    elif -0.30 > x_pillar > -0.35:
        if -0.30 >= x_pillar > -0.31:
            vel_msg.linear.x = -(x_pillar * p_gain_vel * 0.1)
            vel_msg.angular.z = -(y_pillar * p_gain_ang)
        elif -0.31 >= x_pillar > -0.32:
            vel_msg.linear.x = -(x_pillar * p_gain_vel * 0.2)
            vel_msg.angular.z = -(y_pillar * p_gain_ang)
        elif -0.32 >= x_pillar > -0.33:
            vel_msg.linear.x = -(x_pillar * p_gain_vel * 0.3)
            vel_msg.angular.z = -(y_pillar * p_gain_ang)
        elif -0.33 >= x_pillar > -0.34:
            vel_msg.linear.x = -(x_pillar * p_gain_vel * 0.4)
            vel_msg.angular.z = -(y_pillar * p_gain_ang)
        elif -0.34 >= x_pillar > -0.35:
            vel_msg.linear.x = -(x_pillar * p_gain_vel * 0.5)
            vel_msg.angular.z = -(y_pillar * p_gain_ang)
    # backward_repel
    elif 0 > x_pillar > -0.2:
        vel_msg.linear.x = 0.01 / x_pillar
        vel_msg.angular.z = (y_pillar * p_gain_ang)
    else:
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
    print(vel_msg.linear.x, vel_msg.angular.z)

    pub.publish(vel_msg)


# 位置计算
# 根据得到的激光雷达数据，实时的计算此时距离机器人的最近点，推算出它的距离，
# 进而推算出x和y坐标。（可以观察x,y的取值，它是以目标点为原点建立的tf）这里要用到激光雷达数据的相关知识。
# calculate the position of the car & item
def position_cal(data):
    smallest_distance = 200
    arr_size = int(math.floor((data.angle_max - data.angle_min) / data.angle_increment))
    for i in range(arr_size):
        if data.ranges[i] < smallest_distance:
            smallest_distance = data.ranges[i]
            alpha_pillar = (data.angle_min + (i * data.angle_increment))
    x = smallest_distance * math.cos(alpha_pillar)
    y = smallest_distance * math.sin(alpha_pillar)
    return x, y


# 在主函数里创建结点，定义发布频率，调用前面的函数
def main():
    rospy.init_node('follower')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        lidar_data = capturer()
        cmd_pub(lidar_data)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print('Error or Interuption happen')
        pass
