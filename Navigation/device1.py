import random
import time
import json
import math
import rospy
import socket

from paho.mqtt import client as mqtt_client
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import LaserScan

broker = '172.19.137.237'  # broker地址，为wsl
port = 1883  # mqtt默认端口
subTopic = "command1"  # 订阅mqtt指令话题，可使用/分隔
pubTopic = "ResponseTopic"  # 发布mqtt回复话题
dataTopic = "data/device1/camera/"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'

isRunning = False
velocity = 0.0
angular = 0.0
position = "(0,0)"
# cameraUrl = "http://192.168.1.2:8080/shot.jpg"
laserScan = {}
rawImage = {}

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
hostIp = s.getsockname()[0]

def setVelocity(velocity):
    # TODO: 设置小车速度为 velocity (m/s)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = velocity
    twist.angular.z = 0
    print(f'Setting Velocity to: {velocity} m/s...')
    # while not rospy.is_shutdown():
    pub.publish(twist)
    # rate.sleep()


def getVelocity():
    # TODO: 读出小车速度并返回
    rospy.Subscriber("/odom", Odometry, queue_size=1)
    odom = Odometry()
    vel = odom.twist.twist.linear.x
    print(f'Current Velocity is: {vel} m/s...')
    return vel


def setAngular(angular):
    # TODO: 设置小车角速度为 angular (rad/s)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    # rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = velocity
    twist.angular.z = angular
    print(f'Setting Angular to: {angular} rad/s...')
    # while not rospy.is_shutdown():
    pub.publish(twist)
    # rate.sleep()


def getAngular():
    # TODO: 读出小车角速度并返回
    rospy.Subscriber("/odom", Odometry, queue_size=1)
    odom = Odometry()
    ang = odom.twist.twist.angular.z
    print(f'Current Angular is: {ang} rad/s...')
    return ang


def getPosition():
    # TODO: 获得小车坐标，字符串拼接并返回
    print(f'Current Position is: {position}')
    return position


def getCameraUrl():
    global rawImage
    # TODO: 获取小车摄像头图像编码并返回
    imgdata = rospy.wait_for_message("/camera/rgb/image_raw", Image, timeout=None)  # catch image datas
    rawImage = {}
    rawImage['height'] = imgdata.height
    rawImage['width'] = imgdata.width
    rawImage['encoding'] = imgdata.encoding
    rawImage['data'] = imgdata.data
    result = str(rawImage)
    print(f'rawImage is: {result}')
    return result


def getLaserScan():
    global laserScan
    # TODO: 获取小车激光雷达扫描结果，字符串拼接并返回
    ldata = rospy.wait_for_message("/scan", LaserScan, timeout=None)  # catch lidar datas
    laserScan = {}
    laserScan['angle_min'] = ldata.angle_min
    laserScan['angle_max'] = ldata.angle_max
    laserScan['angle_increment'] = ldata.angle_increment
    laserScan['ranges'] = ldata.ranges
    result = str(laserScan)
    print(f'LaserScan is: {result}')
    return result


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        global isRunning, velocity, angular, cameraUrl, laserScan
        # print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        js = json.loads(msg.payload.decode())
        cmd = js['cmd']
        method = js['method']
        if method == 'set':
            if cmd == 'velocity':
                velocity = js['velocity']
                client.publish(pubTopic, json.dumps(js))
                setVelocity(velocity)
            elif cmd == 'angular':
                angular = js['angular']
                client.publish(pubTopic, json.dumps(js))
                setAngular(angular)
        elif method == 'get':
            if cmd == 'velocity':
                js['velocity'] = getVelocity()
                client.publish(pubTopic, json.dumps(js))
            elif cmd == 'angular':
                js['angular'] = getAngular()
                client.publish(pubTopic, json.dumps(js))
            elif cmd == 'position':
                js['position'] = getPosition()
                client.publish(pubTopic, json.dumps(js))
            elif cmd == 'cameraUrl':
                js['cameraUrl'] = getCameraUrl()
                client.publish(pubTopic, json.dumps(js))
            elif cmd == 'laserScan':
                js['laserScan'] = getLaserScan()
                client.publish(pubTopic, json.dumps(js))

    client.subscribe(subTopic)
    client.on_message = on_message


def run():
    rospy.init_node("ros_device-1")
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


# 雷达结点
# 定义一个函数capturer用来接收并返回雷达的相关数据。
# wait_for_message是一个对于稳定输出源很好用的接收函数，它不需要回调函数，是一个直接赋值的功能。
# capture lidar data
def capturer():
    ldata = rospy.wait_for_message("/scan", LaserScan, timeout=None)  # catch lidar data
    # TODO: 将ldata格式化后发布MQTT消息
    return ldata


def cmd_pub():
    vel_msg = Twist()
    # TODO: 接收前端发送的速度控制指令，经过规则引擎转发后的MQTT协议，直接发布twist话题。



if __name__ == '__main__':
    run()
    # try:
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         lidar_data = capturer()     # 小车端获取激光雷达相关数据
    #         cmd_pub()
    #         rate.sleep()
    # except rospy.ROSInterruptException:
    #     print('Error or Interruption happen')
    #     pass
