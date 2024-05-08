import random
import time
import json
import math
import rospy

from paho.mqtt import client as mqtt_client
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import LaserScan


broker = '172.19.137.237'   # broker地址，为本机的docker
# broker = '172.31.95.73'   # broker地址，为本机的docker
port = 1883                 # mqtt默认端口
# topic = "CommandTopic"      # mqtt话题，可使用/分隔
topic = "data/device1/"      # mqtt话题，可使用/分隔
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
laserScan = {}


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

# 连接mqtt broker
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def publish(client):
    msg_count = 0
    while True:
        time.sleep(1)
        # msg = f"messages: {msg_count}"
        # msg = '{"message": "hello world"}'
        msg = '{"message": "' + getLaserScan() + '"}'
        result = client.publish(topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")
        msg_count += 1


def run():
    rospy.init_node("ros_device-1")
    client = connect_mqtt()
    client.loop_start()
    publish(client)


if __name__ == '__main__':
    run()
