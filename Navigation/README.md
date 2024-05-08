# 多机跟随编队

## 实现功能
总体实现：第一辆车使用**巡线算法**实现自主运行，后续小车使用**引力-斥力**算法实现自动跟随前进

附加算法：卡尔曼滤波算法消除激光雷达测距的抖动

期望实现：使用基于坐标的跟随算法，实现特定队形跟随，如：直线型、三角形、一字型


## 配置项

### 需要变更主机的设置
1. 修改MQTT Broker的地址: WSL的EdgeX部署路径下 `docker-compose.yml` 修改 `host_ip`
```yaml
mqtt-broker:
    command:
    - /usr/sbin/mosquitto
    - -c
    - /mosquitto-no-auth.conf
    container_name: edgex-mqtt-broker
    hostname: edgex-mqtt-broker
    image: eclipse-mosquitto:2.0.15
    networks:
      edgex-network: null
    ports:
    - mode: ingress
      host_ip: 172.19.137.237
      target: 1883
      published: "1883"
      protocol: tcp
    read_only: true
    restart: always
    security_opt:
    - no-new-privileges:true
    user: 2002:2001
```

2. 修改规则引擎的MQTT Broker服务器地址: `edgex-kuiper` 容器内修改 `kuiper/etc/mqtt_source.yaml`
```yaml
#Global MQTT configurations
default:
  qos: 1
  server: "tcp://172.19.137.237:1883"
  #username: user1
  #password: password
  #certificationPath: /var/kuiper/xyz-certificate.pem
  #privateKeyPath: /var/kuiper/xyz-private.pem.key
  #rootCaPath: /var/kuiper/xyz-rootca.pem
  #insecureSkipVerify: false
  #connectionSelector: mqtt.mqtt_conf1
  #kubeedgeVersion: 
  #kubeedgeModelFile: ""

demo_conf: #Conf_key
  qos: 0
  server: "tcp://10.211.55.6:1883"
```

### 主机IP
- 费楼711: 172.19.137.237
- 20舍510: 172.31.95.73

### 端口号与对应服务
- 4000: EdgeX UI
- 1883: MQTT Broker
- 8000: 3D
- 8001: imageMark
- 8002: navigation

### MQTT相关话题
**注意**：采用多级话题

#### 订阅指令话题
- command/device0/#
- command/device1/#
- command/device2/#

#### 发布返回话题
- command/response/

#### 发布数据话题
- data/device0/
- data/device1/
- data/device2/


## 数据流转说明（以device1为例）
编写 `device1.py`，完成相关自动跟随算法

小车运行 `device1.py`，订阅名为 `command/device1/#` 的话题；

发布名为 `data/device1/` 和 `command/response/` 的MQTT话题。

EdgeX平台导入预定义的 `Device Profile` 和 `Device Config`， 添加device1和device2。

编辑规则引擎，创建名为 `laserScanStream` 的流，

```sql
CREATE STREAM laserScanStream () WITH (
  datasource = "data/device1",
  format = "JSON",
  type = "mqtt"
)
```

在EdgeX中创建规则Rule SQL
```sql
SELECT * FROM laserScanStream
```

添加Sink到 `hostIP:8002`，至此规则引擎部分完成。

编辑 `navigation.py`，新建HTTP Server，监听 `hostIP:8002`，处理规则引擎传来的POST请求，

解析JSON得到原始激光雷达数据，

对原始数据进行滤波，根据激光雷达数据计算与前车的距离，

计算出决策结果（线速度与角速度）后发送至 `command/device1/#` 话题，

（关于此处是否直接发送至MQTT Broker，更标准的做法是新建**Navigation的虚拟设备**，让规则引擎转发至Broker）

小车运行的 `device1.py` 接收到话题后发布 `Twist()`，实现对小车的运动控制，

至此形成闭环控制。