# Sensor data (1)

[README](../README.md)

---

## 仮想センサのデータを使う

シミュレータで利用可能なセンサは以下の通りである。

### Laser Range Finder

Laser Range Finder（LRF）はロボットから周囲の障害物までの距離を計測するセンサである。  
演習では下記センサをシミュレートしている。

[YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)

- Scanning angle: 0~360 degree
- Angle resolution: 0.5 degree (720 points per 1 scan)

LRFはレーザを放射状に発信し、距離を計測する。計測された結果はメートル単位で実数の配列として得られる。

### カメラ

ロボット正面の画像データが得られる。

### Odometry

Odometry は車輪の回転などでロボットの位置を計測するセンサである。  
おおよその位置を取得できる。ただし、シミュレータ内のロボットをドラッグした場合は計測結果は信頼できなくなる。

## 演習

`oit_pbl_ros_samples`パッケージにプログラムを作成する。

```shell
$ roscd oit_pbl_ros_samples/scripts
$ pwd
/home/[user name]/catkin_ws/src/oit_pbl_ros_samples/scripts
$ touch sensors.py
$ chmod u+x sensors.py
$ cd ..
$ code .
```

下記プログラムを入力する。コピー＆ペーストでも構わない。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import tf
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry


class SensorMessageGetter(object):
    def __init__(self, topic, msg_type, msg_wait=1):
        self.msg_wait = msg_wait
        self.topic = topic
        self.msg_type = msg_type

    def get_msg(self):
        message = None
        try:
            message = rospy.wait_for_message(
                self.topic, self.msg_type, self.msg_wait)
        except rospy.exceptions.ROSException as e:
            rospy.logdebug(e)
        return message


class Sensors(object):
    def __init__(self):
        self.laser = SensorMessageGetter("/base_scan", LaserScan)
        self.odom = SensorMessageGetter("/odom", Odometry)
        self.img = SensorMessageGetter("/image", Image)

    def get_yaw(self, odom):
        q = odom.pose.pose.orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        return yaw

    def process_laser(self, msg):
        if msg:
            rospy.loginfo("Recv sensor data. type = %s", type(msg))
            # check reference
            # http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
            # len(msg.ranges) is range senseor's data count.
            rospy.loginfo("len(msg.ranges) = %d", len(msg.ranges))
            # msg.ranges[i] is distance mesurement of index i.
            rospy.loginfo("msg.ranges[0] = %f", msg.ranges[0])

    def process_odom(self, msg):
        if msg:
            rospy.loginfo("Recv sensor data. type = %s", type(msg))
            # check reference
            # http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
            # msg.pose.pose.position is robot's position
            rospy.loginfo("msg.pose.pose.position.x = %f, msg.pose.pose.position.y = %f",
                          msg.pose.pose.position.x, msg.pose.pose.position.y)
            # self.get_yaw returns yaw angle from odometry data
            rospy.loginfo("yaw = %f", self.get_yaw(msg))

    def process_img(self, msg):
        if msg:
            rospy.loginfo("Recv sensor data. type = %s", type(msg))
            # check reference
            # http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
            rospy.loginfo("msg.width = %d, msg.height = %d",
                          msg.width, msg.height)

    def process(self):
        rate=rospy.Rate(20)
        tm = rospy.Time.now()
        while (rospy.Time.now().to_sec() - tm.to_sec()) < 10:
            self.process_laser(self.laser.get_msg())
            self.process_odom(self.odom.get_msg())
            self.process_img(self.img.get_msg())
            rate.sleep()


def main():
    script_name=os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node=Sensors()
    rospy.loginfo("%s:Started", rospy.get_name())

    node.process()
    rospy.loginfo("%s:Exiting", rospy.get_name())


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("%s:%s", rospy.get_name(), str(e))
        exit(1)

```

### 実行順序

まず、シミュレータを起動する。

```shell
$ roslaunch oit_stage_ros navigation.launch
```

しばらくしてから`sensors.py`を実行。

- 受信したセンサデータの情報が出力される。

```shell
$ rosrun oit_pbl_ros_samples sensors.py
[INFO] [1624081858.601793, 16.200000]: /sensors:Started
[INFO] [1624081858.706859, 16.300000]: Recv sensor data. type = <class 'sensor_msgs.msg._LaserScan.LaserScan'>
[INFO] [1624081858.710373, 16.300000]: len(msg.ranges) = 720
[INFO] [1624081858.714078, 16.300000]: msg.ranges[0] = 1.412500
[INFO] [1624081858.809320, 16.400000]: Recv sensor data. type = <class 'nav_msgs.msg._Odometry.Odometry'>
[INFO] [1624081858.813018, 16.400000]: msg.pose.pose.position.x = 0.000000, msg.pose.pose.position.y = 0.000000
[INFO] [1624081858.816890, 16.400000]: yaw = 0.000000
[INFO] [1624081858.907231, 16.500000]: Recv sensor data. type = <class 'sensor_msgs.msg._Image.Image'>
[INFO] [1624081858.910390, 16.500000]: msg.width = 700, msg.height = 540
[INFO] [1624081859.008587, 16.600000]: Recv sensor data. type = <class 'sensor_msgs.msg._LaserScan.LaserScan'>
[INFO] [1624081859.011822, 16.600000]: len(msg.ranges) = 720
[INFO] [1624081859.014337, 16.600000]: msg.ranges[0] = 1.412500
```

## 問題(1)

- `sensors.py`を実行し、別ターミナルでコマンド入力によりロボットを直進させてみる。
  - [Robot control (1)](../robot_control/robot_control_01.md)の「コマンドでロボットを動かす」を参照。
- 実行結果を観察し、どの値が変化しているか確認しなさい。
- 直進だけではなく、回転もさせてみなさい。

```shell
$ rosrun oit_pbl_ros_samples sensors.py
[INFO] [1624082234.701371, 392.300000]: /sensors:Started
[INFO] [1624082235.716576, 393.300000]: Recv sensor data. type = <class 'sensor_msgs.msg._LaserScan.LaserScan'>
[INFO] [1624082235.720416, 393.300000]: len(msg.ranges) = 720
[INFO] [1624082235.724363, 393.300000]: msg.ranges[0] = 1.432500
[INFO] [1624082235.810526, 393.400000]: Recv sensor data. type = <class 'nav_msgs.msg._Odometry.Odometry'>
[INFO] [1624082235.813862, 393.400000]: msg.pose.pose.position.x = 0.020000, msg.pose.pose.position.y = 0.000000
[INFO] [1624082235.816473, 393.400000]: yaw = 0.000000
[INFO] [1624082235.911198, 393.500000]: Recv sensor data. type = <class 'sensor_msgs.msg._Image.Image'>
[INFO] [1624082235.914241, 393.500000]: msg.width = 700, msg.height = 540
[INFO] [1624082236.010182, 393.600000]: Recv sensor data. type = <class 'sensor_msgs.msg._LaserScan.LaserScan'>
[INFO] [1624082236.013734, 393.600000]: len(msg.ranges) = 720
[INFO] [1624082236.016757, 393.600000]: msg.ranges[0] = 1.462500
[INFO] [1624082236.102175, 393.700000]: Recv sensor data. type = <class 'nav_msgs.msg._Odometry.Odometry'>
[INFO] [1624082236.104988, 393.700000]: msg.pose.pose.position.x = 0.050000, msg.pose.pose.position.y = 0.000000
[INFO] [1624082236.108750, 393.700000]: yaw = 0.000000
[INFO] [1624082236.205693, 393.800000]: Recv sensor data. type = <class 'sensor_msgs.msg._Image.Image'>
[INFO] [1624082236.209284, 393.800000]: msg.width = 700, msg.height = 540
[INFO] [1624082236.303588, 393.900000]: Recv sensor data. type = <class 'sensor_msgs.msg._LaserScan.LaserScan'>
[INFO] [1624082236.306639, 393.900000]: len(msg.ranges) = 720
[INFO] [1624082236.309889, 393.900000]: msg.ranges[0] = 1.492500
[INFO] [1624082236.409391, 394.000000]: Recv sensor data. type = <class 'nav_msgs.msg._Odometry.Odometry'>
[INFO] [1624082236.412709, 394.000000]: msg.pose.pose.position.x = 0.080000, msg.pose.pose.position.y = 0.000000
[INFO] [1624082236.417931, 394.000000]: yaw = 0.000000
[INFO] [1624082236.506177, 394.100000]: Recv sensor data. type = <class 'sensor_msgs.msg._Image.Image'>
[INFO] [1624082236.510218, 394.100000]: msg.width = 700, msg.height = 540
```

### 問題 (2)

- LRFのデータからロボットの右の障害物までの距離を取得しなさい。
- 同様に左、正面方向の距離を取得しなさい。
- なお、`rospy.loginfo("msg.ranges[0] = %f", msg.ranges[0])`はロボットの真後ろの障害物までの距離を表示している。
- 取得できたかどうかは、`stage_ros`内の障害物をロボットに近づけたり遠ざけたりして確認しなさい。

---

[README](../README.md)
