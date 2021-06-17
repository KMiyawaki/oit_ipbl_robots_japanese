# Robot control (1)

[README](../README.md)

---

## ロボットの速度制御

`/cmd_vel`トピックにデータを送信しロボットを前進させる。

## 演習

`oit_pbl_ros_samples`パッケージにプログラムを作成する。

```shell
$ roscd oit_pbl_ros_samples/scripts
$ pwd
/home/[user name]/catkin_ws/src/oit_pbl_ros_samples/scripts
$ touch robot_control.py
$ chmod u+x robot_control.py
$ cd ..
$ code .
```

下記プログラムを入力する。コピー＆ペーストでも構わない。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import os
import rospy
from geometry_msgs.msg import Twist


class RobotControlNode(object):
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def process(self):
        rate = rospy.Rate(10)  # Keep loop with 10hz
        cmd = Twist()
        cmd.linear.x = 0.4  # linear velocity
        start = rospy.Time.now()
        while rospy.Time.now().to_sec() - start.to_sec() < 3:  # 3 seconds
            self.pub_cmd_vel.publish(cmd)
            rate.sleep()  # Keep loop with 10hz
        cmd.linear.x = 0.0
        cmd.angular.z = math.radians(30)  # angular velocity
        start = rospy.Time.now()
        while rospy.Time.now().to_sec() - start.to_sec() < 3:  # 3 seconds
            self.pub_cmd_vel.publish(cmd)
            rate.sleep()  # Keep loop with 10hz


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = RobotControlNode()
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

しばらくしてから`robot_control.py`を実行。

- 起動したら、`Stage simulator`の画面と`rviz`の画面をよく観察すること。
- 秒速 0.4m で 3 秒間直進し、秒速 30度 で 3 秒間旋回するはずである。

```shell
$ rosrun oit_pbl_ros_samples robot_control.py
[INFO] [1623920868.296397, 8.500000]: /robot_control:Started
[INFO] [1623920874.284430, 14.500000]: /robot_control:Exiting
```

## 問題 (1)

- `robot_control.py`を修正し、直進->その場で１回転-> 直進 するプログラムを作成しなさい。

## 問題 (2)

- ロボットを四角形を描くように移動させてみよう。直進->90 度回転->直進・・・。
- 時計回りに動いてから反時計回りに動くなど。

## 応用問題

- ロボットに直進速度と旋回速度の両方を与えてどのような動きをするか観察してみよう。

---

[README](../README.md)
