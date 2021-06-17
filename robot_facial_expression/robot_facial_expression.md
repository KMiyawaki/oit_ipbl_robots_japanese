# Robot facial expression

[README](../README.md)

---

## コマンドによるロボットの表情変更（再）

シミュレータを起動する。

```shell
$ roslaunch oit_stage_ros navigation.launch
```

- `~catkin_ws/src/oit_stage_ros/scripts/face_image_publisher.py`が同時に実行されて、RViz上に表情が表示される。

さらに別ターミナルで`rostopic pub /robot_face_type std_msgs/String "data: 'happy'" -1`と実行すると表情の画像が変わる。  
`'happy'`を`'sad'`や`'normal'`に変えて実行することもできる。

### 考えてみよう

```shell
rostopic pub /robot_face_type std_msgs/String "data: 'happy'" -1
```

このコマンドは`robot_face_type`というトピックに`happy`という文字列を送信している。  
送信する文字列を`'sad'`や`'normal'`にすれば表情が変わる。  
文字列の送信プログラムは[ROS basics](../basics/basics_01.md)の`talker.py`で扱った。  

- **ここまでの知識で表情を変えるプログラムが作成できる。**

テキストの続きを読む前に挑戦しよう。

## 演習

`oit_pbl_ros_samples`パッケージにプログラムを作成する。

```shell
$ roscd oit_pbl_ros_samples/scripts
$ pwd
/home/[user name]/catkin_ws/src/oit_pbl_ros_samples/scripts
$ touch face.py
$ chmod u+x face.py
$ cd ..
$ code .
```

下記プログラムを入力する。コピー＆ペーストでも構わない。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from std_msgs.msg import String


class Face(object):
    def __init__(self):
        self.pub_face = rospy.Publisher(
            '/robot_face_type', String, queue_size=10)

    def process(self):
        types = ["happy", "sad", "normal"]
        for i in range(0, 9):
            self.pub_face.publish(types[i % 3])
            rospy.sleep(3)


def main():
    script_name = os.path.basename(__file__)
    rospy.init_node(os.path.splitext(script_name)[0])
    rospy.sleep(0.5)  # rospy.Time.now() returns 0, without this sleep.

    node = Face()
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

しばらくしてから`face.py`を実行。

- 起動したら、`RViz`の画面をよく観察すること。
- ３秒ごとに表情が変わるはずである。

```shell
$ rosrun oit_pbl_ros_samples face.py
[INFO] [1623925455.117405, 2082.000000]: /face:Started
[INFO] [1623925482.101090, 2109.000000]: /face:Exiting
```

## 問題 (1)

- `navigation.py`に表情の変化を加えてみよう。
  - ウェイポイントについたら`happy`の表情になり、５秒間待機後`normal`の表情に戻して次の目的地に向かうなど。

## 応用問題

- ロボットの表情を好きな画像に変えてみよう。

```shell
$ roscd oit_stage_ros/images/faces
$ ls
happy.png  normal.png  sad.png # それぞれの表情の画像
```

## アドバンスド課題

- プログラムを修正して表情のバリエーションを増やしてみよう。

```shell
$ roscd oit_stage_ros/scripts
$ ls 
face_image_publisher.py # 表情画像をpublishしているノードのプログラム
```

---

[README](../README.md)
