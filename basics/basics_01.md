# ROS(1)

[README](../README.md)

---

## ROS とは

- [ROS のサイト](http://www.ros.org/)

Robot Operating System の略。複数の実行プログラムが協調する分散システムの構築をしやすくするミドルウェア。

- OS と皆さんが作成するアプリケーションの間に位置するソフト。
- 実行プログラム間の通信を隠蔽してくれる仕組み、という理解でも可。

## 何故分散システムなのか

### ソフトウェアの面から

- ロボットの複雑なタスクを達成するにはたくさんの機能が必要。
  - 音声認識
  - 画像処理
  - アクチュエータ制御
  - 統合
- 全てを含んだ一つの実行プログラムを作成するとどうなるか
  - 一つの機能(関数)を変更した際にその影響が全体に及ぶ。
  - 一つの機能に問題があって、実行時エラーが発生した際に全ての機能が停止する。
  - 分担して開発しにくい。一つの機能だけをテストしにくい。

## ROS 用語

- ノード(`Node`)・・・一つの実行プログラム。
- トピック(`Topic`)・・・ノード間で送受信されるデータ。名前(トピック名)と型を持つ。
- パブリッシャ(`Publisher`)・・・トピックを発信するノード
- サブスクライバ(`Subscriber`)・・・トピックを受信(購読)するノード。
- ROS マスター・・・ノード同士を結び付けてくれるプログラム。
- ワークスペース・・・ノードを作成するための場所。通常はホームディレクトリに作成する。
- パッケージ・・・作成したノードをある程度のまとまりでグルーピングしたもの。

## 演習(1)

ROS マスターを起動しましょう。ターミナルで下記コマンドを実行。

```shell
$ roscore
... logging to /home/[user name]/.ros/log/9474a7ce-4941-11ea-a3d0-000c2924787d/roslaunch-ubuntu-7288.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:34303/
ros_comm version 1.14.3


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.3

NODES

auto-starting new master
process[master]: started with pid [7299]
ROS_MASTER_URI=http://ubuntu:11311/

setting /run_id to 9474a7ce-4941-11ea-a3d0-000c2924787d
process[rosout-1]: started with pid [7310]
started core service [/rosout]
```

出力されたメッセージを確認すること。

- `melodic`という文字が出ているはず。

### Melodic Morenia

ROS の LTS (Long Term Support) バージョンの一つ。

- 確認できたら、`Ctrl+C`で終了させておく。

## 簡単なパブリッシャとサブスクライバの作成(1)

- 参考：[ROS/Tutorials/WritingPublisherSubscriber(python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- ターミナルを開き、次のコマンドを実行

```shell
$ roscd oit_pbl_ros_samples/scripts
```

### 問題(1)

- カレントディレクトリを確認しなさい。

## roscd [パッケージ名]

`ROS`パッケージのディレクトリに移動できる。下記のコマンドでその他のパッケージを見に行こう。

```shell
$ roscd navigation
$ pwd
/opt/ros/melodic/share/navigation
$ roscd rviz
$ pwd
/opt/ros/melodic/share/rviz
$ roscd oit_pbl_ros_samples/scripts
```

## 簡単なパブリッシャとサブスクライバの作成(2)

再び`oit_pbl_ros_samples/scripts`に移動する。

```shell
$ roscd oit_pbl_ros_samples/scripts
```

`scripts`ディレクトリに下記二つのファイルをダウンロード

- [talker.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py)

```shell
$ wget https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
--2020-10-07 15:38:36--  https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
Resolving raw.githubusercontent.com (raw.githubusercontent.com)... 151.101.228.133
・・・
talker.py                                 100%[==================================================================================>]   2.17K  --.-KB/s    in 0s      

2020-10-07 15:38:37 (10.1 MB/s) - ‘talker.py’ saved [2217/2217]
```

- [listener.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py)

```shell
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
--2020-10-07 15:40:18--  https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
Resolving raw.github.com (raw.github.com)... 151.101.228.133
・・・
listener.py                               100%[==================================================================================>]   2.35K  --.-KB/s    in 0s      

2020-10-07 15:40:19 (7.51 MB/s) - ‘listener.py’ saved [2406/2406]
```

ダウンロードした二つのファイルの上から２行目に`# -*- coding: utf-8 -*-`と追記し、日本語が使えるようにしておいてください。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
```

### 問題(2)

- ダウンロードした 2 つのファイルにユーザの実行権限をつけなさい。
- 権限が付与されたことを`ls -l`で確認すること。

```shell
$ ls -l
合計 8
-rwxrw-r-- 1 robocup2020 robocup2020 2406  2月 12 12:43 listener.py
-rwxrw-r-- 1 robocup2020 robocup2020 2217  2月 12 12:43 talker.py
```

## talker.py の実行

```shell
$ rosrun oit_pbl_ros_samples talker.py
```

エラーが出て何も起きないはず。エラーが出ずに無事実行できた人は手順を飛ばしているか、勘の良い人。

- どんなメッセージか確認する。

```shell
$ rosrun oit_pbl_ros_samples talker.py
[ERROR] [1623917414.259254]: Unable to immediately register with master node [http://localhost:11311]: master may not be running yet. Will keep trying.
```

- `Ctrl+C`でプログラムを終了させる。

## rosrun [パッケージ名][ノード名]

あるパッケージに含まれるノードを実行する。

- ただし、ノードの実行には原則事前に ROS マスターを起動しておくことが必要。
- 別のターミナルを開き`roscore`を実行する。
- さらに別のターミナルを開き次のコマンドを実行。

```shell
$ rosrun oit_pbl_ros_samples talker.py
[INFO] [1581037099.621621]: hello world 1581037099.62
[INFO] [1581037099.722943]: hello world 1581037099.72
[INFO] [1581037099.822706]: hello world 1581037099.82
...
```

- さらに別のターミナルを開き次のコマンドを実行。

```shell
$ rosrun oit_pbl_ros_samples listener.py
[INFO] [1581037131.453663]: /listener_8862_1581037131191I heard hello world 1581037131.45
[INFO] [1581037131.555024]: /listener_8862_1581037131191I heard hello world 1581037131.55
[INFO] [1581037131.658074]: /listener_8862_1581037131191I heard hello world 1581037131.65
...
```

二つのノードを動かしたまま、次項のコマンドを実行すること。

## rqt_graph

- ROS のノード同士のつながりを可視化する。

```shell
$ rqt_graph
```

![rqt-min.png](./rqt-min.png)

## rostopic list

- 現在流れているトピックのリストを得る。

```shell
$ rostopic list
/chatter
/rosout
/rosout_agg
```

## rostopic echo [トピック名]

- [トピック名]のデータを表示する。
- 例:`rostopic echo /chatter`
  - トピック名は`tab`キー補完可能

```shell
$ rostopic echo /chatter
data: "hello world 1581037256.15"
---
data: "hello world 1581037256.25"
---
data: "hello world 1581037256.35"
...
```

## rostopic type [トピック名]

[トピック名]の型を表示する。

- 例:`rostopic type /chatter`

```shell
$ rostopic type /chatter
std_msgs/String
```

`/chatter`というトピック名で文字列型のデータが送信されていることが分かる。

## talker.py のポイント

適当なテキストエディタで`talker.py`を見てみる。

```python
def talker():
    # 'chatter'というトピック名にデータをパブリッシュする準備。
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 'talker'という名前でノードを生成する。
    # anonymous=True により、名前にランダムな番号が追加される。
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hzでループを回す。
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str) # 端末上に hello_str の内容を表示。
        pub.publish(hello_str) # hello_str をパブリッシュ。
        rate.sleep()
```

## listener.py のポイント

適当なテキストエディタで`listener.py`を見てみる。

```python
def callback(data):
    # 受信したデータを表示。
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    # 'chatter'というトピック名のデータを受信する準備。
    # 受信した瞬間に callback というメソッドが呼ばれるようにしている。
    rospy.Subscriber('chatter', String, callback)
    # 無限ループ開始
    rospy.spin()

if __name__ == '__main__':
    listener()
```

### 問題(3)

- ROS マスター、`talker.py`、`listener.py` を全て`Ctrl+C`で終了させなさい。

## 応用問題(1)

ROS の`std_msg`について調べなさい。

- `String`以外にどのような型が用意されているか、[std_msgs](http://wiki.ros.org/std_msgs)を参考に調べなさい。

---

[README](../README.md)
