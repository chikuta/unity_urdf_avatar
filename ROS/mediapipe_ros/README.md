# mediapipe_ros

## 実行環境

* Ubuntu 20.04
* ROS1 noetic
* python 3
* Intel Realsense D435

## ビルド方法

以下のように適宜読み替えてください。

* ${YOUR_ROS_WS_HOME} ... ROS WS の位置
* ${THIS_UNITY_PROJECT_DIR} ... unity_urdf_avatar を clone したディレクトリ


```bash
source /opt/ros/noetic/setup.bash
cd ${YOUR_ROS_WS_HOME}
mkdir -p ws/src && cd ws/src
cp -r ${THIS_UNITY_PROJECT_DIR}/unity_urdf_avatar/ROS/mediapipe_ros .
cp mediapipe_ros/.rosinstall .
wstool update
cd ../
rosdep install  --from-paths src
catkin build
```

## How to launch

以下のコマンドを実行して必要な ros node を起動します。

```bash
source devel/setup.bash
roslaunch mediapipe_ros pose_estimation.launch
```
