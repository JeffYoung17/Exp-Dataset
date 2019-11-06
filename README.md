# Exp-Dataset

录制数据集的代码  

## 1. 录数据流程

### 1.1 检查相机输出是否正常
```shell
# 1. 编译, launch文件和深度图滤波已经在代码中修改
cd xx/
git clone https://github.com/JeffYoung17/MYNT-EYE-D-SDK.git ./
make init
make ros
source xx/MYNT-EYE-D-SDK/wrappers/ros/devel/setup.sh or setup.zsh
# 2. 查看图像是否正常
roslaunch mynteye_wrapper_d display.launch 
```

### 1.2 检查机器人控制是否正常

```shell
# 1. 上电开机, 直接遥控器控制 /mecanum_base
lsusb
ls -l /dev|grep ttyUSB
# 2. 编译
cd xx/
git clone https://github.com/JeffYoung17/Exp-Dataset.git ./
cd Exp-Dataset/wrappers/ros/
catkin_make -j8
source xx/Exp-Dataset/wrappers/ros/devel/setup.sh or setup.zsh
# 3. 启动机器人ros驱动
roslaunch robot_odom core.launch
rqt_graph / rostopic list / rostopic hz xxx # 检查话题是否正确发布
```

### 1.3 录制数据
```shell
# 1. 启动接收IMU和编码器, 发布odom坐标系到base_footprint坐标系的odom话题,但是不广播二者间的TF.
# TF: 
# (1) map -> odom -> mynteye_link_frame -> ...
# (2) odom -> base_footprint
roslaunch robot_odom core.launch

# 2. 启动相机, 发布图像话题
roslaunch mynteye_wrapper_d mynteye.launch

# 3. 启动rtabmap_ros, 得到视觉里程计topic(坐标系和自车坐标系朝向一致)
# 如果图像时间戳硬同步 approx_sync:=false
# 如果图像时间戳不同步 approx_sync:=true
roslaunch rtabmap_ros rtabmap.launch \
  rtabmap_args:="--delete_db_on_start" \
  depth_topic:=/mynteye/depth/image_raw \
  rgb_topic:=/mynteye/left/image_color \
  camera_info_topic:=/mynteye/left/camera_info approx_sync:=true \
  frame_id:=mynteye_link_frame

# 4. 启动ekf定位
roslaunch robot_odom ekf.launch

# 5. 启动record节点记录数据 /test下创建rgb和depth两个文件夹
rosrun record record_node /home/fqy/data/test/
```

## Appendix: 小觅相机的ros话题的消息格式和编码类型

```shell
topic: /mynteye/left/image_color
	type: sensor_msgs/Image
	encoding: "bgr8"
topic: /mynteye/left/image_mono
	type: sensor_msgs/Image
	encoding: "mono8"
topic: /mynteye/depth/image_raw
	type: sensor_msgs/Image
	encoding: "mono16"
```

```shell
/camera_mesh
/camera_mesh_array
/clicked_point
/initialpose
/move_base_simple/goal
/mynteye/depth/camera_info
/mynteye/depth/image_raw
/mynteye/depth/image_raw/compressed
/mynteye/depth/image_raw/compressed/parameter_descriptions
/mynteye/depth/image_raw/compressed/parameter_updates
/mynteye/depth/image_raw/compressedDepth
/mynteye/depth/image_raw/compressedDepth/parameter_descriptions
/mynteye/depth/image_raw/compressedDepth/parameter_updates
/mynteye/depth/image_raw/theora
/mynteye/depth/image_raw/theora/parameter_descriptions
/mynteye/depth/image_raw/theora/parameter_updates
/mynteye/imu/data_raw
/mynteye/imu/data_raw_processed
/mynteye/left/camera_info
/mynteye/left/image_color
/mynteye/left/image_color/compressed
/mynteye/left/image_color/compressed/parameter_descriptions
/mynteye/left/image_color/compressed/parameter_updates
/mynteye/left/image_color/theora
/mynteye/left/image_color/theora/parameter_descriptions
/mynteye/left/image_color/theora/parameter_updates
/mynteye/left/image_mono
/mynteye/left/image_mono/compressed
/mynteye/left/image_mono/compressed/parameter_descriptions
/mynteye/left/image_mono/compressed/parameter_updates
/mynteye/left/image_mono/theora
/mynteye/left/image_mono/theora/parameter_descriptions
/mynteye/left/image_mono/theora/parameter_updates
/mynteye/points/data_raw
/mynteye/right/camera_info
/mynteye/right/image_color
/mynteye/right/image_color/compressed
/mynteye/right/image_color/compressed/parameter_descriptions
/mynteye/right/image_color/compressed/parameter_updates
/mynteye/right/image_color/theora
/mynteye/right/image_color/theora/parameter_descriptions
/mynteye/right/image_color/theora/parameter_updates
/mynteye/right/image_mono
/mynteye/right/image_mono/compressed
/mynteye/right/image_mono/compressed/parameter_descriptions
/mynteye/right/image_mono/compressed/parameter_updates
/mynteye/right/image_mono/theora
/mynteye/right/image_mono/theora/parameter_descriptions
/mynteye/right/image_mono/theora/parameter_updates
/mynteye/temp/data_raw
```