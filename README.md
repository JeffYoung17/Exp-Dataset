# Exp-Dataset

录制数据集的代码  

## 1. 小觅相机深度版使用

### sdk编译
```shell
# 1. ROS Wrapper 安装
make init
make ros
# 2. 修改mynteye_wrapper_nodelet.cc, 使得深度图topic发布的为滤波后的数据
# 3. 设置launch文件, 配置相机参数
```

### ros话题的消息格式和编码类型
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

## 2. 录数据流程
```shell
# 0. 上电开机, 测试遥控控制是否正常, 确保保存数据的文件夹已经创建
# lsusb

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

# 5. 启动record节点记录数据
roslaunch record record.launch
```

## 结果比较
```shell

```