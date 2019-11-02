# Exp-Dataset

录制数据集的代码  


## ros话题的消息格式和编码类型

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

## 录数据流程

```shell
# 0. 上电开机, 测试遥控控制是否正常, 确保保存数据的文件夹都存在
# lsusb

# 1. 启动接收IMU和编码器, 发布odom话题, 不广播tf(odom坐标系到base_footprint系)
# TF: 
# (1) map -> odom -> mynteye_link_frame -> ...
# (2) odom_combined -> base_footprint -> base_link
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