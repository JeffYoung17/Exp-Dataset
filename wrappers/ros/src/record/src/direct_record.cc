#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

using namespace std;

string save_dir;
vector<int> compression_params;

void image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    ROS_INFO("Rx img!");

    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat save_img = ptr->image;
    double t = ptr->header.stamp.toSec();

    string f_str = to_string(t);
    std::ofstream foutRGB(save_dir + "rgb.txt", std::ios::app);
    foutRGB.setf(std::ios::fixed, std::ios::floatfield);
    foutRGB.precision(6);
    string name = "rgb/" + f_str + ".png";
    foutRGB << f_str << " " << name << '\n';
    foutRGB.close();

    if (cv::imwrite(save_dir + "rgb/" + f_str + ".png", save_img, compression_params))
        ROS_INFO("Save img!");

    return ;
}

void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    ROS_INFO("Rx depth!");

    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);
    cv::Mat save_img = ptr->image;
    double t = ptr->header.stamp.toSec();
    
    string f_str = to_string(t);
    
    std::ofstream foutDep(save_dir + "depth.txt", std::ios::app);
    foutDep.setf(std::ios::fixed, std::ios::floatfield);
    foutDep.precision(6);
    string name = "depth/" + f_str + ".png";
    foutDep << f_str << " " << name << '\n';
    foutDep.close();            

    if (cv::imwrite(save_dir + "depth/" + f_str + ".png", save_img, compression_params))
        ROS_INFO("Save depth!");    

    return ;
}

void ekf_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &ekf_msg) {
    ROS_INFO("Rx ekf msg!");
    std::ofstream foutPose(save_dir + "groundtruth.txt", std::ios::app);
    foutPose.setf(std::ios::fixed, std::ios::floatfield);
    foutPose.precision(6);
    foutPose << ekf_msg->header.stamp.toSec() << " "
            << ekf_msg->pose.pose.position.x << " " << ekf_msg->pose.pose.position.y << " " << ekf_msg->pose.pose.position.z << " "
            << ekf_msg->pose.pose.orientation.x << " " << ekf_msg->pose.pose.orientation.y << " " << ekf_msg->pose.pose.orientation.z << " " << ekf_msg->pose.pose.orientation.w
            << '\n';
    foutPose.close();
    return ;
}

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg) {
    ROS_INFO("Rx odom msg!");
    std::ofstream foutOdom(save_dir + "odometry.txt", std::ios::app);
    foutOdom.setf(std::ios::fixed, std::ios::floatfield);
    foutOdom.precision(6);
    foutOdom << odom_msg->header.stamp.toSec() << " "
            << odom_msg->pose.pose.position.x << " " << odom_msg->pose.pose.position.y << " " << odom_msg->pose.pose.position.z << " "
            << odom_msg->pose.pose.orientation.x << " " << odom_msg->pose.pose.orientation.y << " " << odom_msg->pose.pose.orientation.z << " " << odom_msg->pose.pose.orientation.w
            << '\n';
    foutOdom.close();
    return ;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::Subscriber sub_depth = nh.subscribe("/mynteye/depth/image_raw", 500, depth_callback); // save depth img
    ros::Subscriber sub_image = nh.subscribe("/mynteye/left/image_color", 500, image_callback); // save color img
    ros::Subscriber sub_pose = nh.subscribe("/robot_pose_ekf/odom_combined", 1000, ekf_callback); // save groundtruth
    ros::Subscriber sub_odom = nh.subscribe("/base/odom", 1000, odom_callback); // save wheel odometry

    ROS_INFO("Sub Finish!");
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    save_dir = std::string(argv[1]);
    std::cout << "save dir is: " << save_dir << '\n';

    ros::spin();
    return 0;
}