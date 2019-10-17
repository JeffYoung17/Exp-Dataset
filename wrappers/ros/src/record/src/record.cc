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

string save_dir = "/home/fqy/data/zjuData/";

vector<int> compression_params;

queue<pair<double, cv::Mat>> rgb_buf;
queue<pair<double, cv::Mat>> depth_buf;
std::mutex buf_mutex;

// callback of camera, saved as image
void image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    ROS_INFO("Rx img!");
    buf_mutex.lock();
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat save_img = ptr->image;
    double t = ptr->header.stamp.toSec();
    rgb_buf.push(make_pair(t, save_img.clone()));
    buf_mutex.unlock();

    return ;
}

void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    ROS_INFO("Rx depth!");
    buf_mutex.lock();
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);
    cv::Mat save_img = ptr->image;
    double t = ptr->header.stamp.toSec();
    depth_buf.push(make_pair(t, save_img.clone()));
    buf_mutex.unlock();
    
    return ;
}

void save_rgb() {
    while (1) {
        if (!rgb_buf.empty()) {
            buf_mutex.lock();
            auto img = rgb_buf.front();
            string f_str = to_string(img.first);
            // cv::imshow("rgb image", img.second);
            // string test = save_dir + "rgb/" + f_str + ".png";
            // std::cout << test << std::endl;
            if (cv::imwrite(save_dir + "rgb/" + f_str + ".png", img.second, compression_params))
                ROS_INFO("Save img!");
            rgb_buf.pop();
            // cv::waitKey(0);
            buf_mutex.unlock();
        } else {
            std::chrono::milliseconds dura(30);
            std::this_thread::sleep_for(dura);
            continue;
        }
    }
}

void save_depth() {
    while (1) {
        if (!depth_buf.empty()) {
            buf_mutex.lock();
            auto depth = depth_buf.front();
            string f_str = to_string(depth.first);
            // cv::imshow("depth image", depth.second);
            if (cv::imwrite(save_dir + "depth/" + f_str + ".png", depth.second, compression_params))
                ROS_INFO("Save depth!");
            depth_buf.pop();
            // cv::waitKey(0);
            buf_mutex.unlock();
        } else {
            std::chrono::milliseconds dura(30);
            std::this_thread::sleep_for(dura);
            continue;
        }
    }
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::Subscriber sub_depth = nh.subscribe("/mynteye/depth/image_raw", 200, depth_callback); // save depth img
    ros::Subscriber sub_image = nh.subscribe("/mynteye/left/image_color", 200, image_callback); // save color img
    ros::Subscriber sub_pose = nh.subscribe("/robot_pose_ekf/odom_combined", 200, ekf_callback); // save groundtruth

    ROS_INFO("Sub Finish!");
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    std::thread rgb_thread(save_rgb);
    rgb_thread.detach();
    std::thread depth_thread(save_depth);
    depth_thread.detach();
    ROS_INFO("Thread Start!");

    // std::ofstream foutPose(save_dir + "groundtruth.txt", std::ios::app);
    // tf::TransformListener listener;
    // ros::Rate rate(10.0);
    // while (nh.ok()) {
    //     // save tf
    //     tf::StampedTransform transform; // from target_frame to source_frame
    //     try {
    //         listener.lookupTransform("/odom", "/map", ros::Time(0), transform); // target_frame, source_frame
    //     }
    //     catch (tf::TransformException &ex) {
    //         ROS_ERROR("%s",ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }
        
    //     foutPose.setf(std::ios::fixed, std::ios::floatfield);
    //     foutPose.precision(6);
    //     foutPose << transform.stamp_.toSec() << " "
    //             << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << " "
    //             << transform.getRotation().x() << " " << transform.getRotation().y() << " " << transform.getRotation().z() << " " << transform.getRotation().w()
    //             << '\n';
    //     foutPose.close();
    //     rate.sleep();
    // }

    ros::spin();
    return 0;
}