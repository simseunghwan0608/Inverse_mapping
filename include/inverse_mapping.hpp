/**
 * @file      inverse_mapping.hpp
 * @brief     inverse_mapping image u,v to baselink
 * 
 * @date      2025-2-1 created by Seunghwan Shim (paul800@hanyang.ac.kr)
 */


#ifndef __INVERSE_MAPPING_HPP__
#define __INVERSE_MAPPING_HPP__
#pragma once

// STD
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// Config
// 파라미터 너무 적어서 따로 부르지 x 생략

// msg
#include "custom_msgs/msg/my_lane.hpp"
#include <sensor_msgs/msg/image.hpp>




class InverseMappingNode : public rclcpp::Node {
public:
    explicit InverseMappingNode(const std::string & node_name,
                          const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~InverseMappingNode();

    void ProcessParams();
    void Run();

private:


    // Callback Functions
    inline void CallbackYolo(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        std::lock_guard<std::mutex> lock(mutex_image_);
        cv_ptr_ = cv_bridge::toCvCopy(msg,"mono8");
    }

    // Fuctions

    void UpdateIntrinsic();
    void UpdateExtrinsic();
    void UpdateDistortion();
    void MakeHmatrix();
                   
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr             s_lane_camera_points_;
    
    
    // Publisher
    rclcpp::Publisher<custom_msgs::msg::MyLane>::SharedPtr     p_lane_point_baselink_;


    // Timer
    rclcpp::TimerBase::SharedPtr t_run_node_;


    // Mutex
    std::mutex mutex_image_;


    // Variable

    // Image pointer
    cv_bridge::CvImagePtr cv_ptr_;


    // Intrinsic

    double intrinsic_fx_;
    double intrinsic_fy_;
    double intrinsic_cx_;
    double intrinsic_cy_;

    cv::Mat K_;

    // Extrinsic

    double extrinsic_00_; double extrinsic_01_; double extrinsic_02_; double extrinsic_03_;
    double extrinsic_10_; double extrinsic_11_; double extrinsic_12_; double extrinsic_13_;
    double extrinsic_20_; double extrinsic_21_; double extrinsic_22_; double extrinsic_23_;
    double extrinsic_30_; double extrinsic_31_; double extrinsic_32_; double extrinsic_33_;

    cv::Mat Ext_;

    // Distortion

    double dist_k1_;
    double dist_k2_;
    double dist_p1_;
    double dist_p2_;
    double dist_k3_;
    cv::Mat D_;

    // H matrix

    cv::Mat H_;
    cv::Mat H_inv_;

    // loop_hz
    double loop_rate_hz_;

    

};

#endif //__INVERSE_MAPPING_HPP__
