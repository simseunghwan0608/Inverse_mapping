/**
 * @file      inverse_mapping.cpp
 * @brief     
 *
 * @date      2025-2-2 created by Seunghwan Shim (paul800@hanyang.ac.kr)
 */

#include "inverse_mapping.hpp"




InverseMappingNode::InverseMappingNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : Node(node_name, options) 
{
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters

    // Intrinsic
    this->declare_parameter<double>("intrinsic_fx", 0.0);
    this->declare_parameter<double>("intrinsic_fy", 0.0);
    this->declare_parameter<double>("intrinsic_cx", 0.0);
    this->declare_parameter<double>("intrinsic_cy", 0.0);

    // Extrinsic 
    this->declare_parameter<double>("extrinsic_00", 1.0);
    this->declare_parameter<double>("extrinsic_01", 0.0);
    this->declare_parameter<double>("extrinsic_02", 0.0);
    this->declare_parameter<double>("extrinsic_03", 0.0);

    this->declare_parameter<double>("extrinsic_10", 0.0);
    this->declare_parameter<double>("extrinsic_11", 1.0);
    this->declare_parameter<double>("extrinsic_12", 0.0);
    this->declare_parameter<double>("extrinsic_13", 0.0);

    this->declare_parameter<double>("extrinsic_20", 0.0);
    this->declare_parameter<double>("extrinsic_21", 0.0);
    this->declare_parameter<double>("extrinsic_22", 1.0);
    this->declare_parameter<double>("extrinsic_23", 0.0);

    this->declare_parameter<double>("extrinsic_30", 0.0);
    this->declare_parameter<double>("extrinsic_31", 0.0);
    this->declare_parameter<double>("extrinsic_32", 0.0);
    this->declare_parameter<double>("extrinsic_33", 1.0);

    // Distortion
    this->declare_parameter<double>("dist_k1", 0.0);
    this->declare_parameter<double>("dist_k2", 0.0);
    this->declare_parameter<double>("dist_p1", 0.0);
    this->declare_parameter<double>("dist_p2", 0.0);
    this->declare_parameter<double>("dist_k3", 0.0);

    // Loop rate
    this->declare_parameter<double>("loop_rate_hz", 30.0);


    ProcessParams();

    // camera params update
    UpdateIntrinsic();
    UpdateExtrinsic();
    UpdateDistortion();
    MakeHmatrix();

    // Subscriber init
    s_lane_camera_points_ = this->create_subscription<sensor_msgs::msg::Image>(
        "YOLO", qos_profile, std::bind(&InverseMappingNode::CallbackYolo, this, std::placeholders::_1));

    // Publisher init
    p_lane_point_baselink_ = this->create_publisher<custom_msgs::msg::MyLane>(
        "lane_baselink", qos_profile);

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate_hz_)),
        [this]() { this->Run(); }); 
}

InverseMappingNode::~InverseMappingNode() {}

void InverseMappingNode::ProcessParams() 
{
    // Intrinsic
    this->get_parameter("intrinsic_fx", intrinsic_fx_);
    this->get_parameter("intrinsic_fy", intrinsic_fy_);
    this->get_parameter("intrinsic_cx", intrinsic_cx_);
    this->get_parameter("intrinsic_cy", intrinsic_cy_);

    // Extrinsic 
    this->get_parameter("extrinsic_00", extrinsic_00_);
    this->get_parameter("extrinsic_01", extrinsic_01_);
    this->get_parameter("extrinsic_02", extrinsic_02_);
    this->get_parameter("extrinsic_03", extrinsic_03_);

    this->get_parameter("extrinsic_10", extrinsic_10_);
    this->get_parameter("extrinsic_11", extrinsic_11_);
    this->get_parameter("extrinsic_12", extrinsic_12_);
    this->get_parameter("extrinsic_13", extrinsic_13_);

    this->get_parameter("extrinsic_20", extrinsic_20_);
    this->get_parameter("extrinsic_21", extrinsic_21_);
    this->get_parameter("extrinsic_22", extrinsic_22_);
    this->get_parameter("extrinsic_23", extrinsic_23_);

    this->get_parameter("extrinsic_30", extrinsic_30_);
    this->get_parameter("extrinsic_31", extrinsic_31_);
    this->get_parameter("extrinsic_32", extrinsic_32_);
    this->get_parameter("extrinsic_33", extrinsic_33_);

    // Distortion
    this->get_parameter("dist_k1", dist_k1_);
    this->get_parameter("dist_k2", dist_k2_);
    this->get_parameter("dist_p1", dist_p1_);
    this->get_parameter("dist_p2", dist_p2_);
    this->get_parameter("dist_k3", dist_k3_);

    // Loop rate
    this->get_parameter("loop_rate_hz", loop_rate_hz_);

    
}

void InverseMappingNode::UpdateIntrinsic(){
    K_ = (cv::Mat_<double>(3,3) << intrinsic_fx_, 0 , intrinsic_cx_,
                                    0, intrinsic_fy_, intrinsic_cy_,
                                    0,             0,              1);

}

void InverseMappingNode::UpdateExtrinsic(){
    Ext_ = (cv::Mat_<double>(4,4) << extrinsic_00_, extrinsic_01_, extrinsic_02_, extrinsic_03_,
                                     extrinsic_10_, extrinsic_11_, extrinsic_12_, extrinsic_13_,
                                     extrinsic_20_, extrinsic_21_, extrinsic_22_, extrinsic_23_,
                                     extrinsic_30_, extrinsic_31_, extrinsic_32_, extrinsic_33_);      

}

void InverseMappingNode::UpdateDistortion(){
    D_ = (cv::Mat_<double>(1, 5) <<
          dist_k1_, dist_k2_, dist_p1_, dist_p2_, dist_k3_);

}


void InverseMappingNode::MakeHmatrix(){
    cv::Mat A;
    A =(cv::Mat_<double>(3,3) << extrinsic_00_, extrinsic_01_, extrinsic_03_,
                                 extrinsic_10_, extrinsic_11_, extrinsic_13_,
                                 extrinsic_20_, extrinsic_21_, extrinsic_23_);
    H_ = K_ *  A;
    H_inv_ = H_.inv();

}



void InverseMappingNode::Run() 
{

    // output variable
    custom_msgs::msg::MyLane my_lane;


    cv::Mat img;
    {
        std::lock_guard<std::mutex> lock(mutex_image_);
        if (!cv_ptr_) return;
        img = cv_ptr_->image;
        my_lane.header.stamp = cv_ptr_->header.stamp;
    }


    // change to baselink (baselink is under floor of camera)
    std::vector<cv::Point2i> image_none_zero_i;
    cv::findNonZero(img, image_none_zero_i);
    std::vector<cv::Point2f> image_none_zero_f(image_none_zero_i.begin(),image_none_zero_i.end());
    std::vector<cv::Point2f> world_points;
    cv::perspectiveTransform(image_none_zero_f, world_points, H_inv_);


    // fill message & publish  
    my_lane.header.frame_id ="base_link";
    for(const auto& pt : world_points){
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0;
        my_lane.lane_points.push_back(p);
    }

    p_lane_point_baselink_ -> publish(my_lane);


}



int main(int argc, char **argv) {
    std::string node_name = "inverse_mapping_node";

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InverseMappingNode>(node_name));
    rclcpp::shutdown();
    return 0;
}