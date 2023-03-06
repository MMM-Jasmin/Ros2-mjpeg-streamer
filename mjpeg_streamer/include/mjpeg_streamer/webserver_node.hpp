#pragma once
// SYSTEM
#include <chrono>
#include <iostream>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
// OPENCV
#include <opencv2/opencv.hpp>

// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"
#include <nadjieb/mjpeg_streamer.hpp>

#include "Timer.h"


/**
 * @brief Image viewer node class for receiving and visualizing fused image.
 */
class WebserverNode : public rclcpp::Node
{
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

public:
	WebserverNode(const std::string &name);
	void init();

private:
	nadjieb::MJPEGStreamer* m_streamer_ptr;

	bool m_print_fps;
	uint64_t m_frameCnt = 0;
	std::string m_FPS_STR = "";

	std::vector<std::string> m_ros_topic, m_ros_topic_sel;
	int m_topic_index = 0;

	Timer m_timer;        // Timer used to measure the time required for one iteration
	double m_elapsedTime; // Sum of the elapsed time, used to check if one second has passed

	time_point m_callback_time = hires_clock::now();
	time_point m_callback_time_image_small = hires_clock::now();
	time_point m_callback_time_depth = hires_clock::now();

	double m_loop_duration = 0.0;
	double m_loop_duration_image_small = 0.0;
	double m_loop_duration_depth = 0.0;

	rclcpp::QoS m_qos_profile = rclcpp::SystemDefaultsQoS();
	rclcpp::QoS m_qos_profile_sysdef = rclcpp::SystemDefaultsQoS();

	rclcpp::SubscriptionOptions m_cam_options;

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_topic_sel_subscription;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_fps_publisher 	=	 nullptr;

	void topicSelCallback(std_msgs::msg::String::SharedPtr img_msg);
	void imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	void PrintFPS(const float fps, const float itrTime);
	void CheckFPS(uint64_t* pFrameCnt);
	
};
