#include "webserver_node.hpp"
#include "Utils.h"

const double ONE_SECOND            = 1000.0; // One second in milliseconds

/**
 * @brief Contructor.
 */
WebserverNode::WebserverNode(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false)) {

	this->declare_parameter("topic", std::vector<std::string>());
	this->declare_parameter("topic_sel_word", std::vector<std::string>());
	this->declare_parameter("topic_sel_topic", "");
	this->declare_parameter("fps_topic", "test/fps");
	this->declare_parameter("port", 7777);
	this->declare_parameter("print_fps", true);
	this->declare_parameter("FPS_STR", "FPS" );
	this->declare_parameter("qos_sensor_data", true);
	this->declare_parameter("qos_history_depth", 10);
}

/**
 * @brief Initialize image node.
 */
void WebserverNode::init() {

	std::string fps_topic, topic_sel_topic;
	bool qos_sensor_data;
	int qos_history_depth;
	int output_port;

	this->get_parameter("topic", m_ros_topic);
	this->get_parameter("topic_sel_word", m_ros_topic_sel);
	this->get_parameter("topic_sel_topic", topic_sel_topic);
	this->get_parameter("fps_topic", fps_topic);
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);
	this->get_parameter("port", output_port);
	this->get_parameter("FPS_STR", m_FPS_STR );
	this->get_parameter("print_fps", m_print_fps);


	if(qos_sensor_data){
		RCLCPP_INFO(this->get_logger(), "using ROS2 qos_sensor_data");
		m_qos_profile = rclcpp::SensorDataQoS();
	}


	m_qos_profile = m_qos_profile.keep_last(qos_history_depth);
	//m_qos_profile = m_qos_profile.lifespan(std::chrono::milliseconds(500));
	m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	

	m_qos_profile_sysdef = m_qos_profile_sysdef.keep_last(qos_history_depth);
	//m_qos_profile_sysdef = m_qos_profile_sysdef.lifespan(std::chrono::milliseconds(500));
	//m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	m_qos_profile_sysdef = m_qos_profile_sysdef.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	//m_qos_profile_sysdef = m_qos_profile_sysdef.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT );
	//m_qos_profile = m_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	//m_qos_profile = m_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	//m_qos_profile = m_qos_profile.deadline(std::chrono::nanoseconds(static_cast<int>(1e9 / 30)));
	
	m_streamer_ptr = new nadjieb::MJPEGStreamer();
	m_streamer_ptr->start(output_port);

	rclcpp::CallbackGroup::SharedPtr my_callback_cam_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	rclcpp::CallbackGroup::SharedPtr my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	rclcpp::SubscriptionOptions options;
	
	m_cam_options.callback_group = my_callback_cam_group;
	options.callback_group = my_callback_group;

	RCLCPP_INFO(this->get_logger(), "subscribing to %s", m_ros_topic[m_topic_index].c_str());

	m_image_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_ros_topic[m_topic_index], m_qos_profile, std::bind(&WebserverNode::imageSmallCallback, this, std::placeholders::_1), m_cam_options);
	m_topic_sel_subscription = this->create_subscription<std_msgs::msg::String>(topic_sel_topic, m_qos_profile, std::bind(&WebserverNode::topicSelCallback, this, std::placeholders::_1), options);
	
	m_fps_publisher    		= this->create_publisher<std_msgs::msg::String>(fps_topic, m_qos_profile_sysdef);

	m_elapsedTime = 0;
	m_timer.Start();
}

void WebserverNode::topicSelCallback(std_msgs::msg::String::SharedPtr topic_msg) {

	std::string str(topic_msg->data.data());
	if (str == "TOGGLE"){
		
		m_topic_index++;
		m_topic_index = m_topic_index % m_ros_topic_sel.size();
		m_image_subscription.reset();
		m_image_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_ros_topic[m_topic_index], m_qos_profile, std::bind(&WebserverNode::imageSmallCallback, this, std::placeholders::_1), m_cam_options);
		RCLCPP_INFO(this->get_logger(), "toggling stream to %s", m_ros_topic[m_topic_index].c_str());

	} else {

		auto it = std::find(m_ros_topic_sel.begin(), m_ros_topic_sel.end(), topic_msg->data.data());
		if (it == m_ros_topic_sel.end())
		{
			RCLCPP_INFO(this->get_logger(), "%s: not in list of topics..", topic_msg->data.data());
		} else {
  			auto index = std::distance(m_ros_topic_sel.begin(), it);
			if (index == m_topic_index){
				RCLCPP_INFO(this->get_logger(), "already on topic %s" , m_ros_topic[index].c_str());
			} else {
				RCLCPP_INFO(this->get_logger(),  "switching to topic %s" , m_ros_topic[index].c_str());
				m_topic_index = index;

				m_image_subscription.reset();
				m_image_subscription = this->create_subscription<sensor_msgs::msg::Image>(m_ros_topic[index], m_qos_profile, std::bind(&WebserverNode::imageSmallCallback, this, std::placeholders::_1), m_cam_options);
			}
		}
	}

}


/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void WebserverNode::imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {

	std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 100};

	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_8UC3, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR); 

	// http://localhost:8080/bgr
	std::vector<uchar> buff_bgr;
	cv::imencode(".jpg", color_image, buff_bgr, params);
	m_streamer_ptr->publish("/bgr", std::string(buff_bgr.begin(), buff_bgr.end()));

	m_frameCnt++;
	CheckFPS(&m_frameCnt);
}

void WebserverNode::CheckFPS(uint64_t* pFrameCnt)
	{
		m_timer.Stop();

		double itrTime      = m_timer.GetElapsedTimeInMilliSec();
		double fps;

		m_elapsedTime += itrTime;

		fps = 1000 / (m_elapsedTime / (*pFrameCnt));

		if (m_elapsedTime >= ONE_SECOND)
		{
			PrintFPS(fps, itrTime);

			*pFrameCnt    = 0;
			m_elapsedTime = 0;
		}

		m_timer.Start();
	}

void WebserverNode::PrintFPS(const float fps, const float itrTime)
{
		
	std::stringstream str("");

	if (fps == 0.0f)
			str << string_format("{\"%s\": 0.0}", m_FPS_STR.c_str());
	else
		str << string_format("{\"%s\": %.2f, \"lastCurrMSec\": %.2f }", m_FPS_STR.c_str(), fps, itrTime);

	auto message = std_msgs::msg::String();
	message.data = str.str();

	try{
		m_fps_publisher->publish(message);
	}
  	catch (...) {
    	RCLCPP_INFO(this->get_logger(), "m_fps_publisher: hmm publishing dets has failed!! ");
  	}
		
	if (m_print_fps)
		RCLCPP_INFO(this->get_logger(), message.data.c_str());

}