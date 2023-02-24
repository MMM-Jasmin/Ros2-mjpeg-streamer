#include "webserver_node.hpp"
#include "Utils.h"

const double ONE_SECOND            = 1000.0; // One second in milliseconds

/**
 * @brief Contructor.
 */
WebserverNode::WebserverNode(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false)) {

	this->declare_parameter("rotation", 0);
	this->declare_parameter("debug", false);
	this->declare_parameter("topic", "");
	this->declare_parameter("fps_topic", "test/fps");
	this->declare_parameter("max_fps", 30.0f);
	this->declare_parameter("port", 7777);
	this->declare_parameter("out_width", 1080);
	this->declare_parameter("out_height", 1920);
	this->declare_parameter("print_fps", true);
	this->declare_parameter("FPS_STR", "FPS" );
	this->declare_parameter("qos_sensor_data", true);
	this->declare_parameter("qos_history_depth", 10);
	
}

/**
 * @brief Initialize image node.
 */
void WebserverNode::init() {

	std::string ros_topic, fps_topic;
	bool qos_sensor_data;
	int qos_history_depth;
	int output_port;

	this->get_parameter("topic", ros_topic);
	this->get_parameter("fps_topic", fps_topic);
	this->get_parameter("qos_sensor_data", qos_sensor_data);
	this->get_parameter("qos_history_depth", qos_history_depth);
	this->get_parameter("port", output_port);
	
	
	this->get_parameter("max_fps", m_maxFPS);
	this->get_parameter("out_width", m_out_width);
	this->get_parameter("out_height", m_out_height);
	this->get_parameter("rotation", m_rotation);
	this->get_parameter("FPS_STR", m_FPS_STR );
	this->get_parameter("print_fps", m_print_fps);


	if(qos_sensor_data){
		std::cout << "using ROS2 qos_sensor_data" << std::endl;
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
	
	// Create mjpeg writer with a choosen port.
	// TODO maybe parameterize the port ???
	//m_mjpeg_writer_ptr = new MJPEGWriter(output_port);
	m_streamer_ptr = new nadjieb::MJPEGStreamer();
	m_streamer_ptr->start(output_port);
	//center_writer_thr = new std::thread(write_to_mjpeg_writer,std::ref(post_draw_image));
	//mjpeg_writer_ptr->start(); //Starts the HTTP Server on the selected port

	m_image_small_subscription = this->create_subscription<sensor_msgs::msg::Image>( ros_topic, m_qos_profile, std::bind(&WebserverNode::imageSmallCallback, this, std::placeholders::_1));
	//cv::namedWindow(m_window_name_image_small, cv::WINDOW_AUTOSIZE);

	m_fps_publisher    		= this->create_publisher<std_msgs::msg::String>(fps_topic, m_qos_profile_sysdef);

	m_elapsedTime = 0;
	m_timer.Start();
}


/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void WebserverNode::imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {

	/*
	if(!m_color_image_last.empty()){
		m_color_image_for_send = m_color_image_last;
		m_mjpeg_writer_ptr->write(m_color_image_for_send);
		if(!m_webservice_started){
			m_mjpeg_writer_ptr->start(); //Starts the HTTP Server on the selected port
			m_webservice_started = true;
		}	
	}

	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_8UC3, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	
	cv::resize(color_image, color_image, Size(m_out_width, m_out_height), INTER_LINEAR);
	cv::cvtColor(color_image, m_color_image_last, cv::COLOR_RGB2BGR); 

	*/
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