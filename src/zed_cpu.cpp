#include <zed_cpu.hpp>

#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

namespace zed_cpu
{

ZedCameraInfoHandler::ZedCameraInfoHandler(std::string url, std::string frame_id, std::string topic_name, rclcpp::Node::SharedPtr node)
: node_{node}
, camera_frame_id_{frame_id}
, camera_info_manager_{std::make_shared<camera_info_manager::CameraInfoManager>(node_.get(), "zed_camera", url)}
, camera_info_pub_{node_->create_publisher<camera_info_manager::CameraInfo>(topic_name, 1)}
{
  camera_info_manager_->loadCameraInfo(url);
}

void ZedCameraInfoHandler::PublishCameraInfo(rclcpp::Time timestamp)
{
  if (camera_info_manager_->isCalibrated()) {
    sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_manager_->getCameraInfo();
    camera_info_msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = camera_frame_id_;
    camera_info_pub_->publish(camera_info_msg);
  }
  else
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Camera is not calibrated. Cannot publish camera info.");
  }
}

ZedCameraNode::ZedCameraNode() : Node("zed_camera", "zed_camera")
{
  this->declare_parameter<std::string>("left_camera_info_frame_id", "zed_camera_frame");
  this->declare_parameter<std::string>("right_camera_info_frame_id", "zed_camera_frame");
  this->declare_parameter<std::string>("left_camera_info_url", "");
  this->declare_parameter<std::string>("right_camera_info_url", "");

  this->get_parameter("left_camera_info_frame_id", left_camera_frame_id_);
  this->get_parameter("right_camera_info_frame_id", right_camera_frame_id_);
  this->get_parameter("left_camera_info_url", left_camera_info_url_);
  this->get_parameter("right_camera_info_url", right_camera_info_url_);

  camera_info_created_ = false;
  // ROS initialization
  left_image_pub_ = std::make_unique<image_transport::Publisher>(image_transport::create_publisher(this, "rgb/left_image"));
  right_image_pub_ = std::make_unique<image_transport::Publisher>(image_transport::create_publisher(this, "rgb/right_image"));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);

  CameraInit();
  SensorInit();


  RCLCPP_INFO(this->get_logger(), "Node started");
}

void ZedCameraNode::InitCameraInfo()
{
  if (!camera_info_created_)
  {
    left_camera_info_ = std::make_unique<ZedCameraInfoHandler>(
      left_camera_info_url_,
      left_camera_frame_id_,
      "rgb/left_camera/camera_info",
      this->shared_from_this()
    );

    right_camera_info_ = std::make_unique<ZedCameraInfoHandler>(
      right_camera_info_url_,
      right_camera_frame_id_,
      "rgb/right_camera/camera_info",
      this->shared_from_this()
    );
    camera_info_created_ = true;
  }
}

void ZedCameraNode::run()
{
  PublishImages();
  PublishIMU();
}

void ZedCameraNode::CameraInit()
{
  // Initialize ZED camera
  sl_oc::video::VideoParams params;
  params.res = sl_oc::video::RESOLUTION::HD720;
  params.fps = sl_oc::video::FPS::FPS_60;
  params.verbose = sl_oc::VERBOSITY::ERROR;

  // Create Video Capture
  cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
  if (!cap_->initializeVideo()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open camera video capture");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Connected to camera sn: " << cap_->getSerialNumber() << " [" << cap_->getDeviceName() << "]");
}

void ZedCameraNode::SensorInit()
{
  sens_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

  std::vector<int> devs = sens_->getDeviceList();

  if (devs.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "No available ZED 2, ZED 2i or ZED Mini cameras");
    rclcpp::shutdown();
    return;
  }

  uint16_t fw_maior;
  uint16_t fw_minor;
  sens_->getFirmwareVersion(fw_maior, fw_minor);
  RCLCPP_INFO_STREAM(
    this->get_logger(), "Connected to IMU firmware version: " << std::to_string(fw_maior) << "."
                                                        << std::to_string(fw_minor));

  // Initialize the sensors
  if (!sens_->initializeSensors(devs[0])) {
    RCLCPP_ERROR(this->get_logger(), "IMU initialize failed");
    rclcpp::shutdown();
    return;
  }
}

void ZedCameraNode::PublishImages()
{
  // Get last available frame
  const sl_oc::video::Frame frame = cap_->getLastFrame();

  // Process and publish the frame
  if (frame.data != nullptr) {
    cv::Mat frame_yuv = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frame_bgr;
    cv::cvtColor(frame_yuv, frame_bgr, cv::COLOR_YUV2BGR_YUYV);

    // Split the frame into left and right images
    cv::Mat left_img = frame_bgr(cv::Rect(0, 0, frame_bgr.cols / 2, frame_bgr.rows));
    cv::Mat right_img =
      frame_bgr(cv::Rect(frame_bgr.cols / 2, 0, frame_bgr.cols / 2, frame_bgr.rows));

    // Convert the OpenCV images to ROS image messages
    sensor_msgs::msg::Image::SharedPtr left_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_img).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr right_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_img).toImageMsg();
    
    auto now = this->get_clock()->now();
    left_msg->header.stamp = now;
    right_msg->header.stamp = now;
    left_msg->header.frame_id = left_camera_frame_id_;
    right_msg->header.frame_id = right_camera_frame_id_;

    // Publish the left and right image messages
    left_image_pub_->publish(left_msg);
    right_image_pub_->publish(right_msg);

    if (camera_info_created_)
    {
      left_camera_info_->PublishCameraInfo(now);
      right_camera_info_->PublishCameraInfo(now);
    }
  }
}

void ZedCameraNode::PublishIMU()
{
  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(5000);

  if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_frame";

    // Convert the IMU data to the sensor_msgs/Imu message fields
    imu_msg.linear_acceleration.x = -imu_data.aX;
    imu_msg.linear_acceleration.y = imu_data.aY;
    imu_msg.linear_acceleration.z = imu_data.aZ;

    imu_msg.angular_velocity.x = -imu_data.gX;
    imu_msg.angular_velocity.y = imu_data.gY;
    imu_msg.angular_velocity.z = imu_data.gZ;

    // Publish the sensor_msgs/Imu message
    imu_pub_->publish(imu_msg);
  }
}

}  // namespace zed_cpu