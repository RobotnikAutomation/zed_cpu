#include <memory>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace zed_cpu
{


class ZedCameraInfoHandler
{
  public:
    ZedCameraInfoHandler(std::string url, std::string frame_id, std::string topic_name, rclcpp::Node::SharedPtr node);
    ~ZedCameraInfoHandler()=default;
    void PublishCameraInfo(rclcpp::Time timestamp);

  private:
    rclcpp::Node::SharedPtr node_;
    std::string camera_frame_id_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    rclcpp::Publisher<camera_info_manager::CameraInfo>::SharedPtr camera_info_pub_;
};

class ZedCameraNode : public rclcpp::Node
{
public:
  ZedCameraNode();
  ~ZedCameraNode()=default;
  void run();
  void InitCameraInfo();

private:
  void CameraInit();
  void SensorInit();
  void PublishImages();
  void PublishIMU();

  bool camera_info_created_;
  std::string left_camera_frame_id_;
  std::string right_camera_frame_id_;
  std::string left_camera_info_url_;
  std::string right_camera_info_url_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::unique_ptr<image_transport::Publisher> left_image_pub_;
  std::unique_ptr<image_transport::Publisher> right_image_pub_;
  std::unique_ptr<sl_oc::video::VideoCapture> cap_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
  std::unique_ptr<ZedCameraInfoHandler> left_camera_info_;
  std::unique_ptr<ZedCameraInfoHandler> right_camera_info_;

};

} // namespace zed_cpu