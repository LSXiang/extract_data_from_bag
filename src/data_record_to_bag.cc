#include <iostream>
#include <string>
#include <list>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>

int main(int argc, char** argv) {
  std::string in_path, out_bag;

  //========= Handling Program options =========
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("inpath,i",
       boost::program_options::value<std::string>(
           &in_path)->default_value("./"),
       "Input data path")
      ("outbag,o",
       boost::program_options::value<std::string>(
           &out_bag)->default_value(""),
       "Output bag file");

  boost::program_options::positional_options_description pdesc;
  pdesc.add("inpath", 1);
  pdesc.add("outbag", 2);

  boost::program_options::variables_map vm;
  boost::program_options::store(
      boost::program_options::command_line_parser(
          argc, argv).options(desc).positional(pdesc).run(),
      vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || out_bag.empty()) {
    std::cout << desc << std::endl;
    return 1;
  }

  in_path = in_path.substr(0, in_path.find_last_of('/')) + "/";

  std::string imu_file = in_path + "imu.txt";
  std::string color_file = in_path + "color1.txt";
  std::string infra1_file = in_path + "infra1.txt";
  std::string infra2_file = in_path + "infra2.txt";
  std::string depth_file = in_path + "depth.txt";
  std::string aligned_d2c_file = in_path + "aligned_depth_to_color.txt";

  std::string imu_topic = "/feynman_camera/default/imu0_single";
  std::string color_topic = "/feynman_camera/default/rgb0/image_rect_color";
  std::string infra1_topic = "/feynman_camera/default/leftir/image_rect";
  std::string infra2_topic = "/feynman_camera/default/rightir/image_rect";
  std::string depth_topic = "/feynman_camera/default/depth/image_raw";
  std::string aligned_d2c_topic =
      "/feynman_camera/default/depthalignrgb/image_raw";

  std::vector<std::pair<std::string, std::string>> datas = {
      {color_file, color_topic},
      {infra1_file, infra1_topic},
      {infra2_file, infra2_topic},
      {depth_file, depth_topic},
      {aligned_d2c_file, aligned_d2c_topic}
  };

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rosbag_writer");

  rosbag2_cpp::Writer rosbag_writer;
  rosbag_writer.open(out_bag);

  // Read IMU
  {
    FILE* f = fopen(imu_file.c_str(), "r");
    if (!f) {
      RCLCPP_WARN_STREAM(node->get_logger(),
                         "Cannot be opened the file: " << imu_file);
      return 1;
    }
    char comment[100];
    fgets(comment, sizeof(comment), f);
    double timestamp;
    RCLCPP_INFO(node->get_logger(), "Writing %s to bag", "imu");
    for (sensor_msgs::msg::Imu::SharedPtr imu_msg =
            std::make_shared<sensor_msgs::msg::Imu>();
         fscanf(f, "%lf %lf %lf %lf %lf %lf %lf", &timestamp,
                &(imu_msg->angular_velocity.x),
                &(imu_msg->angular_velocity.y),
                &(imu_msg->angular_velocity.z),
                &(imu_msg->linear_acceleration.x),
                &(imu_msg->linear_acceleration.y),
                &(imu_msg->linear_acceleration.z)) != EOF;) {
      imu_msg->header.frame_id = "imu";
      imu_msg->header.stamp =
          rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
      rosbag_writer.write<sensor_msgs::msg::Imu>(
          *imu_msg, imu_topic, imu_msg->header.stamp);
    }
    fclose(f);
  }

  // Read image
  for (const auto& item : datas) {
    FILE* f = fopen(item.first.c_str(), "r");
    if (!f) {
      RCLCPP_WARN_STREAM(node->get_logger(),
                         "Cannot be opened the file: " << item.first);
      continue;
    }
    std::string frame_id = item.first.substr(item.first.rfind('/') + 1,
                                             item.first.rfind('.'));
    RCLCPP_INFO(node->get_logger(), "Writing %s to bag", frame_id.c_str());
    double timestamp;
    char img_name[100];
    while (fscanf(f, "%lf %s", &timestamp, img_name) != EOF) {
      cv::Mat image =
          cv::imread(in_path + std::string(img_name), cv::IMREAD_UNCHANGED);
      if (image.empty()) {
        RCLCPP_WARN_STREAM(
            node->get_logger(),
            "Cannot be opened the image: " << in_path + std::string(img_name));
        continue;
      }
      std::string encode;
      switch (image.type()) {
        case CV_8U:
          encode = sensor_msgs::image_encodings::MONO8;
          break;

        case CV_16U:
          encode = sensor_msgs::image_encodings::TYPE_16UC1;
          break;

        case CV_8UC3:
          encode = sensor_msgs::image_encodings::RGB8;
          cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
          break;

        default:
          RCLCPP_WARN(node->get_logger(), "Image type not implemented yet");
          break;
      }
      if (encode.empty()) continue;
      auto img_msg = cv_bridge::CvImage(
          std_msgs::msg::Header(), encode, image).toImageMsg();
      img_msg->header.frame_id = frame_id;
      img_msg->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp*1e9));
      rosbag_writer.write<sensor_msgs::msg::Image>(
          *img_msg, item.second, img_msg->header.stamp);
    }
    fclose(f);
  }

  RCLCPP_INFO(node->get_logger(), "Done.");

  rclcpp::shutdown();
  return 0;
}