#include <iostream>
#include <string>
#include <list>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>

int main(int argc, char** argv) {
  std::string in_bag, out_folder;

  //========= Handling Program options =========
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("inbag,i",
          boost::program_options::value<std::string>(
              &in_bag)->default_value(""),
          "Input bag file")
      ("outfolder,o",
          boost::program_options::value<std::string>(
              &out_folder)->default_value(""),
          "Output folder");

  boost::program_options::positional_options_description pdesc;
  pdesc.add("inbag", 1);
  pdesc.add("outfolder", 2);

  boost::program_options::variables_map vm;
  boost::program_options::store(
      boost::program_options::command_line_parser(
          argc, argv).options(desc).positional(pdesc).run(),
      vm);
  boost::program_options::notify(vm);

  if (vm.count("help") || in_bag.empty()) {
    std::cout << desc << std::endl;
    return 1;
  }

  // Startup this node
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger(__func__), "Starting up");

  if (out_folder.empty())
    out_folder = boost::filesystem::path(in_bag).parent_path().string();
  if (out_folder.back() != '/')
    out_folder += "/";
  if (!boost::filesystem::exists(out_folder))
    boost::filesystem::create_directories(out_folder);

  if (!boost::filesystem::exists(out_folder + "color"))
    boost::filesystem::create_directories(out_folder + "color");
  if (!boost::filesystem::exists(out_folder + "infra1"))
    boost::filesystem::create_directories(out_folder + "infra1");
  if (!boost::filesystem::exists(out_folder + "infra2"))
    boost::filesystem::create_directories(out_folder + "infra2");
  if (!boost::filesystem::exists(out_folder + "depth"))
    boost::filesystem::create_directories(out_folder + "depth");
  if (!boost::filesystem::exists(out_folder + "aligned_depth_to_color"))
    boost::filesystem::create_directories(out_folder + "aligned_depth_to_color");

  std::string imu_topic = "/camera/camera/imu";
  std::string color_topic = "/camera/camera/color/image_raw";
  std::string infra1_topic = "/camera/camera/infra1/image_rect_raw";
  std::string infra2_topic = "/camera/camera/infra2/image_rect_raw";
  std::string depth_topic = "/camera/camera/depth/image_rect_raw";
  std::string aligned_d2c_topic =
      "/camera/camera/aligned_depth_to_color/image_raw";

  // Load rosbag here, and find messages we can play
  rosbag2_cpp::Reader bag_reader;
  bag_reader.open(in_bag);

  RCLCPP_INFO_STREAM(
      rclcpp::get_logger(__func__),
      "Extracting D400 RGB, Infra, Depth, and IMU data from " +
      in_bag + " to " + out_folder);

  FILE* color_file = fopen((out_folder + "color.txt").c_str(), "wb");
  FILE* infra1_file = fopen((out_folder + "infra1.txt").c_str(), "wb");
  FILE* infra2_file = fopen((out_folder + "infra2.txt").c_str(), "wb");
  FILE* depth_file = fopen((out_folder + "depth.txt").c_str(), "wb");
  FILE* aligned_depth_to_color_file = fopen(
      (out_folder + "aligned_depth_to_color.txt").c_str(), "wb");
  FILE* imu_file = fopen((out_folder + "imu.txt").c_str(), "wb");
  fprintf(imu_file, "#Time Gx Gy Gz Ax Ay Ax \n");

  // Step through the rosbag and send to algo methods
  while (bag_reader.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr bag_message =
        bag_reader.read_next();
    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

    // Handle color message
    if (bag_message->topic_name == color_topic) {
      sensor_msgs::msg::Image::SharedPtr image_msg =
          std::make_shared<sensor_msgs::msg::Image>();

      rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
      serialization.deserialize_message(&serialized_msg, image_msg.get());
      cv_bridge::CvImageConstPtr image_ptr;
      try {
        image_ptr = cv_bridge::toCvShare(image_msg,
                                         sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger(__func__),
                     "cv_bridge exception: %s", e.what());
      }

      cv::Mat image = image_ptr->image;
      if (image.empty()) continue;
      char tmp[30] = {0};
      sprintf(tmp, "%.6f", rclcpp::Time(image_ptr->header.stamp).seconds());
      std::string time(tmp);
      fprintf(color_file, "%s %s\n",
              time.c_str(), ("color/" + time + ".png").c_str());
      fflush(color_file);
      cv::imwrite(out_folder + "color/" + time + ".png", image);
    }

    // Handle infra1 message
    if (bag_message->topic_name == infra1_topic) {
      sensor_msgs::msg::Image::SharedPtr image_msg =
          std::make_shared<sensor_msgs::msg::Image>();

      rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
      serialization.deserialize_message(&serialized_msg, image_msg.get());
      cv_bridge::CvImageConstPtr image_ptr;
      try {
        image_ptr = cv_bridge::toCvShare(image_msg,
                                         sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger(__func__),
                     "cv_bridge exception: %s", e.what());
      }

      cv::Mat image = image_ptr->image;
      if (image.empty()) continue;
      char tmp[30] = {0};
      sprintf(tmp, "%.6f", rclcpp::Time(image_ptr->header.stamp).seconds());
      std::string time(tmp);
      fprintf(infra1_file, "%s %s\n",
              time.c_str(), ("infra1/" + time + ".png").c_str());
      fflush(infra1_file);
      cv::imwrite(out_folder + "infra1/" + time + ".png", image);
    }

    // Handle infra2 message
    if (bag_message->topic_name == infra2_topic) {
      sensor_msgs::msg::Image::SharedPtr image_msg =
          std::make_shared<sensor_msgs::msg::Image>();

      rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
      serialization.deserialize_message(&serialized_msg, image_msg.get());
      cv_bridge::CvImageConstPtr image_ptr;
      try {
        image_ptr = cv_bridge::toCvShare(image_msg,
                                         sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger(__func__),
                     "cv_bridge exception: %s", e.what());
      }

      cv::Mat image = image_ptr->image;
      if (image.empty()) continue;
      char tmp[30] = {0};
      sprintf(tmp, "%.6f", rclcpp::Time(image_ptr->header.stamp).seconds());
      std::string time(tmp);
      fprintf(infra2_file, "%s %s\n",
              time.c_str(), ("infra2/" + time + ".png").c_str());
      fflush(infra2_file);
      cv::imwrite(out_folder + "infra2/" + time + ".png", image);
    }

    // Handle depth message
    if (bag_message->topic_name == depth_topic) {
      sensor_msgs::msg::Image::SharedPtr image_msg =
          std::make_shared<sensor_msgs::msg::Image>();

      rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
      serialization.deserialize_message(&serialized_msg, image_msg.get());
      cv_bridge::CvImageConstPtr image_ptr;
      try {
        image_ptr = cv_bridge::toCvShare(image_msg,
                                         sensor_msgs::image_encodings::TYPE_16UC1);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger(__func__),
                     "cv_bridge exception: %s", e.what());
      }

      cv::Mat image = image_ptr->image;
      if (image.empty()) continue;
      char tmp[30] = {0};
      sprintf(tmp, "%.6f", rclcpp::Time(image_ptr->header.stamp).seconds());
      std::string time(tmp);
      fprintf(depth_file, "%s %s\n",
              time.c_str(), ("depth/" + time + ".png").c_str());
      fflush(depth_file);
      cv::imwrite(out_folder + "depth/" + time + ".png", image);
    }

    // Handle aligned_d2c_topic message
    if (bag_message->topic_name == aligned_d2c_topic) {
      sensor_msgs::msg::Image::SharedPtr image_msg =
          std::make_shared<sensor_msgs::msg::Image>();

      rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
      serialization.deserialize_message(&serialized_msg, image_msg.get());
      cv_bridge::CvImageConstPtr image_ptr;
      try {
        image_ptr = cv_bridge::toCvShare(image_msg,
                                         sensor_msgs::image_encodings::TYPE_16UC1);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger(__func__),
                     "cv_bridge exception: %s", e.what());
      }

      cv::Mat image = image_ptr->image;
      if (image.empty()) continue;
      char tmp[30] = {0};
      sprintf(tmp, "%.6f", rclcpp::Time(image_ptr->header.stamp).seconds());
      std::string time(tmp);
      fprintf(aligned_depth_to_color_file, "%s %s\n",
              time.c_str(), ("aligned_depth_to_color/" + time + ".png").c_str());
      fflush(aligned_depth_to_color_file);
      cv::imwrite(out_folder + "aligned_depth_to_color/" + time + ".png", image);
    }

    // Handle IMU message
    if (bag_message->topic_name == imu_topic) {
      sensor_msgs::msg::Imu::SharedPtr imu_msg =
          std::make_shared<sensor_msgs::msg::Imu>();

      rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
      serialization.deserialize_message(&serialized_msg, imu_msg.get());

      fprintf(imu_file, "%.8f %.8f %.8f %.8f %.8f %.8f %.8f\n",
              rclcpp::Time(imu_msg->header.stamp).seconds(),
              imu_msg->angular_velocity.x,
              imu_msg->angular_velocity.y,
              imu_msg->angular_velocity.z,
              imu_msg->linear_acceleration.x,
              imu_msg->linear_acceleration.y,
              imu_msg->linear_acceleration.z);
      fflush(imu_file);
    }
  }

  bag_reader.close();

  fclose(color_file);
  fclose(infra1_file);
  fclose(infra2_file);
  fclose(depth_file);
  fclose(aligned_depth_to_color_file);
  fclose(imu_file);

  rclcpp::shutdown();

  return 0;
}


