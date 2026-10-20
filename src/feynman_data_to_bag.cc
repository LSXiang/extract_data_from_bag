#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {
namespace fs = std::filesystem;

struct ImageEntry {
  fs::path path;
  std::string topic;
  int64_t timestamp_ns;
};

int64_t timestampFromFilename(const fs::path& path, const std::string& topic) {
  // The final field is an integer number of microseconds, not a decimal fraction:
  // e.g. 17976 means 017976 us, not 179760 us.
  const std::regex pattern(
      "^" + topic + R"(-(\d{4})-(\d{1,2})-(\d{1,2})-(\d{1,2})-(\d{1,2})-(\d{1,2})-(\d{1,6})$)");
  std::smatch match;
  const std::string stem = path.stem().string();
  if (!std::regex_match(stem, match, pattern)) {
    throw std::runtime_error("Invalid image filename: " + path.string());
  }

  const int year = std::stoi(match[1]);
  const int month = std::stoi(match[2]);
  const int day = std::stoi(match[3]);
  const int hour = std::stoi(match[4]);
  const int minute = std::stoi(match[5]);
  const int second = std::stoi(match[6]);
  const int microsecond = std::stoi(match[7]);
  if (hour > 23 || minute > 59 || second > 59) {
    throw std::runtime_error("Invalid clock time: " + path.string());
  }

  try {
    // Interpret filenames as Beijing time (UTC+8), independent of host TZ.
    const boost::posix_time::ptime local_time(
        boost::gregorian::date(year, month, day),
        boost::posix_time::hours(hour) + boost::posix_time::minutes(minute) +
            boost::posix_time::seconds(second));
    const boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
    const int64_t seconds =
        (local_time - boost::posix_time::hours(8) - epoch).total_seconds();
    if (seconds < 0 || seconds > std::numeric_limits<int32_t>::max()) {
      throw std::runtime_error("Timestamp outside ROS Image header range");
    }
    return seconds * 1000000000LL + microsecond * 1000LL;
  } catch (const std::exception& error) {
    throw std::runtime_error("Invalid timestamp in " + path.string() + ": " + error.what());
  }
}

void collectImages(const fs::path& directory, const std::string& topic,
                   std::vector<ImageEntry>& entries) {
  if (!fs::is_directory(directory)) {
    throw std::runtime_error("Input directory does not exist: " + directory.string());
  }
  const auto initial_size = entries.size();
  for (const auto& file : fs::directory_iterator(directory)) {
    if (!file.is_regular_file()) continue;
    std::string extension = file.path().extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char ch) { return std::tolower(ch); });
    if (extension != ".bmp" && extension != ".png" && extension != ".jpg" &&
        extension != ".jpeg" && extension != ".tif" && extension != ".tiff") {
      continue;
    }
    entries.push_back({file.path(), topic, timestampFromFilename(file.path(), topic)});
  }
  if (entries.size() == initial_size) {
    throw std::runtime_error("No images found in " + directory.string());
  }
}

std::string imageEncoding(const cv::Mat& image) {
  // imread returns BGR/BGRA. Declare that encoding without changing pixels.
  namespace enc = sensor_msgs::image_encodings;
  switch (image.type()) {
    case CV_8UC1: return enc::MONO8;
    case CV_16UC1: return enc::MONO16;
    case CV_8UC3: return enc::BGR8;
    case CV_16UC3: return enc::BGR16;
    case CV_8UC4: return enc::BGRA8;
    case CV_16UC4: return enc::BGRA16;
    default:
      throw std::runtime_error("Unsupported OpenCV image type: " +
                               std::to_string(image.type()));
  }
}
}  // namespace

int main(int argc, char** argv) {
  namespace po = boost::program_options;
  std::string input_path;
  std::string output_bag;
  po::options_description options("Pack L/leftvi-* and R/rightvi-* images into a ROS2 bag");
  options.add_options()
      ("help,h", "Show usage")
      ("inpath,i", po::value<std::string>(&input_path)->default_value(
           "/home/jacob/Projects/CamCalibTool/data/N2/B/N2261014020/20"),
       "Input directory containing L and R")
      ("outbag,o", po::value<std::string>(&output_bag),
       "New output bag directory (required; must not already exist)");
  po::positional_options_description positional;
  positional.add("inpath", 1).add("outbag", 1);

  try {
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options)
                  .positional(positional).run(), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << options << '\n';
      return 0;
    }
    if (output_bag.empty()) {
      std::cerr << "--outbag is required.\n" << options << '\n';
      return 1;
    }
    if (fs::exists(output_bag)) {
      throw std::runtime_error("Output path already exists: " + output_bag);
    }

    std::vector<ImageEntry> entries;
    collectImages(fs::path(input_path) / "L", "leftvi", entries);
    const auto left_count = entries.size();
    collectImages(fs::path(input_path) / "R", "rightvi", entries);
    const auto right_count = entries.size() - left_count;
    std::sort(entries.begin(), entries.end(), [](const ImageEntry& a, const ImageEntry& b) {
      if (a.timestamp_ns != b.timestamp_ns) return a.timestamp_ns < b.timestamp_ns;
      if (a.topic != b.topic) return a.topic < b.topic;
      return a.path < b.path;
    });

    std::cout << "Writing leftvi: " << left_count << ", rightvi: " << right_count
              << " images; filename timezone UTC+8, final field in microseconds.\n";
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = output_bag;
    storage_options.storage_id = "sqlite3";
    rosbag2_cpp::Writer writer;
    writer.open(storage_options);
    size_t written = 0;
    for (const auto& entry : entries) {
      const cv::Mat image = cv::imread(entry.path.string(), cv::IMREAD_UNCHANGED);
      if (image.empty()) {
        throw std::runtime_error("Cannot read image: " + entry.path.string());
      }
      auto message = cv_bridge::CvImage(
          std_msgs::msg::Header(), imageEncoding(image), image).toImageMsg();
      const rclcpp::Time timestamp(entry.timestamp_ns, RCL_SYSTEM_TIME);
      message->header.stamp = timestamp;
      message->header.frame_id = entry.topic;
      writer.write<sensor_msgs::msg::Image>(*message, entry.topic, timestamp);
      if (++written % 100 == 0) {
        std::cout << "Written " << written << '/' << entries.size() << '\n';
      }
    }
    writer.close();
    std::cout << "Done: " << written << " images written to " << output_bag << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
