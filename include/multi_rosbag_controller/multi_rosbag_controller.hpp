#ifndef MULTI_ROSBAG_CONTROLLER
#define MULTI_ROSBAG_CONTROLLER

#include <vector>
#include <string>

#include <yaml-cpp/yaml.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

class MultiRosbagController
{
public:
  MultiRosbagController();
  MultiRosbagController(std::vector<std::string> rosbag_names);
  ~MultiRosbagController();

  void openRosbag(std::vector<std::string> rosbag_names);
  bool findTopic(std::string topic_name);
  void setTopic(std::string topic_name);
  void setPointsTopic(std::string topic_name);
  void addQueries(rosbag::View& view);
  void resetTopic();
  void setLidarTopics(std::string config);
  bool getIsFirst();
  int getTopicType();

private:
  // Variables
  int num_rosbag_;
  int topic_type_;
  bool is_first_;
  std::string points_corrected_topic_name;
  std::vector<rosbag::Bag> rosbags_;
  std::vector<std::string> topics_;
  std::vector<std::set<std::string>> topic_list_;
  bool is_topic_listed_;
};

#endif
