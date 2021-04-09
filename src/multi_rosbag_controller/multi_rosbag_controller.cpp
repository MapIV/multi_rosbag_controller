#include <multi_rosbag_controller/multi_rosbag_controller.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

MultiRosbagController::MultiRosbagController()
{
  points_corrected_topic_name = std::string("/map4_points_corrected");
}

MultiRosbagController::MultiRosbagController(std::vector<std::string> rosbag_names)
{
  points_corrected_topic_name = std::string("/map4_points_corrected");
  openRosbag(rosbag_names);
}

void MultiRosbagController::openRosbag(std::vector<std::string> rosbag_names)
{
  num_rosbag_ = rosbag_names.size();
  rosbags_.resize(num_rosbag_);
  topic_list_.resize(num_rosbag_);

  for (int i = 0; i < num_rosbag_; i++)
  {
    rosbags_[i].open(rosbag_names[i], rosbag::bagmode::Read);
    std::cout << "Opened : " << rosbag_names[i] << std::endl;
  }

  std::cout << "ROSBAG count: " << num_rosbag_ << std::endl;

  is_topic_listed_ = false;
}

MultiRosbagController::~MultiRosbagController()
{
  for (int i = 0; i < num_rosbag_; i++)
  {
    rosbags_[i].close();
  }
}

bool MultiRosbagController::findTopic(std::string topic_name)
{
  // Collect all topic names in this rosbag
  if (!is_topic_listed_)
  {
    for (int bag_id = 0; bag_id < num_rosbag_; bag_id++)
    {
      rosbag::View search_topic(rosbags_[bag_id]);
      std::vector<const rosbag::ConnectionInfo*> connection_infos = search_topic.getConnections();

      BOOST_FOREACH (const rosbag::ConnectionInfo* info, connection_infos)
      {
        // If the current topic is not already in topic_list_[bag_id], add
        if (topic_list_[bag_id].find(info->topic) == topic_list_[bag_id].end())
        {
          topic_list_[bag_id].insert(info->topic);
        }
      }
    }
    is_topic_listed_ = true;
  }

  // Search the query topic
  for (int bag_id = 0; bag_id < num_rosbag_; bag_id++)
  {
    if (topic_list_[bag_id].find(topic_name) == topic_list_[bag_id].end())
    {
      return false;
    }
  }

  return true;
}

void MultiRosbagController::setTopic(std::string topic_name)
{
  if (!findTopic(topic_name))
  {
    std::cerr << "\033[31mError: Cannot find topic: " << topic_name << "\033[m" << std::endl;
    exit(4);
  }
  topics_.push_back(topic_name);
}

void MultiRosbagController::setPointsTopic(std::string topic_name)
{
  // Set topic_name if it exists, otherwise set /points_raw
  if (findTopic(topic_name))
  {
    topics_.push_back(topic_name);
    is_first_ = true;
  }
  else if (findTopic(points_corrected_topic_name))
  {
    topics_.push_back(points_corrected_topic_name);
    is_first_ = false;
  }
  else
  {
    std::cerr << "\033[31mError: Cannot find topic: " << topic_name << " and " << points_corrected_topic_name
              << "\033[m" << std::endl;
    exit(4);
  }
}

void MultiRosbagController::addQueries(rosbag::View& view)
{
  for (int bag_id = 0; bag_id < num_rosbag_; bag_id++)
  {
    view.addQuery(rosbags_[bag_id], rosbag::TopicQuery(topics_));
  }
}

void MultiRosbagController::resetTopic()
{
  topics_.clear();
}

void MultiRosbagController::setLidarTopics(std::string config)
{
  std::string points_topic;
  std::string extra_points_topic;
  bool use_extra_lidar;
  std::string difop_topic, extra_difop_topic;

  try
  {
    YAML::Node conf = YAML::LoadFile(config);

    // LiDAR config
    points_topic = conf["lidar"]["points_topic"].as<std::string>();
    setPointsTopic(points_topic);
    if (is_first_)
    {
      topic_type_ = conf["lidar"]["topic_type"].as<int>();
    }
    else
    {
      topic_type_ = 1;  // PointCloud2 map4_points_corrected
    }

    if (topic_type_ == 2)
    {
      difop_topic = conf["lidar"]["difop_topic"].as<std::string>();
      setTopic(difop_topic);
    }

    use_extra_lidar = conf["lidar"]["use_extra_lidar"].as<bool>();
    if (use_extra_lidar)
    {
      extra_points_topic = conf["lidar"]["extra_lidar"]["points_topic"].as<std::string>();
      setTopic(extra_points_topic);

      if (topic_type_ == 2)
      {
        extra_difop_topic = conf["lidar"]["extra_lidar"]["difop_topic"].as<std::string>();
        setTopic(extra_difop_topic);
      }
    }
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[31mError: " << e.what() << "\033[m" << std::endl;
    exit(3);
  }
}

bool MultiRosbagController::getIsFirst()
{
  return is_first_;
}

int MultiRosbagController::getTopicType()
{
  return topic_type_;
}
