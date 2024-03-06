#include <multi_rosbag_controller/multi_rosbag_controller.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

MultiRosbagController::MultiRosbagController()
{
  verbose_ = true;
}

MultiRosbagController::MultiRosbagController(std::vector<std::string> rosbag_names, const bool verbose)
{
  verbose_ = verbose;
  openRosbag(rosbag_names);
}

void MultiRosbagController::openRosbag(const std::vector<std::string>& rosbag_names)
{
  num_rosbag_ = rosbag_names.size();
  rosbags_.resize(num_rosbag_);
  topic_list_.resize(num_rosbag_);

  for (int i = 0; i < num_rosbag_; i++)
  {
    rosbags_[i].open(rosbag_names[i], rosbag::bagmode::Read);
    if (verbose_)
      std::cout << "Opened : " << rosbag_names[i] << std::endl;
  }

  if (verbose_)
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

bool MultiRosbagController::findTopic(const std::string& topic_name, const bool require_all_rosbags)
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
      if (require_all_rosbags)
      {
        return false;
      }
    }
    else
    {
      if (!require_all_rosbags)
      {
        return true;
      }
    }
  }

  return true;
}

bool MultiRosbagController::setTopic(const std::string& topic_name, const bool require_all_rosbags)
{
  if (!findTopic(topic_name, require_all_rosbags))
  {
    return false;
  }
  topics_.push_back(topic_name);
  return true;
}

int MultiRosbagController::selectTopicPriority(std::string prior_topic, std::string post_topic)
{
  if (setTopic(prior_topic))
  {
    return 0;
  }
  else if (setTopic(post_topic))
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

int MultiRosbagController::selectTopicPriority(std::vector<std::string> topics)
{
  for (int i = 0; i < topics.size(); i++)
  {
    if (setTopic(topics[i]))
    {
      return i;
    }
  }

  return -1;
}

void MultiRosbagController::addQueries(rosbag::View& view)
{
  for (int bag_id = 0; bag_id < num_rosbag_; bag_id++)
  {
    if (topics_.empty())
    {
      view.addQuery(rosbags_[bag_id]);
    }
    else
    {
      view.addQuery(rosbags_[bag_id], rosbag::TopicQuery(topics_));
    }
  }
}

void MultiRosbagController::resetTopic()
{
  topics_.clear();
}
