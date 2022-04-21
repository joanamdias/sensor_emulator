#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <actionlib/server/simple_action_server.h>
#include <sensor_emulator_msgs/SensorEmulatorAction.h>

typedef pcl::PointXYZRGBNormal PointT;

class SensorEmulatorAction
{
public:
  SensorEmulatorAction(std::string name) : as_(nh_, name, boost::bind(&SensorEmulatorAction::goalCB, this, _1), false),
                                           action_name_(name)
  {

    if (!nh_.getParam(name + "/ambient_pointcloud_topic", _ambient_pointcloud_topic))
    {
      ROS_ERROR("Ambient pointcloud topic not defined");
    }
    else
    {
      ROS_INFO("Ambient pointcloud topic: %s", _ambient_pointcloud_topic.c_str());
    }

    if (!nh_.getParam(name + "/frame_id", _frame_id))
    {
      nh_.param<std::string>(name+"/frame_id", _frame_id, "camera_frame");
    };
    nh_.getParam(name + "/folder_name", _folder_name);

    cluster_pub = nh_.advertise<sensor_msgs::PointCloud2>(_ambient_pointcloud_topic, 1000, true);

    as_.registerPreemptCallback(boost::bind(&SensorEmulatorAction::preemptCB, this));

    as_.start();
  }

  ~SensorEmulatorAction(void)
  {
  }

  void goalCB(const sensor_emulator_msgs::SensorEmulatorGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    }
    else if (pcl::io::loadPLYFile<PointT>(_folder_name + goal->file_name + ".ply", *cloud) == -1)
    {
      ROS_INFO("File not found");
      ROS_INFO("%s: Aborted", action_name_.c_str());
      success = false;
      as_.setAborted();
    }
    else
    {
      // Convert to ROS data type
      sensor_msgs::PointCloud2 output_cloud;
      pcl::toROSMsg(*cloud, output_cloud);
      output_cloud.header.frame_id = _frame_id;
      output_cloud.header.stamp = ros::Time::now();
      // Publish point cloud
      cluster_pub.publish(output_cloud);
    }

    feedback_.succeeded = success;
    as_.publishFeedback(feedback_);

    if (success)
    {
      result_.succeeded = feedback_.succeeded;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<sensor_emulator_msgs::SensorEmulatorAction> as_;
  std::string action_name_;
  std::string _folder_name;
  std::string _frame_id;
  std::string _ambient_pointcloud_topic;
  ros::Publisher cluster_pub;
  sensor_emulator_msgs::SensorEmulatorFeedback feedback_;
  sensor_emulator_msgs::SensorEmulatorResult result_;
};