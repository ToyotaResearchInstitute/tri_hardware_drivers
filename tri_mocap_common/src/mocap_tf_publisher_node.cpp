#include <ros/ros.h>
#include <iostream>
#include <string>
#include <tri_mocap_common/MocapState.h>
#include <tf2_msgs/TFMessage.h>

ros::Publisher g_transform_pub;
bool g_override_timestamps;

void MocapMsgCB(tri_mocap_common::MocapState msg)
{
  tf2_msgs::TFMessage transforms_msg;
  const std::string& mocap_name = msg.tracker_name;
  const ros::Time transforms_time = (g_override_timestamps) ? ros::Time::now()
                                                            : msg.header.stamp;
  const std::string& mocap_frame = msg.header.frame_id;
  for (size_t idx_o = 0; idx_o < msg.tracked_objects.size(); idx_o++)
  {
    const tri_mocap_common::MocapObject& current_object
            = msg.tracked_objects[idx_o];
    for (size_t idx_s = 0; idx_s < current_object.segments.size(); idx_s++)
    {
      const tri_mocap_common::MocapSegment& current_segment
              = current_object.segments[idx_s];
      if (!current_segment.occluded)
      {
        /* Get the full segment name */
        const std::string full_segment_name = mocap_name
                                              + "_" + current_object.name
                                              + "_" + current_segment.name;
        geometry_msgs::TransformStamped segment_transform;
        segment_transform.header.stamp = transforms_time;
        segment_transform.header.frame_id = mocap_frame;
        segment_transform.child_frame_id = full_segment_name;
        segment_transform.transform = current_segment.transform;
        transforms_msg.transforms.push_back(segment_transform);
      }
    }
  }
  if (transforms_msg.transforms.size() > 0)
  {
    g_transform_pub.publish(transforms_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mocap_tf_publisher");
  ROS_INFO("Starting mocap_tf_publisher...");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  const std::string tracker_topic = nhp.param(std::string("tracker_topic"),
                                              std::string("mocap_tracking"));
  g_override_timestamps = nhp.param(std::string("override_timestamps"), false);
  const std::string tf_topic = nhp.param(std::string("tf_topic"),
                                         std::string("/tf"));
  const double spin_rate = nhp.param(std::string("spin_rate"), 200.0);
  if (g_override_timestamps)
  {
    ROS_WARN("Parameter override_timestamps is set to TRUE - TF publisher"
             " will overwrite the timestamps from the mocap system with"
             " the current time. USE CAREFULLY!");
  }
  else
  {
    ROS_INFO("Parameter override_timestamps is set to FALSE - TF publisher"
             " will use the timestamps from the mocap system.");
  }
  // Create the TF broadcaster
  g_transform_pub = nh.advertise<tf2_msgs::TFMessage>(tf_topic, 1, false);
  // Create the mocap data subscriber
  ros::Subscriber mocap_sub = nh.subscribe(tracker_topic, 1, MocapMsgCB);
  // Start streaming data
  ROS_INFO("Broadcasting transforms...");
  ros::Rate ros_spin_rate(spin_rate);
  while (ros::ok())
  {
    // Handle ROS stuff
    ros::spinOnce();
    // Sleep briefly
    ros_spin_rate.sleep();
  }
  return 0;
}
