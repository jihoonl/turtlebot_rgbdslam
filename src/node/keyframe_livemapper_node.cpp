/**
 * See LICNESE file
 */

#include "turtlebot_rgbdslam/keyframe_livemapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "KeyframeLiveMapper");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  turtlebot_rgbdslam::KeyframeLiveMapper live_mapper(nh, nh_private);
  ROS_INFO("Initialized");
  ros::spin();
  return 0;
}
