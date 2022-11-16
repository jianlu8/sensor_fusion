#include <ros/ros.h>
#include "include/sensors_fusion.h"

/** Main entry point. */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "sensors_fusion_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  apollo::perception::fusion::PerceptionYX yx(node, priv_nh);
  // ros::spin();
  ros::Rate rate(20);
  while (ros::ok())
  {
        ros::spinOnce();
        rate.sleep();
  }
  return 0;
}
