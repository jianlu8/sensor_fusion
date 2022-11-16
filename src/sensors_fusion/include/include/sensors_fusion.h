#ifndef _PERCEPTION_YX_H_
#define _PERCEPTION_YX_H_
#include <can_msgs/Frame.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <pthread.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/StdVector>
#include <map>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include "common_msgs/GpsPosition.h"
#include "common_msgs/Guidance.h"
#include "common_msgs/LocalPose.h"
#include "common_msgs/MmWave_Objects.h"
#include "common_msgs/TrafficLightResultArray.h"
#include "common_msgs/VehiclePose.h"
#include "common_msgs/cicv_moving_object.h"
#include "common_msgs/cicv_moving_objects.h"
#include "common_msgs/mmwave_object.h"
#include "geometry_msgs/Point32.h"
#include "include/base/frame.h"
#include "include/obstacle_multi_sensor_fusion.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "util/Ultrasonic.h"
#include "util/VehicleInfo.h"
#include "util/cicv_moving_object.h"
#include "util/cicv_moving_objects.h"
#include "util/moving_objects.h"
#define M_PI_ 59.29578
#define PI_ 3.1415926
#define MAXLIDARTORADARDIS 5
#define MAXSINGLEFRAMEDATANUMBER 50
#define GPSDATASTORAGENUMBER 20
#define HISTORYVEHICLEATTITUDEINFONUMBER 5
#define MAXSHOWFUSIONOUTPUTNUMBER 100
#define VELOCITYLENGTH 5
namespace apollo {
namespace perception {
namespace fusion {

class PerceptionYX {
 public:
  /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
  PerceptionYX(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~PerceptionYX();

  /** callback to process data input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
 private:
  void processFusion(const base::FrameConstPtr& frame,
                     geometry_msgs::PoseStamped gnss_data);
  void callbackTfData(const tf2_msgs::TFMessage::ConstPtr& msg);
  void callbackGpsPose(const common_msgs::GpsPosition::ConstPtr& msg);
  void callbackLidarObjects(
      const common_msgs::cicv_moving_objects::ConstPtr& msg);

  // tool functions
  std::shared_ptr<base::Object> ObstacleMsgToFusionMsg(
      common_msgs::cicv_moving_object object,
      geometry_msgs::PoseStamped gnss_data);
  geometry_msgs::PoseStamped AccordingTimeStampMatchGPSdata(double timestamp);
  void ObjectsDataFilter(common_msgs::cicv_moving_objects& input_data);

  static bool myfunction(common_msgs::cicv_moving_object i,
                  common_msgs::cicv_moving_object j) {
    return (i.variance < j.variance);
  }
  geometry_msgs::Point calcAbsoluteCoordinate(
      geometry_msgs::Point point_msg, geometry_msgs::PoseStamped current_pose) {
    geometry_msgs::Pose current_pose_msg;
    current_pose_msg.orientation = current_pose.pose.orientation;
    current_pose_msg.position = current_pose.pose.position;
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose_msg, inverse);

    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = inverse * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);
    return tf_point_msg;
  }
  geometry_msgs::Point calcAbsoluteCoordinate(
      geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose) {
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);
    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = inverse * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);
    return tf_point_msg;
  }
  geometry_msgs::Point calcRelativeCoordinate(
      geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose) {
    tf::Transform inverse;
    tf::poseMsgToTF(current_pose, inverse);
    tf::Transform transform = inverse.inverse();
    tf::Point p;
    pointMsgToTF(point_msg, p);
    tf::Point tf_p = transform * p;
    geometry_msgs::Point tf_point_msg;
    pointTFToMsg(tf_p, tf_point_msg);
    return tf_point_msg;
  }

  tf::TransformListener
      base_link_lidar_mid_listener;  // base_link到lidar_mid的tf监听器
  tf::StampedTransform base_link_lidar_mid_transform;
  bool base_link_lidar_mid_transform_flag = false;

  common_msgs::cicv_moving_object m_object;
  common_msgs::cicv_moving_objects camera_objects;

  // Parameters for projection
  pthread_mutex_t proj_lock_;

  std_msgs::Header lidar_header;
  std::string lidar_input_topic;
  std::string fusion_output_topic;
  std::string fusion_polygon_output_topic;
  std::string gps_input_topic;

  ros::Subscriber lidar_objects;
  ros::Subscriber gps_pose_sub;

  ros::Publisher fusion_polygon_publisher_;
  ros::Publisher fusion_velocity_publisher_;
  ros::Publisher fusion_objects_output_pub;
  ros::Publisher fusion_polygon_objects_output_pub;

  // parameters in launch
  bool is_use_fusion_time;

  // global data
  geometry_msgs::PoseStamped gps_pose;
  std::vector<geometry_msgs::PoseStamped> m_gps_history_data;
  base::FramePtr lidar_objects_;
  ObstacleMultiSensorFusion obstacle_fusion_;

  // tracked objects
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo

#endif
