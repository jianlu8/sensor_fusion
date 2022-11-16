#include "include/sensors_fusion.h"

#include <iostream>
#define m_PI 0.0174533
namespace apollo {
namespace perception {
namespace fusion {

/**
 * @brief Construct a new PerceptionYX object
 *
 * @param node ros node handle
 * @param priv_nh ros private node handle
 */
PerceptionYX::PerceptionYX(ros::NodeHandle node, ros::NodeHandle priv_nh) {
  // Get parameters using private node handle
  // topics
  priv_nh.param("lidar_input_topic", lidar_input_topic,
                std::string("/minibus/lidar_moving_objects"));
  priv_nh.param("fusion_polygon_output_topic", fusion_polygon_output_topic,
                std::string("/minibus/polygon_objects"));
  priv_nh.param("gps_input_topic", gps_input_topic,
                std::string("/minibus/gpsposition"));

  // parameters in program
  priv_nh.param("is_use_fusion_time", is_use_fusion_time, false);

  /**
   * @brief init objects
   */
  lidar_objects_.reset(new base::Frame);
  ObstacleMultiSensorFusionParam param;
  if (!obstacle_fusion_.Init(param)) {
    ROS_WARN("284345350");
    return;
  }

  /**
   * @brief ROS Sub & Pub
   */

  lidar_objects =
      node.subscribe(lidar_input_topic, 1, &PerceptionYX::callbackLidarObjects,
                     this, ros::TransportHints().reliable().tcpNoDelay(true));
  gps_pose_sub =
      node.subscribe(gps_input_topic, 1, &PerceptionYX::callbackGpsPose, this,
                     ros::TransportHints().reliable().tcpNoDelay(true));

  /*订阅cicv视觉与gps数据*/
  fusion_polygon_publisher_ = node.advertise<visualization_msgs::MarkerArray>(
      "minibus/fusion_polygon_marker", 1);
  fusion_velocity_publisher_ = node.advertise<visualization_msgs::MarkerArray>(
      "minibus/fusion_velocity_marker", 1);
  /*发布融合输出结果*/
  fusion_objects_output_pub =
      node.advertise<util::moving_objects>(fusion_output_topic, 1);
  fusion_polygon_objects_output_pub =
      node.advertise<util::cicv_moving_objects>(fusion_polygon_output_topic, 1);
}

PerceptionYX::~PerceptionYX() {}

void PerceptionYX::processFusion(const base::FrameConstPtr& frame,
                                 geometry_msgs::PoseStamped gnss_data) {
  std::vector<base::ObjectPtr> fused_objects;
  obstacle_fusion_.Process(frame, &fused_objects);

  return;
}

void PerceptionYX::callbackTfData(const tf2_msgs::TFMessage::ConstPtr& msg) {
  /*订阅base_link到lidar_mid的tf值*/
  if (!base_link_lidar_mid_transform_flag) {
    try {
      base_link_lidar_mid_listener.waitForTransform(
          "/base_link", "/lidar_mid", ros::Time(0), ros::Duration(0.2));
      base_link_lidar_mid_listener.lookupTransform(
          "base_link", "lidar_mid", ros::Time(0),
          base_link_lidar_mid_transform);
      base_link_lidar_mid_transform_flag = true;
    } catch (tf::TransformException& ex) {
      ROS_WARN("no tf base_link to lidar_mid");
    }
  }
}

void PerceptionYX::callbackGpsPose(
    const common_msgs::GpsPosition::ConstPtr& msg) {
              std::cout << "start gps data loading";

  /*悦享欧拉角是东北天，融合程序使用的欧拉角是北西天坐标系，需要转换一下*/
  double yaw = msg->azimuth * 0.01;
  if (yaw < 0) {
    yaw = 360 + yaw;
  }
  yaw = yaw - 90;
  if (yaw > 180) {
    yaw = yaw - 360;
  }
  gps_pose.pose.position.y = msg->gaussX * 0.01;
  gps_pose.pose.position.x = msg->gaussY * 0.01;
  gps_pose.pose.position.z = msg->height;
  gps_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      msg->roll * 0.01 * m_PI, msg->pitch * 0.01 * m_PI, yaw * m_PI);
  gps_pose.header.stamp = msg->header.stamp;
  gps_pose.header.frame_id = "map";
  // GPS数据存储数量有限制
  if (m_gps_history_data.size() >= GPSDATASTORAGENUMBER)
    m_gps_history_data.erase(m_gps_history_data.begin());
  m_gps_history_data.push_back(gps_pose);
  // pose_publisher.publish(gps_pose);

  // tf::Transform map_to_base_link_transform;
  // map_to_base_link_transform.setOrigin(tf::Vector3(gps_pose.pose.position.x,
  //                                                  gps_pose.pose.position.y,
  //                                                  gps_pose.pose.position.z));
  // map_to_base_link_transform.setRotation(tf::createQuaternionFromRPY(
  //     msg->roll * 0.01 * m_PI, msg->pitch * 0.01 * m_PI, yaw * m_PI));
  // ros::Time send_time = msg->header.stamp;
  // map_to_base_link_br.sendTransform(tf::StampedTransform(
  //     map_to_base_link_transform, send_time, "/map", "/base_link"));
}

void PerceptionYX::callbackLidarObjects(
    const common_msgs::cicv_moving_objects::ConstPtr& msg) {
        std::cout << "start lidar fusion";

  lidar_header = msg->header;
  double Received_Data_Time = ros::Time::now().toSec();
  if (m_gps_history_data.size() == 0) {
    ROS_INFO(
        "received lidar objects,but not rceived gps data,now waiting gps data");
    return;
  }
  geometry_msgs::PoseStamped correspondingGpsData =
      AccordingTimeStampMatchGPSdata(msg->header.stamp.toSec());
  common_msgs::cicv_moving_objects m_LidarObjects;
  m_LidarObjects = *msg;
  ObjectsDataFilter(m_LidarObjects);  //防止单帧雷达目标数据数量过大
  lidar_objects_->objects.clear();
  if (m_LidarObjects.objects.size() > 0) {
    for (int i = 0; i < m_LidarObjects.objects.size(); i++) {
      std::shared_ptr<base::Object> m_object;
      m_object.reset(new base::Object);
      m_object = ObstacleMsgToFusionMsg(m_LidarObjects.objects.at(i),
                                        correspondingGpsData);
      if (is_use_fusion_time) {
        m_object->timestamp = Received_Data_Time;
      } else {
        m_object->timestamp = msg->header.stamp.toSec();
      }

      lidar_objects_->objects.push_back(m_object);
    }
  }
  lidar_objects_->sensor_info.type = base::SensorType::VELODYNE_128;
  // lidar_objects_->sensor_id = GetSensorType(lidar_objects_->sensor_type);
  if (is_use_fusion_time) {
    lidar_objects_->timestamp = Received_Data_Time;
  } else {
    lidar_objects_->timestamp = msg->header.stamp.toSec();
  }
  lidar_objects_->sensor2world_pose = Eigen::Matrix4d::Identity();
  processFusion(lidar_objects_, correspondingGpsData);
}

geometry_msgs::PoseStamped PerceptionYX::AccordingTimeStampMatchGPSdata(
    double timestamp) {
  for (int i = 0; i < m_gps_history_data.size(); i++) {
    if (m_gps_history_data.at(i).header.stamp.toSec() - timestamp >= 0)
      return m_gps_history_data.at(i);
  }
  /*如果所有数据都比形参时间戳小，则返回最新的值*/
  return m_gps_history_data.at(m_gps_history_data.size() - 1);
}

void PerceptionYX::ObjectsDataFilter(
    common_msgs::cicv_moving_objects& input_data) {
  for (int i = 0; i < input_data.objects.size(); i++) {
    float dis = sqrt(pow(input_data.objects.at(i).pose.position.x, 2) +
                     pow(input_data.objects.at(i).pose.position.y, 2));
    input_data.objects.at(i).variance = dis;
  }
  sort(input_data.objects.begin(), input_data.objects.end(), myfunction);
  if (input_data.objects.size() > MAXSINGLEFRAMEDATANUMBER) {
    input_data.objects.erase(
        input_data.objects.begin() + MAXSINGLEFRAMEDATANUMBER,
        input_data.objects.end());
  }
  input_data.num = input_data.objects.size();
}

std::shared_ptr<base::Object> PerceptionYX::ObstacleMsgToFusionMsg(
    common_msgs::cicv_moving_object object,
    geometry_msgs::PoseStamped gnss_data) {
  std::shared_ptr<base::Object> m_object;
  m_object.reset(new base::Object);
  m_object->id = object.id;
  m_object->theta = tf::getYaw(object.pose.orientation);
  m_object->size << object.dimensions.x,
                    object.dimensions.y,
                    object.dimensions.z;
  // m_object->length = object.dimensions.x;
  // m_object->height = object.dimensions.z;
  // m_object->width = object.dimensions.y;
  /*为求4个顶点*/
  // double cos_heading_ = cos(m_object->theta);
  // double sin_heading_ = sin(m_object->theta);
  // const double dx1 = cos_heading_ * m_object->length;
  // const double dy1 = sin_heading_ * m_object->length;
  // const double dx2 = sin_heading_ * m_object->width;
  // const double dy2 = -cos_heading_ * m_object->width;

  if (lidar_header.frame_id != "/base_link") {
    /*base_link坐标系下的目标值*/
    object.pose.position.x = object.pose.position.x +
                             base_link_lidar_mid_transform.getOrigin().getX();
    object.pose.position.y = object.pose.position.y +
                             base_link_lidar_mid_transform.getOrigin().getY();
    object.pose.position.z = object.pose.position.z +
                             base_link_lidar_mid_transform.getOrigin().getZ();
  }

  // for rectangle object
  //   geometry_msgs::Point mm;
  //   mm.x = object.pose.position.x, mm.y = object.pose.position.y,mm.z =
  //   object.pose.position.z; geometry_msgs::Point output =
  //   calcAbsoluteCoordinate(mm,gnss_data);
  //   m_object->center<<output.x,output.y,output.z;
  //   m_object->anchor_point<<output.x,output.y,output.z;

  //   mm.x = object.pose.position.x+ dx1 + dx2, mm.y = object.pose.position.y +
  //   dy1 + dy2,mm.z = object.pose.position.z; output =
  //   calcAbsoluteCoordinate(mm,gnss_data);
  //   apollo_perception_standalone::pcl_util::PointD mm_l;
  //   mm_l.x = output.x ,mm_l.y = output.y,mm_l.z = output.z;
  //   m_object->polygon.push_back(mm_l);

  //   mm.x = object.pose.position.x+ dx1 - dx2, mm.y = object.pose.position.y +
  //   dy1 - dy2,mm.z = object.pose.position.z; output =
  //   calcAbsoluteCoordinate(mm,gnss_data); mm_l.x = output.x ,mm_l.y =
  //   output.y,mm_l.z = output.z; m_object->polygon.push_back(mm_l);

  //   mm.x = object.pose.position.x- dx1 - dx2, mm.y = object.pose.position.y -
  //   dy1 - dy2,mm.z = object.pose.position.z; output =
  //   calcAbsoluteCoordinate(mm,gnss_data); mm_l.x = output.x ,mm_l.y =
  //   output.y,mm_l.z = output.z; m_object->polygon.push_back(mm_l);

  //   mm.x = object.pose.position.x- dx1 + dx2, mm.y = object.pose.position.y -
  //   dy1 + dy2,mm.z = object.pose.position.z; output =
  //   calcAbsoluteCoordinate(mm,gnss_data); mm_l.x = output.x ,mm_l.y =
  //   output.y,mm_l.z = output.z; m_object->polygon.push_back(mm_l);

  // @TODO: for polygon object
  geometry_msgs::Point mm, mm_i;
  mm.x = object.pose.position.x, mm.y = object.pose.position.y,
  mm.z = object.pose.position.z;
  geometry_msgs::Point output = calcAbsoluteCoordinate(mm, gnss_data);
  m_object->center << output.x, output.y, output.z;
  m_object->anchor_point << output.x, output.y, output.z;
  for (int i = 0; i < object.convex_hull.polygon.points.size(); ++i) {
    base::PointD mm_o;
    mm_i.x = object.convex_hull.polygon.points[i].x,
    mm_i.y = object.convex_hull.polygon.points[i].y,
    mm_i.z = object.convex_hull.polygon.points[i].z;
    output = calcAbsoluteCoordinate(mm_i, gnss_data);
    mm_o.x = output.x, mm_o.y = output.y, mm_o.z = output.z;
    // mm_o.x = object.convex_hull.polygon.points[i].x, mm_o.y =
    // object.convex_hull.polygon.points[i].y, mm_o.z =
    // object.convex_hull.polygon.points[i].z;
    //   std::cout << "object.convex_hull.polygon.points[i].x = " << output.x <<
    //   std::endl;
    m_object->polygon.push_back(mm_o);
  }
  //
  //   if (m_object->length > 1 && m_object->length < 6 && m_object->width < 6
  //   &&
  //       m_object->width > 1) {
  //     m_object->type = apollo_perception_standalone::ObjectType::FORKLIFT;
  //   } else {
  //     m_object->type = apollo_perception_standalone::ObjectType::UNKNOWN;
  //   }
  if (object.label == "pedestrain") {
    m_object->type = base::ObjectType::PEDESTRIAN;
  } else if (object.label == "vehicle") {
    m_object->type = base::ObjectType::VEHICLE;
  } else {
    m_object->type = base::ObjectType::UNKNOWN;
  }
  // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!! label :" <<
  // GetObjectName(m_object->type) << std::endl;
  /*赋值车速信息,,认为目标相对车辆没有横向速度*/
  // if (is_use_lidar_moving_objects_speed)
  // m_object->velocity << object.speed * cos(m_VehicleAttitudeInfo.origin_yaw),
  //     object.speed * sin(m_VehicleAttitudeInfo.origin_yaw), 0;
  m_object->velocity << object.speed, object.speed, 0;
  // else
  // m_object->velocity << 0, 0, 0;
  return m_object;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo