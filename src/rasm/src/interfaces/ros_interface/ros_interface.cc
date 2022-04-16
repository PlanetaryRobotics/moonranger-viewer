#include "ros_interface.h"
#include "rasm/ROBOT_POSE_MSG.h"
#include "rasm/RASM_GOAL_MSG.h"
#include "rasm/RASM_MAP_MSG.h"
#include "rasm/RASM_PATH_COMMAND_MSG.h"
// #include "rasm/RASM_STOP_COMMAND_MSG.h"
// #include "rasm/RASM_GOAL_REACHED_MSG.h"
// #include "rasm/RASM_GOAL_ACCEPTED_MSG.h"
// #include "rasm/RASM_DELETE_GOAL_MSG.h"
#include "rasm/RASM_DRIVE_ARC_MSG.h"
#include <assert.h>
#include <cstdio>
#include <cstdlib>
#include <string.h>
#include <util.h>
#include <sys/time.h>
#include <comms_interface.h>
#include <cmath>

#include <vector>
#include <tuple>
#include <iostream>
#include <fstream>

#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt32.h"

using std::vector;
using std::tuple;

std::string   poseFile    = "poseTracking.txt";
std::ofstream fstream;
vector<tuple<int, double, geometry_msgs::Quaternion>> odomString;
vector<tuple<int, double, rasm::ROBOT_POSE_MSG>> conversionString;

void ROSInterface::robot_pose_handler(const nav_msgs::Odometry::ConstPtr& odom) {
  this->m_previous_pose = this->m_latest_pose;

  /*!
   * 12/5/2020:
   * If uncommented, the proceeding block enables the user to print the rover's
   * pose to a text file for the purpose of debugging. More specifically, the
   * block converts the rover's Quaternion-based yaw into an Euler-based yaw for
   * human-interpretation's sake. At present, neither pitch nor roll are converted
   * and tracked.
   */
//-------------------------------BEGIN BLOCK------------------------------------------------------
  ///*!
  // * Track the most recent odometry reading from Gazebo:
  // */
  //odomString.push_back(std::make_tuple(__LINE__, now(), odom->pose.pose.orientation));


  ///*!
  // * Track the most recent odometry conversion to be used
  // * by the navigator:
  // */
  //conversionString.push_back(std::make_tuple(__LINE__, now(), call_data));

  ///*!
  // * Now format and print this data to the poseFile:
  // */
  //if (this->m_zero_first_position) fstream.open(poseFile);
  //else fstream.open(poseFile, std::ios_base::app);
  //fstream << "Recorded odometry:\n" << std::get<0>(odomString[odomString.size() - 1]) << " "
  //  << std::get<1>(odomString[odomString.size() - 1]) << " "
  //  << std::get<2>(odomString[odomString.size() - 1]) <<
  //  "\nConverted yaw:\n" << std::get<0>(conversionString[conversionString.size() - 1]) <<
  // " " << std::get<1>(conversionString[conversionString.size() - 1]) << " " <<
  // std::get<2>(conversionString[conversionString.size() - 1]).yaw_rad <<
  //"\nYaw in degrees: " << std::get<2>(conversionString[conversionString.size() - 1]).yaw_rad * 180/M_PI << "\n\n";
  //fstream.close();
//-------------------------------END BLOCK--------------------------------------------------------

  rasm::ROBOT_POSE_MSG call_data = Odom_to_RasmPose_converter(odom);
  //printf("call_data value: %d, %d, %d\n", call_data.x_meters, call_data.y_meters, call_data.z_meters);

  static double first_x = 0;
  static double first_y = 0;
  static double first_z = 0;
  static bool first_pos_init = false;
  if (!first_pos_init && this->m_zero_first_position) {
    first_x =
        call_data.x_meters;
    first_y = call_data.y_meters;
    first_z = call_data.z_meters;
    first_pos_init = true;
  }

  /*
   * Now fill in latest pose
   */
  RASM::point3d vehicle_world_position_meters;
  vehicle_world_position_meters.set(
      /* x = */ METERS_TO_RASM(call_data.x_meters - first_x),
      /* y = */ METERS_TO_RASM(call_data.y_meters - first_y),
      /* z = */ METERS_TO_RASM(call_data.z_meters - first_z));

  float roll = call_data.roll_rad;
  float pitch = call_data.pitch_rad;
  float yaw = call_data.yaw_rad;
  //printf("roll, pitch, and yaw: %d, %d, %d\n", roll, pitch, yaw);

  double t = call_data.timestamp;

  this->set_pose(/*position=*/vehicle_world_position_meters,
                 /*roll=*/roll,
                 /*pitch=*/pitch,
                 /*yaw=*/yaw,
                 /*timestamp=*/t);
  this->eulerPosePublisher.publish(call_data);
}

void ROSInterface::goal_handler(rasm::RASM_GOAL_MSG call_data) {
  RASM::goal g;
  g.m_position.set(METERS_TO_RASM(-call_data.origin_y_meters),
                   METERS_TO_RASM(call_data.origin_x_meters));

  g.m_semimajor_axis = call_data.length_meters / 2.0;
  g.m_semiminor_axis = call_data.width_meters / 2.0;
  g.m_orientation_radians = call_data.orientation_radians;

  this->set_goal(g);

  printf("[%s:%d %f] set goal (%0.3f, %0.3f) with semi-major/minor axes "
         "%0.3f/%0.3f, orientation %f     deg\n",
         __FILE__, __LINE__, now(), RASM_TO_METERS(g.m_position.X()),
         RASM_TO_METERS(g.m_position.Y()), g.m_semimajor_axis,
         g.m_semiminor_axis, g.m_orientation_radians * 180.0 / M_PI);
}

// void ROSInterface::goal_reached_handler(
//     rasm::RASM_GOAL_REACHED_MSG call_data) {

//   this->m_latest_goal_reached.m_position.set(
//       METERS_TO_RASM(call_data.origin_x_meters),
//       METERS_TO_RASM(call_data.origin_y_meters));
//   this->m_latest_goal_reached.m_semimajor_axis = call_data.length_meters / 2.0;
//   this->m_latest_goal_reached.m_semiminor_axis = call_data.width_meters / 2.0;
//   this->m_latest_goal_reached.m_orientation_radians =
//       call_data.orientation_radians;

//   printf("[%s:%d %f] received goal-reached notification (%0.3f, %0.3f) with semi-major/minor axes %0    .3f/%0.3f, orientation %f deg, time %f\n",
//   __FILE__, __LINE__, now(),
//   call_data.origin_x_meters,
//   call_data.origin_y_meters,
//   call_data.length_meters,
//   call_data.width_meters,
//   call_data.orientation_radians * 180.0/M_PI,
//   call_data.timestamp);
// }

// void ROSInterface::goal_accepted_handler(
//     rasm::RASM_GOAL_ACCEPTED_MSG call_data) {

//   this->m_latest_goal_accepted.m_position.set(
//       METERS_TO_RASM(call_data.origin_x_meters),
//       METERS_TO_RASM(call_data.origin_y_meters));
//   this->m_latest_goal_accepted.m_semimajor_axis = call_data.length_meters / 2.0;
//   this->m_latest_goal_accepted.m_semiminor_axis = call_data.width_meters / 2.0;
//   this->m_latest_goal_accepted.m_orientation_radians =
//       call_data.orientation_radians;

//   printf("[%s:%d %f] received goal-accepted notification (%0.3f, %0.3f) with semi-major/minor axes %    0.3f/%0.3f, orientation %f deg, time %f\n",
//    __FILE__, __LINE__, now(),
//   call_data.origin_x_meters,
//   call_data.origin_y_meters,
//   call_data.length_meters,
//   call_data.width_meters,
//   call_data.orientation_radians * 180.0/M_PI,
//   call_data.timestamp);
// }

// void ROSInterface::delete_goal_handler(rasm::RASM_DELETE_GOAL_MSG call_data) {
//   this->delete_goal();

//   printf("[%s:%d %f] Deleted goal\n", __FILE__, __LINE__, now());
// }

void ROSInterface::map_handler(rasm::RASM_MAP_MSG call_data) {
  printf("[%s:%d %f] Got map type %d number %d with %d vertices @ t %f\n",
         __FILE__, __LINE__, now(), call_data.map_type,
         call_data.sequence_number, call_data.num_vertices,
         call_data.timestamp);
  fflush(stdout);

  if (call_data.map_type < RASM::MapType::NUM_MAP_TYPES) {
    RASM::mesh map;

    map.m_num_vertices = call_data.num_vertices;
    map.m_vertices = new RASM::point3d[map.m_num_vertices];
    assert(NULL != map.m_vertices);
    for (unsigned int i = 0; i < map.m_num_vertices; i++) {
      map.m_vertices[i].coord3d[0] =
          METERS_TO_RASM(call_data.vertices_meters[i].x);
      map.m_vertices[i].coord3d[1] =
          METERS_TO_RASM(call_data.vertices_meters[i].y);
      map.m_vertices[i].coord3d[2] =
          METERS_TO_RASM(call_data.vertices_meters[i].z);
    }

    map.m_num_faces = call_data.num_triangles;
    map.m_faces = new RASM::triangle[map.m_num_faces];
    assert(NULL != map.m_faces);
    for (unsigned int i = 0; i < map.m_num_faces; i++) {
      for (unsigned int j = 0; j < 3; j++)
        { map.m_faces[i].points[j] = call_data.triangles[i].indices[j]; }
    }

    this->set_map(call_data.map_type, call_data.sequence_number, map,
                  call_data.timestamp);

    delete[] map.m_vertices;
    delete[] map.m_faces;
  } else {
    printf("[%s:%d %f] WARNING: unknown map type %d\n", __FILE__, __LINE__,
           now(), call_data.map_type);
    return;
  }
}

void ROSInterface::path_cmd_handler(rasm::RASM_PATH_COMMAND_MSG call_data) {
  this->set_path_cmd(call_data.path_index, call_data.forward, call_data.timestamp);
  return;
}

// void ROSInterface::stop_cmd_handler(rasm::RASM_STOP_COMMAND_MSG call_data) {
//   this->set_stop_cmd(call_data.timestamp);
//   return;
// }

ROSInterface::ROSInterface(const char *module_name, std::map<std::string, std::string>& config, bool zero_first_position)
    : CommsInterface(), m_zero_first_position(zero_first_position) {
  assert(NULL != module_name);

  //Handle config file's parameters:
  if (config.find("pose_estimate_topic") != config.end()) {
    this->m_pose_estimate_topic = config["pose_estimate_topic"];
    //std::cout << "Using pose topic named: " << m_pose_estimate_topic << std::endl;
  }

  //Publishers:
  this->mapPublisher         = n.advertise<rasm::RASM_MAP_MSG>("/mapper/current_map", 1);
  this->posePublisher        = n.advertise<nav_msgs::Odometry>(m_pose_estimate_topic, 1);
  this->eulerPosePublisher   = n.advertise<rasm::ROBOT_POSE_MSG>("/navigator/eulerPose", 1);
  // this->atGoalPublisher      = n.advertise<rasm::RASM_GOAL_REACHED_MSG>("/rover_executive/goal_reached", 1);
  // this->acceptGoalPublisher  = n.advertise<rasm::RASM_GOAL_ACCEPTED_MSG>("/navigator/goal_accepted", 1);
  // this->deleteGoalPublisher  = n.advertise<rasm::RASM_DELETE_GOAL_MSG>("/rover_executive/delete_goal", 1);
  this->goalPublisher        = n.advertise<rasm::RASM_GOAL_MSG>("/rover_executive/goal_command", 1);
  // this->stopCommandPublisher = n.advertise<rasm::RASM_STOP_COMMAND_MSG>("/rover_executive/stop_command", 1);
  this->pathPublisher        = n.advertise<rasm::RASM_PATH_COMMAND_MSG>("/navigator/drive_arc_index", 1);
  this->driveArcPublisher    = n.advertise<rasm::RASM_DRIVE_ARC_MSG>("/navigator/drive_arc", 1);
  // this->arcCostPublisher     = n.advertise<rasm::ARC_COSTS>("/navigator/drive_arc_costs", 1);

  //Subscribers:
  this->poseSubscriber       = n.subscribe(m_pose_estimate_topic, 1, &ROSInterface::robot_pose_handler, this);
}

ROSInterface::~ROSInterface() {}

bool ROSInterface::pause(unsigned int time_ms) const{
  //allow the subscribers to receive new messages
  ros::spinOnce();
  //if ros is not running quit
  if(!ros::ok()) {
    exit(0);
  }
  return 1;
}

bool ROSInterface::publish_map(unsigned int map_type, unsigned int sequence_number,
           RASM::mesh &map, RASM::point3d origin, double timestamp) {
  rasm::RASM_MAP_MSG msg;
  memset(&msg, 0, sizeof(rasm::RASM_MAP_MSG));

  msg.map_type        = map_type;
  msg.sequence_number = sequence_number;
  msg.timestamp = timestamp;

  msg.num_vertices = map.m_num_vertices;
  for(unsigned int i=0; i < msg.num_vertices; i++)
    {
      geometry_msgs::Point toPushback;
      toPushback.x = RASM_TO_METERS(map.m_vertices[i].X() + origin.X());
      toPushback.y = RASM_TO_METERS(map.m_vertices[i].Y() + origin.Y());
      toPushback.z = RASM_TO_METERS(map.m_vertices[i].Z() + origin.Z());
      msg.vertices_meters.push_back(toPushback);
    }
  
  msg.num_triangles = map.m_num_faces;
  for(unsigned int i=0; i < msg.num_triangles; i++)
    {
      rasm::triangle toInsert;
      toInsert.indices.push_back(map.m_faces[i].points[0]);
      toInsert.indices.push_back(map.m_faces[i].points[1]);
      toInsert.indices.push_back(map.m_faces[i].points[2]);
      msg.triangles.push_back(toInsert);
    }
  
  printf("[%s:%d %f] Sending map type %d number %d with %d vertices and %d triangles\n",
   __FILE__, __LINE__, now(),
   msg.map_type,
   msg.sequence_number,
   msg.num_vertices,
   msg.num_triangles);
  
  this->mapPublisher.publish(msg);
  

 // free(msg.vertices_meters);
 // free(msg.triangles);
  
  return 1;
}

// bool ROSInterface::publish_at_goal_notification(double timestamp) {
//   printf("[%s:%d %f] publishing at-goal notification\n",
//   __FILE__, __LINE__, now());

//   rasm::RASM_GOAL_REACHED_MSG msg;
//   memset(&msg, 0, sizeof(rasm::RASM_GOAL_REACHED_MSG));

//   RASM::goal g;
//   this->update_goal(g);

//   msg.origin_x_meters     = RASM_TO_METERS(g.m_position.X());
//   msg.origin_y_meters     = RASM_TO_METERS(g.m_position.Y());
//   msg.length_meters       = (2.0 * g.m_semimajor_axis);
//   msg.width_meters        = (2.0 * g.m_semiminor_axis);
//   msg.orientation_radians = g.m_orientation_radians;
//   msg.timestamp           = timestamp;

//   this->atGoalPublisher.publish(msg);
//   return 1;

// }

// bool ROSInterface::publish_accept_goal_notification(double timestamp) {
//   printf("[%s:%d %f] publishing ACCEPT-goal notification\n",
//   __FILE__, __LINE__, now());

//   rasm::RASM_GOAL_ACCEPTED_MSG msg;
//   memset(&msg, 0, sizeof(rasm::RASM_GOAL_ACCEPTED_MSG));

//   RASM::goal g;
//   this->update_goal(g);

//   msg.origin_x_meters     = RASM_TO_METERS(g.m_position.X());
//   msg.origin_y_meters     = RASM_TO_METERS(g.m_position.Y());
//   msg.length_meters       = (2.0 * g.m_semimajor_axis);
//   msg.width_meters        = (2.0 * g.m_semiminor_axis);
//   msg.orientation_radians = g.m_orientation_radians;
//   msg.timestamp           = timestamp;
  
//   this->acceptGoalPublisher.publish(msg);

//   return 1;

// }

// bool ROSInterface::publish_delete_goal_command(double timestamp) {
//   rasm::RASM_DELETE_GOAL_MSG msg;
//   memset(&msg, 0, sizeof(rasm::RASM_DELETE_GOAL_MSG));

//   msg.timestamp = timestamp;

//   this->deleteGoalPublisher.publish(msg);
//   return 1;

// }

bool ROSInterface::publish_goal(RASM::goal &g, double timestamp) {
  printf("[%s:%d %f] publishing goal\n",
  __FILE__, __LINE__, now());

  rasm::RASM_GOAL_MSG msg;
  memset(&msg, 0, sizeof(rasm::RASM_GOAL_MSG));

  msg.origin_x_meters     = RASM_TO_METERS(g.m_position.X());
  msg.origin_y_meters     = RASM_TO_METERS(g.m_position.Y());
  msg.length_meters       = (2.0 * g.m_semimajor_axis);
  msg.width_meters        = (2.0 * g.m_semiminor_axis);
  msg.orientation_radians = g.m_orientation_radians;
  msg.timestamp           = timestamp;

  this->goalPublisher.publish(msg);
  return 1;
}

//from wikipedia https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
Quaternion2 ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
  //RASM is y forward, so pitch is along x and negative
  pitch = -pitch;
  //roll is still the same

    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion2 q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

/*!
 *This function is called by @node sim_posest. Presumably, we dont have to switch the axis
 */
bool ROSInterface::publish_pose(RASM::point3d position, float roll_rad, float pitch_rad,
                  float yaw_rad, double timestamp) {
  printf("[%s:%d %f] publishing pose\n",
  __FILE__, __LINE__, now());

  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = RASM_TO_METERS(position.X());
  msg.pose.pose.position.y = RASM_TO_METERS(position.Y());
  msg.pose.pose.position.z = RASM_TO_METERS(position.Z());

  Quaternion2 q = ToQuaternion(yaw_rad, pitch_rad,roll_rad);
  msg.pose.pose.orientation.w = q.w;
  msg.pose.pose.orientation.x = q.x;
  msg.pose.pose.orientation.y = q.y;
  msg.pose.pose.orientation.z = q.z;
  
  for(int i=0;i<36;i++)
  {
    msg.pose.covariance[i] = 0;
  }
  msg.header.stamp = ros::Time::now();
  
  this->posePublisher.publish(msg);
  return 1;
}

// void ROSInterface::publish_stop_cmd() {
//   rasm::RASM_STOP_COMMAND_MSG msg;
//   memset(&msg, 0, sizeof(rasm::RASM_STOP_COMMAND_MSG));
//   msg.timestamp = now();

//   this->stopCommandPublisher.publish(msg);
//   return;
// }

void ROSInterface::publish_path_cmd(uint32_t path_index) {
  rasm::RASM_PATH_COMMAND_MSG msg;
  memset(&msg, 0, sizeof(rasm::RASM_PATH_COMMAND_MSG));
  msg.path_index = path_index;
  msg.timestamp  = now();

  this->pathPublisher.publish(msg);
  return;
}

void ROSInterface::publish_path_cmd(bool driving_forward, uint32_t path_index) {
  rasm::RASM_PATH_COMMAND_MSG msg;
  memset(&msg, 0, sizeof(rasm::RASM_PATH_COMMAND_MSG));
  msg.path_index = path_index;
  msg.forward    = driving_forward;
  msg.timestamp  = now();

  this->pathPublisher.publish(msg);
  return;
}

void ROSInterface::publish_drive_arc(float _radius, float _speed, float _duration) {
  rasm::RASM_DRIVE_ARC_MSG msg;
  msg.radius = _radius;
  msg.speed = _speed;
  msg.duration = _duration;
  msg.timestamp = now();
  this->driveArcPublisher.publish(msg);
  return;
}

// void ROSInterface::publish_arc_costs(int arc_set_size, float *arc_costs, bool _direction) {
//   rasm::ARC_COSTS msg;
//   msg.arc_set_size = arc_set_size;
//   msg.direction    = _direction;

//   int i = 0;
//   for (i; i < arc_set_size; ++i) {
//     msg.near.push_back(arc_costs[i]);
//     msg.far.push_back(arc_costs[i + arc_set_size]);
//     msg.total.push_back(arc_costs[i + (arc_set_size * 2)]);
//   }
//   this->arcCostPublisher.publish(msg);
//   return;
// }

bool ROSInterface::subscribe_to_goal_cmds() {
  this->goalCommandsSubscriber = n.subscribe("/rover_executive/goal_command", 1, &ROSInterface::goal_handler, this);
  // this->deleteGoalSubscriber = n.subscribe("/rover_executive/delete_goal", 1, &ROSInterface::delete_goal_handler, this);
  return 1;
}

// bool ROSInterface::subscribe_to_goal_msgs() {
//   this->goalAcceptedSubscriber = n.subscribe("/navigator/goal_accepted", 1, &ROSInterface::goal_accepted_handler, this);
//   this->goalReachedSubscriber  = n.subscribe("/rover_executive/goal_reached", 1, &ROSInterface::goal_reached_handler, this);
//   return 1;
// }

bool ROSInterface::subscribe_to_map() {
  this->mapSubscriber = n.subscribe("/mapper/current_map", 1, &ROSInterface::map_handler, this);
  return 1;
}

bool ROSInterface::subscribe_to_path_cmds() {
  this->pathSubscriber = n.subscribe("/navigator/drive_arc_index", 1, &ROSInterface::path_cmd_handler, this);
  // this->stopCommandSubscriber = n.subscribe("/rover_executive/stop_command", 1, &ROSInterface::stop_cmd_handler, this);
  return 1;
}

/*!
 * The following function converts an /Odometry/Filtered message of type nav_msgs::Odometry to a
 * RASM ROBOT_POSE_MSG. ROS's local frame is NED; RASM's is ENU; the following function SHOULD
 * account for this disconnect (note the commented-out portions to do so).
 */
rasm::ROBOT_POSE_MSG ROSInterface::Odom_to_RasmPose_converter(const nav_msgs::Odometry::ConstPtr& odom)
{
  rasm::ROBOT_POSE_MSG pose_command;
  //pose_command.x_meters = odom->pose.pose.position.x;
  //pose_command.y_meters = odom->pose.pose.position.y;
  pose_command.x_meters = -odom->pose.pose.position.y;
  pose_command.y_meters = odom->pose.pose.position.x;
  pose_command.z_meters = odom->pose.pose.position.z;

  //pose_command.dx_meters_per_sec = odom->twist.twist.linear.x;
  //pose_command.dy_meters_per_sec = odom->twist.twist.linear.y;

  pose_command.dx_meters_per_sec = -odom->twist.twist.linear.y;
  pose_command.dy_meters_per_sec = odom->twist.twist.linear.x;
  pose_command.dz_meters_per_sec = odom->twist.twist.linear.z;
  Quaternion2 pose_q;
  pose_q.w = odom->pose.pose.orientation.w;
  pose_q.x = odom->pose.pose.orientation.x;
  pose_q.y = odom->pose.pose.orientation.y;
  pose_q.z = odom->pose.pose.orientation.z;
  EulerAngles pose_angles = ToEulerAngles(pose_q);
  pose_command.roll_rad  = pose_angles.pitch; //TODO: Might need to be negative
  pose_command.pitch_rad = pose_angles.roll;
  pose_command.yaw_rad   = pose_angles.yaw;
  pose_command.droll_rad_per_sec  = odom->twist.twist.angular.y; //TODO: Might need to be negative
  pose_command.dpitch_rad_per_sec = odom->twist.twist.angular.x;
  pose_command.dyaw_rad_per_sec   = odom->twist.twist.angular.z;
  //printf("roll, pitch, and yaw: %d, %d, %d", pose_command.roll_
  //pose_command.roll_rad = pose_angles.roll;
  //pose_command.pitch_rad = pose_angles.pitch;
  //pose_command.yaw_rad = pose_angles.yaw;
  //pose_command.droll_rad_per_sec = odom->twist.twist.angular.x;
  //pose_command.dpitch_rad_per_sec = odom->twist.twist.angular.y;
  //pose_command.dyaw_rad_per_sec = odom->twist.twist.angular.z;

  /*!
   * 12/05/2020:
   * As far as I can tell, the navigator's covariance matrix is not currently used
   * within navigation itself UNLESS using covariance decimation of the point clouds.
   */
  for(int i=0;i<36;i++)
  {
      pose_command.covariance_matrix.push_back(odom->pose.covariance[i]); //TODO: May need to transform this matrix because of misaligned frames between ROS and RASM
  }
  pose_command.timestamp = now();
  return pose_command;
}

EulerAngles ToEulerAngles(Quaternion2 q) {
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
      angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      angles.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}
