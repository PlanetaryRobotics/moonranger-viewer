/*!
 * 12/05/2020:
 * This interface--implemented summer 2020--serves as the primary means of message-passing between the navigator's
 * associated nodes. It is designed in the ROS framework and based upon the implementation of RASM circa 2012 (which
 * utilized IPC (https://www.cs.cmu.edu/afs/cs/project/TCA/ftp/IPC_Manual.pdf)).
 *
 * An extremely high-level description of the ROS framework: "Nodes" are individual programs that contain specific,
 * user-defined functionalities. Nodes "subscribe" and "publish" to "topics," and "messages" with some specific datatypes
 * are passed between nodes through these means to jointly produce system-level functionalities. The navigator-specific
 * message-types are defined within the rasm/src/msg directory. For further explanation of the ROS framework,
 * see http://wiki.ros.org/ROS/Tutorials.
 */

#ifndef __ROS_INTERFACE__
#define __ROS_INTERFACE__

#include <assert.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include <sys/time.h>
#include <util.h>
#include <cmath>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "rasm/ROBOT_POSE_MSG.h"
#include "rasm/RASM_GOAL_MSG.h"
#include "rasm/RASM_MAP_MSG.h"
// #include "rasm/RASM_GOAL_REACHED_MSG.h"
// #include "rasm/RASM_GOAL_ACCEPTED_MSG.h"
#include "rasm/RASM_PATH_COMMAND_MSG.h"
// #include "rasm/RASM_STOP_COMMAND_MSG.h"
// #include "rasm/RASM_DELETE_GOAL_MSG.h"
#include "rasm/RASM_DRIVE_ARC_MSG.h"
// #include "rasm/ARC_COSTS.h"
#include <comms_interface.h>

/*!
 * 12/05/2020:
 * Note that ROSInterface derives from the base class "CommsInterface" which is declared and defined within the
 * /core/common directory. Also note the difference between 'send_*' and 'publish_*' functions: 'send_*' functions
 * are wrappers for the 'publish_*' functions. These wrappers are in place to publish messages AND perform some
 * other computation that must occur simultaneously. This layering is a feature of RASM circa 2012; it's not yet
 * been deemed worthwhile to determine if the same functionality can be achieved without wrapping the publishers.
 *
 * It is worth noting that the planner calls the wrapper functions (typically from within rasmEval.cc)
 */

class ROSInterface : public RASM::CommsInterface {
  public:
  //Constructor & destructor:
  ROSInterface(const char *module_name, std::map<std::string, std::string>& config, bool zero_first_position = false);
  virtual ~ROSInterface();

  //Publish and subscribe functions:
  bool pause(unsigned int time_ms = GENERIC_PAUSE_TIME_MSEC) const;
  bool publish_map(unsigned int map_type, unsigned int sequence_number,
                   RASM::mesh &map, RASM::point3d origin, double timestamp);
  // bool publish_at_goal_notification(double timestamp);
  // bool publish_accept_goal_notification(double timestamp);
  // bool publish_delete_goal_command(double timestamp);
  bool publish_goal(RASM::goal &g, double timestamp);
  bool publish_pose(RASM::point3d position, float roll_rad, float pitch_rad,
                    float yaw_rad, double timestamp);
  // void publish_stop_cmd();
  void publish_path_cmd(uint32_t path_index);
  void publish_path_cmd(bool driving_forward, uint32_t path_index);
  void publish_drive_arc(float _radius, float _speed, float _duration);
  // void publish_arc_costs(int arc_set_size, float *arc_costs, bool _direction);
  bool subscribe_to_goal_cmds();
  // bool subscribe_to_goal_msgs();
  bool subscribe_to_map();
  bool subscribe_to_path_cmds();

  bool m_zero_first_position;

  std::string m_pose_estimate_topic;

  // Callback functions:
  void robot_pose_handler(const nav_msgs::Odometry::ConstPtr& odom);
  void goal_handler(rasm::RASM_GOAL_MSG call_data);
  // void goal_reached_handler(rasm::RASM_GOAL_REACHED_MSG call_data);
  // void goal_accepted_handler(rasm::RASM_GOAL_ACCEPTED_MSG call_data);
  // void delete_goal_handler(rasm::RASM_DELETE_GOAL_MSG call_data);
  void map_handler(rasm::RASM_MAP_MSG call_data);
  void path_cmd_handler(rasm::RASM_PATH_COMMAND_MSG call_data);
  // void stop_cmd_handler(rasm::RASM_STOP_COMMAND_MSG call_data);
  rasm::ROBOT_POSE_MSG Odom_to_RasmPose_converter(const nav_msgs::Odometry::ConstPtr& odom);

  ros::NodeHandle n;
  //Subscribers:
  ros::Subscriber mapSubscriber;
  ros::Subscriber poseSubscriber;
  ros::Subscriber pathSubscriber;
  // ros::Subscriber stopCommandSubscriber;
  // ros::Subscriber goalReachedSubscriber;
  ros::Subscriber goalCommandsSubscriber;
  // ros::Subscriber deleteGoalSubscriber;
  ros::Subscriber goalMessagesSubscriber;
  // ros::Subscriber goalAcceptedSubscriber;
  //Publishers:
  // ros::Publisher  atGoalPublisher;
  ros::Publisher  mapPublisher;
  // ros::Publisher  acceptGoalPublisher;
  // ros::Publisher  deleteGoalPublisher;
  ros::Publisher  goalPublisher;
  ros::Publisher  posePublisher;
  ros::Publisher  eulerPosePublisher;
  // ros::Publisher  stopCommandPublisher;
  ros::Publisher  pathPublisher;
  ros::Publisher  driveArcPublisher;
  // ros::Publisher  arcCostPublisher;
};

//Miscellaneous/utilities:
struct Quaternion2{
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};
EulerAngles ToEulerAngles(Quaternion2 q);

#endif /* __ROS_INTERFACE__ */
