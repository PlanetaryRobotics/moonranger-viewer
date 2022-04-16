/**
 * @file comms_interface.h
 *
 * @section LICENSE
 * Copyright 2012, Carnegie Mellon University and ProtoInnovations, LLC. 
 * All rights reserved. This document contains information that is proprietary
 * to Carnegie Mellon University and ProtoInnovations, LLC. Do not distribute
 * without approval.
 *
 */

/*!
 * 12/10/2020:
 * This is the base class for a communication (ie message-passing) interface.
 * MoonRanger's derived interface is the ROS_interface which defines
 * framework- and MoonRanger-specific message-passing functionalities.
 *
 * Note the 'send_*' functions, which wrap the ROS_interface
 * 'publish_*' functions with other simultaneous functionalities.
 */

#ifndef __COMMS_INTERFACE_H__
#define __COMMS_INTERFACE_H__

#include <stdint.h>
#include <rasm_common_types.h>
#include <rasm_pose.h>
#include <safety_gate.h>

namespace RASM {

  class SafetyGate; // fwd ref, see safety_gate.h

  /*!
   * 12/07/2020:
   * Categorize a tmap based on its type:
   */
  class MapType
  {
  public:
    static const unsigned int WORLD = 0;
    static const unsigned int ARC = 1;
    static const unsigned int PATH = 2;
    static const unsigned int NUM_MAP_TYPES = 3;
  };

  class CommsInterface
  {
  public:

    static const uint32_t GENERIC_PAUSE_TIME_MSEC = 100;

    /***************************************************************************
     * Below are the methods to override when implementing a real interface.
     */

    /*
     * Pause for a given period of time, perhaps allowing comms infrastructure
     * to poll for messages.
     */
    virtual bool pause(unsigned int time_msec = GENERIC_PAUSE_TIME_MSEC) const = 0;

    /*
     * Message-sending methods
     */
    virtual bool publish_map(unsigned int map_type,
			     unsigned int sequence_number,
			     RASM::mesh& map,
			     RASM::point3d origin,
			     double timestamp) = 0;
    // virtual bool publish_at_goal_notification(double timestamp) = 0;
    // virtual bool publish_accept_goal_notification(double timestamp) = 0;
    // virtual bool publish_delete_goal_command(double timestamp) = 0;
    virtual bool publish_goal(RASM::goal& g,
			      double timestamp) = 0;
    virtual bool publish_pose(RASM::point3d position,
			      float roll_rad,
			      float pitch_rad,
			      float yaw_rad,
			      double timestamp) = 0;
    // virtual void publish_stop_cmd() = 0;
    virtual void publish_path_cmd(uint32_t path_index) = 0;
    virtual void publish_path_cmd(bool driving_forward, uint32_t path_index) = 0;
    virtual void publish_drive_arc(float _radius, float _speed, float _duration) = 0;
    // virtual void publish_arc_costs(int arc_set_size, float *arc_costs, bool _direction) = 0;

    /*
     * Methods to subscribe to messages
     */
    virtual bool subscribe_to_goal_cmds() = 0;
    // virtual bool subscribe_to_goal_msgs() = 0;
    virtual bool subscribe_to_map() = 0;
    virtual bool subscribe_to_path_cmds() = 0;

    bool m_have_goal;
    bool m_have_unaccepted_goal;
    bool m_have_pending_delete_goal;
    bool m_have_at_goal_notification;

    bool m_have_new_map[RASM::MapType::NUM_MAP_TYPES];
    unsigned int m_map_sequence_numbers[RASM::MapType::NUM_MAP_TYPES];
    RASM::mesh m_stored_maps[RASM::MapType::NUM_MAP_TYPES];
    double m_map_timestamps[RASM::MapType::NUM_MAP_TYPES];

    /* goal data -- planner, assigned by set_goal function */
    goal m_latest_goal_received;

    /* goal data -- executive, assigned by message handlers */
    goal m_latest_goal_accepted;
    goal m_latest_goal_reached;

    /* current and previous pose data */
    pose m_latest_pose, m_previous_pose;

    /* time of the last map data (not cleared by getTMAP()) */
  private:
    double m_last_map_timestamp;

  public:
    /* name of sensor to shutdown & to restart */
    char* m_shutdown_sensor_name;
    char* m_restart_sensor_name;

    /*
     * Class con-de/structors. 
     */
    CommsInterface();
    virtual ~CommsInterface();

    /*
     * Wrapper commands for moving the robot. These call the virtual publish*
     * methods, possibly after checking the state of the safety gate (this
     * happens in send_path_cmd() and not send_stop_cmd()).
     */
    // void send_stop_cmd();
    void send_path_cmd(uint32_t path_index);
    void send_path_cmd(bool driving_forward, uint32_t path_index);
    void send_drive_arc(float _radius, float _speed, float _duration);
    void send_arc_costs(int arcSetSize, float *arc_costs, bool _direction);

    /*
     * Wrapper around publish_map allows us to update last_map_timestamp and
     * possibly do other things
     */
    bool send_map(unsigned int map_type,
		  unsigned int sequence_number,
		  RASM::mesh& map,
		  RASM::point3d origin,
		  double timestamp);

    /*
     * Pose-handling methods
     */
    void wait_for_pose();
    bool update_pose(pose &pose);
    bool update_pose(point3d &pos,
		     float &r, 
		     float &p, 
		     float &y,
		     double &t);

    void get_pose_at_time(point3d &position_meters,
			  float &roll_rad, 
			  float &pitch_rad, 
			  float &yaw_rad,
			  double timestamp) const;
    void get_pose_at_time(pose &pose_at_time,
			  double t) const;
    
    /*
     * Methods to get, accept and delete goals.
     */
    void wait_for_goal();
    bool update_goal(RASM::goal& g);	
    // void accept_goal(bool goal_is_valid);
    // void send_at_goal_notification(bool success);  // was clearGoal()
    // bool should_abort_goal();
    void set_goal(RASM::goal& g);
	  
    /*
     * Map-related methods.
     */
    bool have_new_map(unsigned int map_type);
    bool get_map(unsigned int map_type,
		 unsigned int& sequence_number,
		 RASM::mesh& map,
		 double& timestamp);
    bool set_map(unsigned int map_type,
		 unsigned int sequence_number,
		 RASM::mesh& map,
		 double timestamp);

    /* 
     * These functions called by message-callbacks with new data.
     * Note that set_pose is virtual so that subclasses can override it.
     */
    virtual bool set_pose(const point3d &position_meters,
			  float roll_rad, 
			  float pitch_rad, 
			  float yaw_rad,
			  double timestamp);
    bool m_have_new_path_cmd;
    bool m_driving_forward;
    uint32_t m_latest_path_index;
    double m_latest_path_cmd_timestamp;

    bool m_have_new_stop_cmd;
    double m_latest_stop_cmd_timestamp;

    void set_path_cmd(const uint32_t path_index,
          const bool driving_forward,
		      const double timestamp);
    void wait_for_arc_index();
    // void set_stop_cmd(const double timestamp);
    // void delete_goal();                      // was input_goalDel
    // void receive_at_goal_notification();     // was input_atGoal

    /*
     * Time-related methods.
     */
    //    double most_recent_data_timestamp() const { return m_last_map_timestamp; }
    //    double most_recent_data_timestamp(int source) const;
    //    double most_recent_pose_timestamp() const;
    //    double get_current_time() const;

    /*
     * The safety gate. If the safety-gate pointer is NULL, it is not used.
     */
    SafetyGate* m_safety_gate;

    double get_last_map_timestamp() { return m_last_map_timestamp; }
    
  }; /* end of class CommsInterface */

} /* end of namespace RASM */

#endif /* __COMMS_INTERFACE_H__ */
