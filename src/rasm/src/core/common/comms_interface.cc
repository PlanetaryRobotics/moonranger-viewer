#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <comms_interface.h>
#include <util.h>

/*!
 * 12/10/2020:
 * This is the base class for a communication (ie message-passing) interface.
 * MoonRanger's derived interface is the ROS_interface which defines
 * framework- and MoonRanger-specific message-passing functionalities.
 *
 * Note the 'send_*' functions, which wrap the ROS_interface
 * 'publish_*' functions with other simultaneous functionalities.
 */


/* artificially attenuate motion by this factor
 * to stability interpolation/extrapolation
 */
static const double alpha = 0.5;


RASM::CommsInterface::CommsInterface()
  : m_have_goal(false),
    m_have_unaccepted_goal(false),
    m_have_pending_delete_goal(false),
    m_have_at_goal_notification(false),
    m_last_map_timestamp(-1.0),
    m_shutdown_sensor_name(NULL),
    m_restart_sensor_name(NULL),
    m_have_new_path_cmd(false),
    m_driving_forward(true),
    m_latest_path_index(0),
    m_latest_path_cmd_timestamp(-1.0),
    m_have_new_stop_cmd(false),
    m_latest_stop_cmd_timestamp(-1.0),
    m_safety_gate(NULL)
{
  for(unsigned int i=0; i < RASM::MapType::NUM_MAP_TYPES; i++)
    {
      m_have_new_map[i] = false;
      m_map_sequence_numbers[i] = 0;
      m_map_timestamps[i] = -1.0;
    }
}


RASM::CommsInterface::~CommsInterface()
{
  m_have_goal = false;
  m_have_unaccepted_goal = false;
  m_have_pending_delete_goal = false;
  m_have_at_goal_notification = false;
}



void RASM::CommsInterface::wait_for_goal()
{
  while( !(m_have_goal || m_have_unaccepted_goal) ) 
    {
      pause();
    }
}


void RASM::CommsInterface::wait_for_pose()
{
  //wait till both the previous and current poses are valid
  while( !(m_latest_pose.isValid() && m_previous_pose.isValid()) ) 
    {
      pause();
    }
}

void RASM::CommsInterface::wait_for_arc_index()
{
  //Wait until interface has a new path arc to follow
  while( !m_have_new_path_cmd )
    {
      pause();
    }
}

bool RASM::CommsInterface::update_goal(RASM::goal &g)
{
  pause(0);

  g = m_latest_goal_received;

  return m_have_unaccepted_goal;
}


// NOTE: This function's name should be get_pose!
bool RASM::CommsInterface::update_pose(RASM::point3d &pos,
				       float &r, 
				       float &p, 
				       float &y,
				       double &t)
{
  pause(0);

  if(!m_latest_pose.isValid())return false;

  m_latest_pose.get(pos, r, p, y);

  if(m_latest_pose.getTime() + 1e-6 > t){
    t = m_latest_pose.getTime();
    return true;
  }
  return false;
}

// NOTE: This function's name should be get_pose!

bool RASM::CommsInterface::update_pose(RASM::pose &pose)
{
  pause(0);

  if(!m_latest_pose.isValid()) return false;

  double t = pose.getTime(); /* store the old time */
  pose = m_latest_pose;

  return (m_latest_pose.getTime() + 1e-6 > t);
}


// void RASM::CommsInterface::accept_goal(bool goal_is_valid)
// {
//   assert(!m_have_goal);
//   assert(m_have_unaccepted_goal);

//   m_have_unaccepted_goal = false;

//   if(goal_is_valid)
//     {
//       printf("[%s:%d %f] Accepted new goal\n",
// 	     __FILE__, __LINE__, now());
//       m_have_goal = true;
//     }
//   else
//     {
//       printf("[%s:%d %f] Rejected new goal\n",
// 	     __FILE__, __LINE__, now());
//     }

//   publish_accept_goal_notification(now());
// }


// void RASM::CommsInterface::send_at_goal_notification(bool success)
// {
//   assert(m_have_goal);
//   m_have_goal = false;

//   if(success) 
//     {
//       publish_at_goal_notification(now());
//     }

// }


// bool RASM::CommsInterface::should_abort_goal()
// {
//   /* sanity-check */
//   assert(m_have_goal);

//   /* 
//    * Check if a goal-delete command or at-goal notification arrived.
//    * In either case, clear both flags.
//    */
//   if(m_have_pending_delete_goal || m_have_at_goal_notification)
//     {
//       m_have_pending_delete_goal = m_have_at_goal_notification = false;
//       return true;
//     }

//   return false;
// }


void RASM::CommsInterface::get_pose_at_time(RASM::point3d &position_meters,
					    float &roll_rad, 
					    float &pitch_rad, 
					    float &yaw_rad,
					    double timestamp) const
{
  /* sanity-check */
  assert(m_latest_pose.isValid());
  assert(m_previous_pose.isValid());

  RASM::pose::linearFit(m_latest_pose, 
			m_previous_pose, 
			timestamp, 
			alpha, 
			position_meters, 
			roll_rad, 
			pitch_rad,
			yaw_rad);
}


void RASM::CommsInterface::get_pose_at_time(RASM::pose &pose_at_time,
					    double timestamp) const
{
  assert(m_latest_pose.isValid()); 
  assert(m_previous_pose.isValid());

  RASM::point3d position_meters;
  float roll_rad = 0;
  float pitch_rad = 0;
  float yaw_rad = 0;

  RASM::pose::linearFit(m_latest_pose, 
			m_previous_pose, 
			timestamp,
			alpha, 
			position_meters, 
			roll_rad, 
			pitch_rad, 
			yaw_rad);

  pose_at_time.set(position_meters, 
		   roll_rad, 
		   pitch_rad, 
		   yaw_rad,
		   timestamp);
}


bool RASM::CommsInterface::have_new_map(unsigned int map_type)
{
  if(map_type >= RASM::MapType::NUM_MAP_TYPES) return false;
  
  return m_have_new_map[map_type];
}

bool RASM::CommsInterface::get_map(unsigned int map_type,
				   unsigned int& sequence_number,
				   RASM::mesh& map,
				   double& timestamp)
{
  if(map_type >= RASM::MapType::NUM_MAP_TYPES) return false;

  sequence_number = m_map_sequence_numbers[map_type];
  timestamp       = m_map_timestamps[map_type];
  map.copy_from(m_stored_maps[map_type]);

  m_have_new_map[map_type] = false;
    
  return true;
}


bool RASM::CommsInterface::set_pose(const RASM::point3d &position_meters,
				    float roll_rad, 
				    float pitch_rad, 
				    float yaw_rad,
				    double timestamp)
{
  // printf("[%s:%d %f] latest pose (%f %f %f %f) has timestamp %f (overwriting pose at time %f)\n",
  //  	 __FILE__, __LINE__, now(),
  // 	 RASM_TO_METERS(position_meters.X()),
  // 	 RASM_TO_METERS(position_meters.Y()),
  // 	 RASM_TO_METERS(position_meters.Z()),
  // 	 yaw_rad * 180.0 / M_PI,
  //  	 timestamp, m_latest_pose.getTime());
  
  if(m_latest_pose.isValid())
    {
      if(timestamp <= m_latest_pose.getTime())
	{
	  printf("[%s:%d %f] WARNING: ignoring incoming pose with invalid timestamp (incoming timestamp is %f, latest pose timestamp is %f)\n",
		 __FILE__, __LINE__, now(),
		 timestamp,
		 m_latest_pose.getTime());
	  return false;
	}
      
      m_previous_pose = m_latest_pose;

    }

  m_latest_pose.set(position_meters, roll_rad, pitch_rad, yaw_rad, timestamp);

  return true;
}

// TODO Create goal object and update
void RASM::CommsInterface::set_goal(RASM::goal& g)
{
  m_latest_goal_received = g;
	
  m_have_unaccepted_goal = true;

  printf("[%s:%d %f] got goal (%0.3f, %0.3f) with semi-major/minor axes %0.3f/%0.3f, orientation %f deg\n", 
	 __FILE__, __LINE__, now(),
	 RASM_TO_METERS(m_latest_goal_received.m_position.X()), 
	 RASM_TO_METERS(m_latest_goal_received.m_position.Y()), 
	 m_latest_goal_received.m_semimajor_axis,
	 m_latest_goal_received.m_semiminor_axis,
	 m_latest_goal_received.m_orientation_radians * 180.0/M_PI);
}


void RASM::CommsInterface::set_path_cmd(const uint32_t path_index, const bool driving_forward, const double timestamp)
{
  m_latest_path_index = path_index;
  m_driving_forward = driving_forward;
  m_latest_path_cmd_timestamp = timestamp;
  m_have_new_path_cmd = true;
}

// void RASM::CommsInterface::set_stop_cmd(const double timestamp)
// {
//   m_latest_stop_cmd_timestamp = timestamp;
//   m_have_new_stop_cmd = true;
// }

// void RASM::CommsInterface::delete_goal()
// {
//   m_have_pending_delete_goal = true;
// }


// void RASM::CommsInterface::receive_at_goal_notification()
// {
//   m_have_at_goal_notification = true;
// }


// double RASM::CommsInterface::most_recent_pose_timestamp() const 
// {
//   if(m_latest_pose.isValid())
//     {
//       return m_latest_pose.getTime();
//     } 

//   return -1.0;
// }


// double RASM::CommsInterface::get_current_time() const
// {
//   return now();
// }

bool RASM::CommsInterface::set_map(unsigned int map_type,
				   unsigned int sequence_number,
				   RASM::mesh& map,
				   double timestamp)
{
  if(map_type >= RASM::MapType::NUM_MAP_TYPES) return false;
  
  if(NULL == map.m_vertices) return false;
  if(NULL == map.m_faces) return false;
  if(map.m_num_vertices < 3) return false;

  m_stored_maps[map_type].copy_from(map);

  m_have_new_map[map_type]         = true;
  m_map_sequence_numbers[map_type] = sequence_number;
  m_map_timestamps[map_type]       = timestamp;

  m_last_map_timestamp = timestamp;

  return true;
}

// void RASM::CommsInterface::send_stop_cmd()
// {
//   /*
//    * We needn't check the status of the safety gate here; we always want to
//    * send a stop command.
//    */
//   publish_stop_cmd();
// }

void RASM::CommsInterface::send_path_cmd(uint32_t path_index)
{
  printf("[%s:%d %f] calling send_path_cmd with index %u, state is %d\n",
	 __FILE__, __LINE__, now(), path_index, m_safety_gate->get_state());
  fflush(stdout);

  if(NULL != m_safety_gate) 
    {
      uint8_t safety_gate_state = m_safety_gate->get_state();

      if(SafetyGate::OK == safety_gate_state)
	{
	  publish_path_cmd(path_index);
	}
    }
  else
    {
      // safety gate pointer is NULL, so don't check it
      publish_path_cmd(path_index);
    }

}

void RASM::CommsInterface::send_path_cmd(bool driving_forward, uint32_t path_index)
{
  printf("[%s:%d %f] calling send_path_cmd with index %u, state is %d\n",
	 __FILE__, __LINE__, now(), path_index, m_safety_gate->get_state());
  fflush(stdout);

  if(NULL != m_safety_gate) 
    {
      uint8_t safety_gate_state = m_safety_gate->get_state();

      if(SafetyGate::OK == safety_gate_state)
	{
	  publish_path_cmd(driving_forward, path_index);
	}
    }
  else
    {
      // safety gate pointer is NULL, so don't check it
      publish_path_cmd(driving_forward, path_index);
    }

}

void RASM::CommsInterface::send_drive_arc(float _radius, float _speed, float _duration)
{
  printf("[%s:%d %f] calling send_drive_arc of radius %f, speed %f, and duration %f\n",
	 __FILE__, __LINE__, now(), _radius, _speed, _duration);
  fflush(stdout);

  publish_drive_arc(_radius, _speed, _duration);
  pause(0);

  return;
}

// void RASM::CommsInterface::send_arc_costs(int arc_set_size, float *arc_costs, bool _direction)
// {
//   publish_arc_costs(arc_set_size, arc_costs, _direction);
//   pause(0);

//   return;
// }

bool RASM::CommsInterface::send_map(unsigned int map_type,
				    unsigned int sequence_number,
				    RASM::mesh& map,
				    RASM::point3d origin,
				    double timestamp)
{
 if(map_type >= RASM::MapType::NUM_MAP_TYPES) return false;

 m_map_timestamps[map_type] = timestamp;
 m_last_map_timestamp       = timestamp;

 return publish_map(map_type, 
		    sequence_number,
		    map,
		    origin,
		    timestamp);
}

