#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <cmath>
#include <stdlib.h>
#include <comms_interface.h>
#include <rasm_common_types.h>
#include <safety_gate.h>
#include <config.h>
#include <util.h>
#include <unistd.h>

/*
 * Stub function to start the SafetyGate object's work loop within a thread.
 */ 
static void *start_work_loop(void *args)
{
  RASM::SafetyGate* sg = (RASM::SafetyGate*)args;
  assert(NULL != sg);
  sg->work_loop();
  return NULL;
}


RASM::SafetyGate::SafetyGate(std::map<std::string, std::string>& config,
			     CommsInterface& interface) 
  : m_interface(interface),
    m_thread_id(0),
    m_shutdown_flag(false),
    m_state(UNINITIALIZED),
    m_pose_watchdog_period_sec(0.0),
    m_map_watchdog_period_sec(0.0)
{
  if(config.find("pose_watchdog_period_sec") != config.end()) m_pose_watchdog_period_sec = atof(config["pose_watchdog_period_sec"].c_str());
  if(config.find("map_watchdog_period_sec") != config.end())  m_map_watchdog_period_sec  = atof(config["map_watchdog_period_sec"].c_str());
  
  interface.m_safety_gate = this;

  assert(m_pose_watchdog_period_sec > 0.0);
  assert(std::isfinite(m_pose_watchdog_period_sec));

  assert(m_map_watchdog_period_sec > 0.0);
  assert(std::isfinite(m_map_watchdog_period_sec));
}


RASM::SafetyGate::~SafetyGate()
{
  stop();
}

/*
 * This function is called to start the monitoring thread. Will return false if 
 * the thread can't be created for some reason, returns true otherwise.
 */
bool RASM::SafetyGate::start()
{
  /*
   * Monitoring thread is already running, so don't disturb it.
   */
  if(0 != m_thread_id) return true;

  /*
   * Try creating a new thread. Before starting it make sure to reset the state
   * to UNINITIALIZED.
   */
  m_state = UNINITIALIZED;
  pthread_mutex_init(&m_state_mutex, NULL);
  if(-1 == pthread_create(&m_thread_id,
			  NULL,
			  &start_work_loop,
			  this))
    {
      printf("[%s:%d %f] ERROR: could not start thread\n",
	     __FILE__, __LINE__, now());
      return false;
    }

  return true;
}

/*
 * This function is called to stop the monitoring thread. Always returns true.
 * Will set the SafetyGate's state to UNINITIALIZED and m_thread_id back to 0.
 */
bool RASM::SafetyGate::stop()
{
  m_shutdown_flag = true;
  pthread_join(m_thread_id, NULL);

  m_state = UNINITIALIZED;
  m_thread_id = 0;

  return true;
}


/*
 * Here is the monitoring loop. Basically drives a state machine based on the 
 * state of the CommsInterface. Then it takes actions ("sets outputs") based on
 * the current state.
 */
bool RASM::SafetyGate::work_loop()
{
  /*
   * The work loop that monitors the CommsInterface. 
   */
  while(!m_shutdown_flag)
    {
      static const uint32_t WORK_LOOP_WAIT_USEC = 1000;
      usleep(WORK_LOOP_WAIT_USEC);
      //      m_interface.pause(WORK_LOOP_WAIT_MSEC);

      /*
       * First check watchdogs
       */
      bool pose_watchdog_timed_out = true;
      bool map_watchdog_timed_out = true;
      
      double current_time = now();
      assert(std::isfinite(current_time));

      if(std::isfinite(m_interface.m_latest_pose.getTime()))
	{
	  if( (current_time - m_interface.m_latest_pose.getTime()) < m_pose_watchdog_period_sec )
	    {
	      pose_watchdog_timed_out = false;
	    }
	}

      if(std::isfinite(m_interface.get_last_map_timestamp()))
	{
	  if( (current_time - m_interface.get_last_map_timestamp()) < m_map_watchdog_period_sec )
	    {
	      map_watchdog_timed_out = false;
	    }
	}

      /*
       * Then check for invalid pose and map data
       */
      bool pose_invalid = true;
      if(m_interface.m_latest_pose.isValid() &&
       	 m_interface.m_previous_pose.isValid())
	{
	  pose_invalid = false;
	}

      // TBD: check the validity of map data

      /*
       * Then check for state transitions. Update m_state.
       */
      pthread_mutex_lock(&m_state_mutex);   ///////////////

      // printf("[%s:%d %f] current state: %d, pose wd: %d (%f), map wd %d (%f), pose invalid %d\n",
      // 	     __FILE__, __LINE__, now(), 
      // 	     m_state,
      // 	     pose_watchdog_timed_out, m_interface.m_latest_pose.getTime(),
      // 	     map_watchdog_timed_out, m_interface.get_last_map_timestamp(),
      // 	     pose_invalid);
      // fflush(stdout);

      switch(m_state)
	{
	case UNINITIALIZED:
	  {
	    // always transition to ERROR
	    m_state = ERROR;
	    printf("[%s:%d %f] UNINITIALIZED --> ERROR\n", __FILE__, __LINE__, now());
	    break;
	  }
	case OK:
	  {
	    // transition to ERROR if:
	    // - pose watchdog timeout
	    // - map watchdog timeout
	    // - latest or previous pose is invalid
	    // - latest map is invalid
	    if(pose_watchdog_timed_out ||
	       map_watchdog_timed_out  ||
	       pose_invalid)
	      {
		m_state = ERROR;
		printf("[%s:%d %f] OK --> ERROR %d %d %d\n", __FILE__, __LINE__, now(),
		       pose_watchdog_timed_out,
		       map_watchdog_timed_out,
		       pose_invalid);
	      }
	    break;
	  }
	case ERROR:
	  {
	    // transition to OK only if:
	    // - pose watchdog not timed out
	    // - map not watchdog timed out
	    // - latest and previous pose is valid
	    // - latest map is valid
	    if(!pose_watchdog_timed_out &&
	       !map_watchdog_timed_out  &&
	       !pose_invalid)
	      {
		m_state = OK;
		printf("[%s:%d %f] ERROR --> OK\n", __FILE__, __LINE__, now());
	      }
	    break;
	  }
	default:
	  {
	    // something is strange, so set error...
	    m_state = ERROR;
	    printf("[%s:%d %f] UNKNOWN --> ERROR\n", __FILE__, __LINE__, now());
	    continue;
	  }
	} // check for state transitions

      //      uint8_t current_state = m_state;
      pthread_mutex_unlock(&m_state_mutex); ///////////////

    } // the main loop

  return true;

} // work_loop()


uint8_t RASM::SafetyGate::get_state()
{
  uint8_t state = RASM::SafetyGate::ERROR;
  pthread_mutex_lock(&m_state_mutex);   //////////////////
  state = m_state;
  pthread_mutex_unlock(&m_state_mutex); //////////////////
  if(is_state_valid(state)) 
    {
      return state;
    }
  else
    {
      return RASM::SafetyGate::ERROR;
    }
}
