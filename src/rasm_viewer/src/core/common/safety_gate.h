#ifndef __SAFETY_GATE_H__
#define __SAFETY_GATE_H__

#include <comms_interface.h>
#include <pthread.h>
#include <config.h>

namespace RASM {
  
  class CommsInterface; // fwd ref, see comms_interface.h

  class SafetyGate
  {
  public:
    SafetyGate(std::map<std::string, std::string>& config,
	       RASM::CommsInterface& interface);
    ~SafetyGate();

    bool start();
    bool stop();

    bool work_loop();

    static const uint8_t UNINITIALIZED = 0;
    static const uint8_t OK = 1;
    static const uint8_t ERROR = 3;
    uint8_t get_state();
    bool is_state_valid(uint8_t state)
    {
      if((OK == state) ||
	 (ERROR == state))
	{
	  return true;
	}
      else
	{
	  return false;
	}
    }

    RASM::CommsInterface& m_interface;    
    pthread_t             m_thread_id;
    bool                  m_shutdown_flag;

    pthread_mutex_t       m_state_mutex;
    uint8_t               m_state;

    /*
     * The watchdogs used
     */
    double m_pose_watchdog_period_sec;
    double m_map_watchdog_period_sec;

  };

} /* end of namespace RASM */

#endif /* __SAFETY_GATE_H__ */
