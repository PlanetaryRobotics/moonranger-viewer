#include <stdio.h>
#include <stdlib.h>
#include <rasm_common_types.h>
#include <comms_interface.h>
#include <util.h>
#include "ros_interface.h"


#ifdef __cplusplus
extern "C" 
{
#endif //__cplusplus
#ifndef __construct_comms__
#define __construct_comms__
RASM::CommsInterface* construct_comms_interface(const char **argv, int argc)
{
  std::map<std::string, std::string> config;
  config = makeConfig(argc, argv);

  float goal_x = 0;
  float goal_y = 0;
  bool goal_specified = false;

  int i=0;
  for(;i<argc;i++)
    {
      if(0 == strncmp(argv[i], "--goal", strlen("--goal")) &&
	 argc > i+2)
	{
	  goal_x = atof(argv[i+1]);
	  goal_y = atof(argv[i+1]);
	  goal_specified = true;
	  i += 2;
	}
    }

  RASM::CommsInterface* retval = new ROSInterface(argv[0], config);

  if(goal_specified)
    {
      RASM::goal g;
      g.m_position.set(METERS_TO_RASM(goal_x),
		       METERS_TO_RASM(goal_y));
      g.m_semimajor_axis = 2.0;
      g.m_semiminor_axis = 2.0;
      g.m_orientation_radians = 0.0;
      retval->set_goal(g);
    }

  return retval;

}
#endif

#ifdef __cplusplus
} // extern "C"
#endif //__cplusplus
