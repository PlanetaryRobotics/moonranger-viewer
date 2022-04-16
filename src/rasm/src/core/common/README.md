## This directory is home to libraries that are used frequently across both the mapper and the planner. Of particular note are:

### comms_interface.h/.cc:
The library which declares/defines the base message-passing functionalities of an interface. This library is the base class for our ROS interface, MoonrangerArcModel, and ros_point_cloud_interface.

### rasm_common_types.h:
This library defines the RASM-specific data types and manipulators, including point2d, point3d, mesh, RASM_UNITS, and other pertinent functionalities (including various matrix computations).

### config.h/.cc:
Declares and defines the means by which we construct an interface's instantiation from a config file.

### safety_gate.h/.cc:
Declares and defines the reactionary response to missing/late data (including invalid map and pose).

### path_model.h:
Directory which declares and defines the base class for an arc interface (see MoonrangerArcModel for rover-specific definition).
