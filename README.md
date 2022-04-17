# MoonRanger Mesh Viewer
Mesh  viewer for MoonRanger's autonomous terrain mapping software. This code has its origins as the viewer component of RASM, which is described below.

## Origin
The acronym 'RASM' appears throughout the codebase. This term refers to Dr. Wettergreen's Reliable Autonomous Surface Mobility navigation framework, from which the MoonRanger software directly derives. Indeed, the majority of the algorithmic processes defined within this codebase stem directly from the original RASM implementations and may appear in the form as originally found in the 'RASM' repository. Moreover, many individuals have contributed to the codebase over the years, including David Wettergreen, Michael Furlong, Michael Wagner, Ryan McNulty, and Dom Jonak.

For original documentation and journal publication of the RASM software, consult the Drive: 03 Software/Subteam Work/Planning & Navigation/Research/RASM Background
- Drive URL: https://drive.google.com/drive/u/1/folders/1pdmCXVLrA07hE8gancdqH2VsDNAAwp-q

## Setup
This repository is set up as a catkin workspace for integration with ROS. To build the code, run the following:
```
git clone git@github.com:margareh/moonranger-viewer.git
cd moonranger-viewer
catkin_make
source devel/setup.bash
```

Most of RASM's test datasets have been built with ROS Melodic, so ROS Melodic is recommended. The code will build with Noetic but the md5 sums for RASM messages used for mesh and drive arc topics (among other things) will not match those saved in rosbags for which Melodic was used.

## Dependencies
The code depends on two external libraries:
- [Eigen 3.3.7](https://gitlab.com/libeigen/eigen/-/tags/3.3.7)
- [Libigl 2.3.0](https://github.com/libigl/libigl/tree/v2.3.0)

Both are included as in subdirectories under `src/rasm/external` so there is no need to clone the repository recursively or install these dependencies separately.
