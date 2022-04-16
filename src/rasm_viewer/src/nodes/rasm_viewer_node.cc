#include <ros/ros.h>
#include <viewer.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "rasm_viewer");

    viewer(argc,argv);

    return 0;

}