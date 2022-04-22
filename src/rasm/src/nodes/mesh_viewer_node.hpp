/****************************************************************
 *
 * @file      mesh_viewer_node.hpp
 *
 * @brief     Helper file for Moonranger mesh visualization
 *
 * @version   1.0
 * @date      04/22/22
 *
 * @authors   Ashwin Nehete, ...
 * @author    Carnegie Mellon University, Planetary Robotics Lab
 *
 ****************************************************************/

#include <ros/ros.h>
#include "Eigen/Dense"

#include <viewer.h>
#include "rasm/MeshFile.h"
#include "rasm/RASM_MAP_MSG.h"

#include "read_triangle_mesh.h"

#define MESH_MAX_GLOBAL_FACES 1000
#define MESH_CHANNELS 3

/************************************
 * Mesh class
 ***********************************/
class Mesh {
    public:
    
    /*Mesh data struct*/
    MOONRANGER_Mesh_t data;
    
    /**
     * Constructor
     */
    Mesh() {
        memset(&data.Vertices, 0, MESH_MAX_GLOBAL_FACES * MESH_CHANNELS * sizeof(float));
        data.numVertices = 0;
        
        memset(&data.Faces, 0, MESH_MAX_GLOBAL_FACES * MESH_CHANNELS * sizeof(int));
        data.numFaces = 0;

        memset(&data.MeshPose, 0, sizeof(MOONRANGER_Pose_t));
        data.MeshPose.w_quat = 1;
        
        memset(&data.Timestamp, 0, sizeof(CFE_TIME_SysTime_t));
        data.MeshId = 0;
        data.AccumFlag = false;
        data.isTriangulated = false;
    }
};/* end Mesh class */

void loadData(std::string filename, Mesh* mesh, int dims);

bool mesh_viewer (rasm::MeshFile::Request &req,
                    rasm::MeshFile::Response &res);