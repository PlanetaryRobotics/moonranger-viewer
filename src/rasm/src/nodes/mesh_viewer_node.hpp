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

typedef uint32_t uint32;
typedef double float64;

/******************************************************************************/
/*                  Type definition (MOONRANGER pose)
/******************************************************************************/

typedef struct {
  CFE_TIME_SysTime_t  timeStamp; 
  float64             x_pos;     // meters
  float64             y_pos;     // meters
  float64             z_pos;     // meters
  float64             x_quat;    // quaternion
  float64             y_quat;    // quaternion
  float64             z_quat;    // quaternion
  float64             w_quat;    // quaternion
  float64             covariance[36];
} MOONRANGER_Pose_t;

/******************************************************************************/
/*                           Terrain Mesh
/******************************************************************************/

#define MESH_MAX_GLOBAL_FACES 100000 // max number of faces allowed in mesh
#define MESH_CHANNELS 3 // number of channels per mesh container

/**
 * @brief Mesh type to be used internally in an app
 */
typedef struct {

    // vertices
    float Vertices[MESH_MAX_GLOBAL_FACES][MESH_CHANNELS];
    uint32 numVertices;
    
    // faces
    int Faces[MESH_MAX_GLOBAL_FACES][MESH_CHANNELS];
    uint32 numFaces;
    bool isTriangulated = false;

    // timestamp
    CFE_TIME_SysTime_t Timestamp;

    // associated pose (0s for world meshes)
    MOONRANGER_Pose_t MeshPose;
    
    // mesh counter
    uint32 MeshId = 0;
    
    // flag if world mesh (defaults to not a world mesh)
    bool AccumFlag = false;

} MOONRANGER_Mesh_t;

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