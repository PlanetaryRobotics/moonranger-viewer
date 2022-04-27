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
#include "ros_interface.h"
#include "rasm_common_types.h"

#include "read_triangle_mesh.h"

// Include CFE time structure
extern "C" {
    #include "cfe_time.h"
    #include "common_types.h"
}

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

// void loadMoonrangerMeshData(std::string filename, Mesh* mesh, int dims);
void loadRasmMeshData(std::string filename);

bool mesh_viewer (rasm::MeshFile::Request &req,
                    rasm::MeshFile::Response &res);