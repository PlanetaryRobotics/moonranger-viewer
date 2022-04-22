/****************************************************************
 *
 * @file      mesh_viewer_node.cpp
 *
 * @brief     ROS Node for Moonranger mesh visualization
 *
 * @version   1.0
 * @date      04/20/22
 *
 * @authors   Ashwin Nehete, ...
 * @author    Carnegie Mellon University, Planetary Robotics Lab
 *
 ****************************************************************/

#include "mesh_viewer_node.hpp"

int flag = 0;
Mesh grid_mesh;

/**
 * \brief Load mesh data from a file
 * \param filename Path to file to load
 * \param mesh Pointer to mesh object to store results in
 * \param dims Number of dimensions of vertices
 */
void loadData(std::string filename, Mesh* mesh, int dims) {

    // Confirm dims is 2 or 3
    assert(dims == 2 || dims == 3);

    // Eigen matrices
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    // Load data from files into comparison arrays
    igl::read_triangle_mesh(filename, V, F);
    assert(V.rows() > 0);

    // Read into relevant global data structures
    mesh->data.numVertices = V.rows();
    mesh->data.numFaces = F.rows();
    mesh->data.isTriangulated = true;

    for (uint i = 0; i < V.rows(); i++) {
        mesh->data.Vertices[i][0] = V(i, 0);
        mesh->data.Vertices[i][1] = V(i, 1);
        if (dims == 3) mesh->data.Vertices[i][2] = V(i, 2);
    }

    for (uint i = 0; i < F.rows(); i++) {
        mesh->data.Faces[i][0] = F(i, 0);
        mesh->data.Faces[i][1] = F(i, 1);
        mesh->data.Faces[i][2] = F(i, 2);
    }

}

bool mesh_viewer (rasm::MeshFile::Request &req,
                    rasm::MeshFile::Response &res) {
    /*set flag to 1*/
    flag = 1;
    ROS_INFO("Mesh file received.");

    /*Load the mesh file*/
    loadData(req.path_to_mesh, &grid_mesh, 2);

    /*Publish the rasm::RASM_MAP_MSG type message*/

    // viewer(argc,argv);
    return true;

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mesh_viewer");
    ros::NodeHandle nh;

    ros::ServiceServer meshViewerService = nh.advertiseService("mesh_viewer_file", mesh_viewer);
    ROS_INFO("Ready for Moonranger Mesh Visualization.");

    ros::Rate r(1);
    while (flag != 1) {
        ROS_INFO("Waiting for mesh file service call");
        r.sleep();
        ros::spinOnce();
    }

    return 0;

}