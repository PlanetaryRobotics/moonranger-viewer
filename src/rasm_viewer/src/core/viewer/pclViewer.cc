#include <stdio.h>
#include <iostream>
#include <dlfcn.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <comms_interface.h>
#include <tmap_full.h>
#include "pclViewer.h"

using namespace std;

// Free or locked camera
#define PCLVIEW_FREE 1
#define PCLVIEW_LOCKED 2

// Render mode
#define PCLVIEW_FILLED 1
#define PCLVIEW_WIRE 2
#define PCLVIEW_POINT 3

/* meshes[0] - world model
 * meshes[1] - path
 * meshes[2] - selected arc
 * meshes[3] - goal
 * meshes[4] - prior world model
 */
std::vector<pcl::PolygonMesh *> meshes;

// current pose of the rover
RASM::pose roverpose;
RASM::CommsInterface *ipc;

// Ideally we'd like to call this with the additional 'false' flag to avoid creating an interactor,
//  but it seg faults when spinOnce() is called. The interactor captures a lot of prime keystrokes
//  but often has problems (eg 'w' switches to wireframe, but just for one frame). 
pcl::visualization::PCLVisualizer *viewer (new pcl::visualization::PCLVisualizer ("RASM Viewer", true));
RASM::CommsInterface* (*construct_comms_interface)(const char** argv, int argc);

int frameNumber = 0;
int viewmode = PCLVIEW_FREE;
int rendermode = PCLVIEW_FILLED;
int showPrevMesh = 0;

// Keypress handler
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
	// Filled rendering
	if (event.getKeySym() == "v" && event.keyDown()) {
		rendermode = PCLVIEW_FILLED;
	}
	// Wireframe rendering
	else if (event.getKeySym() == "b" && event.keyDown()) {
		rendermode = PCLVIEW_WIRE;
	}
	// Point rendering
	else if (event.getKeySym() == "n" && event.keyDown()) {
		rendermode = PCLVIEW_POINT;
	}
	// Toggle fixed/free camera view
	else if (event.getKeySym() == "c" && event.keyDown()) {
		viewmode = (viewmode == PCLVIEW_LOCKED ? PCLVIEW_FREE : PCLVIEW_LOCKED);
	}
	// Save a screenshot
	else if (event.getKeySym() == "t" && event.keyDown()) {
		char name[100];
		sprintf(name, "model_%04d.png", frameNumber);
		printf("Saving screenshot: %s\n", name);
		viewer->saveScreenshot(name);
	}
	// q is filtered out somewhere else in the viewer code
	else if (event.getKeySym() == "x" && event.keyDown()) {
		viewer->close();
		exit(1);
	}
	// show the previous mesh
	else if (event.getKeySym() == "z" && event.keyDown()) {
		showPrevMesh = (showPrevMesh == 1 ? 0 : 1);
	}
}


// Converts a TMAP to PCL polygon mesh
void transferTMAP(const RASM::mesh &_mesh, unsigned int ind) {
	/* clear the old points and triangles */
	if (ind == 0) {
		delete meshes[4];
		meshes[4] = meshes[0];
	}
	else {
		delete meshes[ind];
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	/* add the new points */
	for(unsigned int i=0;i<_mesh.m_num_vertices;i++) {
		pcl::PointXYZ p;
		p.x = RASM_TO_METERS(_mesh.m_vertices[i].X());
		p.y = RASM_TO_METERS(_mesh.m_vertices[i].Y());
		p.z = RASM_TO_METERS(_mesh.m_vertices[i].Z());
		cloud->points.push_back(p);
	}

	cloud->width = (int) cloud->points.size();
	cloud->height = 1;

	/* add the new triangles */
	std::vector<pcl::Vertices> polygons;
	for(unsigned int i=0;i<_mesh.m_num_faces;i++) {
		pcl::Vertices v;
		v.vertices.push_back(_mesh.m_faces[i].points[0]);
		v.vertices.push_back(_mesh.m_faces[i].points[1]);
		v.vertices.push_back(_mesh.m_faces[i].points[2]);
		polygons.push_back(v);
	}

	pcl::PolygonMesh *mesh = new pcl::PolygonMesh();
	sensor_msgs::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	mesh->cloud = msg;
	mesh->polygons = polygons;

	meshes[ind] = mesh;
}

// Transfers input TMAPs to the correct meshes[] index
void transferTMAP() {
	//	TMAP::tmap mesh;
	RASM::mesh _mesh;
	double dummy=0.0;
	unsigned int seq=0;

	/* check for new world model */
	if(ipc->have_new_map(RASM::MapType::WORLD))	{
		ipc->get_map(RASM::MapType::WORLD, seq, _mesh, dummy);
		printf("Got new world model sequence %d with %d points/%d triangles\n",
			seq, _mesh.m_num_vertices, _mesh.m_num_faces);
		transferTMAP(_mesh, 0);
	}

	 /* check for new rover path */
	if(ipc->have_new_map(RASM::MapType::PATH)) {
		ipc->get_map(RASM::MapType::PATH, seq, _mesh, dummy);
		printf("Got new rover path sequence %d with %d points/%d triangles\n",
			 seq, _mesh.m_num_vertices, _mesh.m_num_faces);
		transferTMAP(_mesh, 1);
	}
	
	/* check for new selected arc */
	if(ipc->have_new_map(RASM::MapType::ARC)) {
		ipc->get_map(RASM::MapType::ARC, seq, _mesh, dummy);
		printf("Got new arc sequence %d with %d points/%d triangles\n",
			seq, _mesh.m_num_vertices, _mesh.m_num_faces);
		transferTMAP(_mesh, 2);
	}

	/* check for goal */
	ipc->pause(0);
	RASM::goal g;	
	ipc->update_goal(g);

	printf("[%s:%d] Goal (%0.1f %0.1f) with semimajor/minor axes %0.1f, %0.1f and orientation %0.1f degrees\n",
		__FILE__, __LINE__,
		RASM_TO_METERS(g.m_position.X()),
		RASM_TO_METERS(g.m_position.Y()),
		g.m_semimajor_axis,
		g.m_semiminor_axis,
		g.m_orientation_radians * 180.0/M_PI);
	
	unsigned int v = 0;
	unsigned int steps = 72;
	
	RASM::mesh goal_mesh;
	goal_mesh.m_num_vertices = steps;
	goal_mesh.m_vertices = new RASM::point3d[goal_mesh.m_num_vertices];
	
	// Draw goal origin
	//	goal_mesh.m_vertices[v++].set(g.m_position.X(),g.m_position.Y(),METERS_TO_RASM(4));
	//	goal_mesh.m_vertices[v++].set(g.m_position.X(),g.m_position.Y(),METERS_TO_RASM(0));
	
	double beta = -g.m_orientation_radians; 
	double sinbeta = sin(beta);
	double cosbeta = cos(beta);
	
	double a = METERS_TO_RASM(g.m_semimajor_axis);
	double b = METERS_TO_RASM(g.m_semiminor_axis);
	
	for (int i = 0; i < 360; i += 360/steps) {
		double alpha = i * (M_PI / 180) ;
		double sinalpha = sin(alpha);
		double cosalpha = cos(alpha);
	
		double px = g.m_position.X() + (a * cosalpha * cosbeta - b * sinalpha * sinbeta);
		double py = g.m_position.Y() + (a * cosalpha * sinbeta + b * sinalpha * cosbeta);
	
		goal_mesh.m_vertices[v++].set(px,py,0.0);
	}

	//printf("v=%d, num_vertices=%d\n", v, goal_mesh.m_num_vertices);
	//assert(v == goal_mesh.m_num_vertices);
	
	goal_mesh.m_num_vertices = v;
	goal_mesh.m_num_faces = 0;
	transferTMAP(goal_mesh, 3);
}

void idle_tmap() {
	double t = 0.0;
	if(roverpose.isValid())t=roverpose.getTime();

	ipc->pause(1);

	/* check if new tmapData arrived */
	if (ipc->have_new_map(RASM::MapType::WORLD) ||
		   ipc->have_new_map(RASM::MapType::PATH) ||
		   ipc->have_new_map(RASM::MapType::ARC)) {
		int isworld = ipc->have_new_map(RASM::MapType::WORLD);
		transferTMAP();
		updateScene();

		if (isworld) frameNumber++;
	}

	/* check if new pose data arrived */
	ipc->update_pose(roverpose);

	if(roverpose.getTime() > (t+1e-5)) {
		updateScene();
	}
}

void init_tmap(const char** argv, int argc){
	printf("Initializing tmap...\n");
	
	assert(!ipc);
	ipc = construct_comms_interface(argv, argc);
	assert(ipc);

	ipc->subscribe_to_map();
	ipc->subscribe_to_goal_cmds();

	printf("Waiting for first tmap...\n");
	while(0 == meshes[0]->polygons.size()) {
		if(ipc->have_new_map(RASM::MapType::WORLD)) // ||
	//		 ipc->have_new_map(RASM::MapType::PATH)) ||
	//		 ipc->have_new_map(RASM::MapType::ARC))
		{
			transferTMAP();
			updateScene();
			frameNumber++;
		}
		else {
			ipc->pause(100);
		}
	}
}

void usage(const char *name) {
	printf("PCL viewer for RASM. Allows the loading of meshes from file or over IPC.\n\n");
	printf("Usage: %s filename [options]\n", name);
	printf("  -p <folder>          Loads mesh files from given folder. Assumes pose data is\n");
	printf("                        in first line of file. Currently only works with Ames logs.\n");
	printf("  --comms <comms lib>  Comms library name.\n");

	printf("\nKeyboard commands within the viewer:\n");
	printf("  TBD\n");

	printf("\n");
}


// Main graphics loop handling rendering of text and maps
void updateScene() {
	RASM::point3d xyz;
	float roll, pitch, yaw;
	roverpose.get(xyz, roll, pitch, yaw);
	xyz.coord3d[0] = RASM_TO_METERS(xyz.coord3d[0]);
	xyz.coord3d[1] = RASM_TO_METERS(xyz.coord3d[1]);
	xyz.coord3d[2] = RASM_TO_METERS(xyz.coord3d[2]);

	// Calculate the xyzrpy transforms
	Eigen::Affine3f t;
	pcl::getTransformation (xyz.coord3d[0], xyz.coord3d[1], xyz.coord3d[2], roll, pitch, yaw, t);


	// if (frameNumber > 0)
	// 	viewer->removeShape("helpstr");

	// Write helper text above the pose
	if (frameNumber == 0) {
		char helpbuf[150];
		sprintf(helpbuf, "Press 'x' to exit, 'c' to toggle fixed/free camera, 'z' to show the previous mesh, 'v' for filled polygons, 'b' for wireframe, 'n' for points, 't' to take a screenshot.");
		string helpstr(helpbuf);
		viewer->addText(helpstr, 10, 10, 12, 1, 1, 1, "helpstr");
	}


	// Write the pose to the bottom of the screen
	char buf[150];
	sprintf(buf, "Pose: x: %0.3f, y: %0.3f, z: %0.3f, roll: %0.3f, pitch: %0.3f, yaw: %0.3f", 
		xyz.coord3d[0], xyz.coord3d[1], xyz.coord3d[2], roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
	string posestr(buf);

	if (frameNumber > 0)
		viewer->removeShape("posestr");
	viewer->addText(posestr, 10, 25, 14, 1, 1, 1, "posestr");

	// Add the prior mesh before the current mesh to show differences
	if (frameNumber > 1) {
		viewer->removePolygonMesh("prevmesh");
	}
	if (frameNumber > 0 && showPrevMesh) {
		viewer->addPolygonMesh(*meshes[4], "prevmesh");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.6, 1.0, "prevmesh");
	}

	// Draw the current world mesh
	if (frameNumber > 0) {
		viewer->removePolygonMesh("mesh");
	}
	viewer->addPolygonMesh(*meshes[0], "mesh");

	// Handle rendering mode. Converts everything currently in the scene to point/wireframe/surface. 
	//  Anything after this point seems to be unaffected. 
	if (rendermode == PCLVIEW_FILLED) {
		viewer->setRepresentationToSurfaceForAllActors();
	}
	else if (rendermode == PCLVIEW_WIRE) {
		viewer->setRepresentationToWireframeForAllActors();
	}
	else {
		viewer->setRepresentationToPointsForAllActors();
	}

	viewer->initCameraParameters();

	// origin
	//viewer->addCoordinateSystem(1.0);

	// Draw the rover position and orientation as an arrow
	pcl::PointXYZ arrow_point;
	arrow_point.x = 0.0;
	arrow_point.y = 0.0;
	arrow_point.z = 0.0;
	pcl::PointXYZ p1 = pcl::transformPoint(arrow_point, t);

	pcl::PointXYZ arrow_back;
	arrow_back.x = 0.0;
	arrow_back.y = -4.0;
	arrow_back.z = 0.0;
	pcl::PointXYZ p2 = pcl::transformPoint(arrow_back, t);

	if (frameNumber > 0)
		viewer->removeShape("rov");
	viewer->addArrow(p1, p2, 1.0, 0.0, 0.0, 0, "rov");

	// Transform the camera to align with the current pose	
	pcl::PointXYZ cam_origin;
	cam_origin.x = 0.0;
	cam_origin.y = -50.0;
	cam_origin.z = 40.0;
	Eigen::Affine3f t2;
	pcl::getTransformation (xyz.coord3d[0], xyz.coord3d[1], xyz.coord3d[2], 0, 0, yaw, t2);
	pcl::PointXYZ cam = pcl::transformPoint(cam_origin, t2);

	// If we're following the rover or we've just started, move the camera above and behind the rover position
	//	if (viewmode == PCLVIEW_LOCKED || frameNumber == 0) {
	if (viewmode == PCLVIEW_LOCKED) {
		viewer->camera_.view[0] = 0;
		viewer->camera_.view[1] = 0;
		viewer->camera_.view[2] = 1;

		viewer->camera_.pos[0] = cam.x;
		viewer->camera_.pos[1] = cam.y;
		viewer->camera_.pos[2] = cam.z;

		viewer->camera_.focal[0] = xyz.coord3d[0];
		viewer->camera_.focal[1] = xyz.coord3d[1];
		viewer->camera_.focal[2] = xyz.coord3d[2];

		viewer->camera_.clip[0] = 0.01;
		viewer->camera_.clip[1] = 500.0;
		viewer->updateCamera(); 
	}
	else if (frameNumber == 0) {
		viewer->camera_.view[0] = 0;
		viewer->camera_.view[1] = 0;
		viewer->camera_.view[2] = 1;

		viewer->camera_.pos[0] = cam_origin.x;
		viewer->camera_.pos[1] = cam_origin.y;
		viewer->camera_.pos[2] = cam_origin.z;

		viewer->camera_.focal[0] = xyz.coord3d[0];
		viewer->camera_.focal[1] = xyz.coord3d[1];
		viewer->camera_.focal[2] = xyz.coord3d[2];

		viewer->camera_.clip[0] = 0.01;
		viewer->camera_.clip[1] = 500.0;
		viewer->updateCamera(); 
	}

	viewer->spinOnce(100);
	sleep(0.1);
}

int main(int argc, char** argv) {
	string meshpath = "";
	string comms_library_filename = "librasm_comms.so";
	string meshformat = "/model.";

	bool usingIPC = false;

	for (int i = 1; i < argc; i++) {
		if(!strcasecmp("-h", argv[i]) || !strcasecmp("--help", argv[i])){
			usage(argv[0]);
			continue;
		}

		// Change this to use a file list, but might need some other edits as Ames logs need
		// a separate file for correct pose information. 
		if (!strcasecmp(argv[i], "-p") && (argc > i+1)) {
			meshpath = argv[i+1];
			printf("Loading meshes from folder: %s\n", meshpath.c_str());
			i++;
			continue;
		}

		if((strncmp(argv[i], "--comms", strlen("--comms")) == 0) && (argc > i+1)) {
			comms_library_filename = argv[i+1];
			printf("Using comms library: %s\n", comms_library_filename.c_str());
			usingIPC = true;
			i++;
			continue;
		}
	}

	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	// world model at index 0
	meshes.push_back(new pcl::PolygonMesh());

	// rover path at index 1 
	meshes.push_back(new pcl::PolygonMesh());

	// selected arc at index 2
	meshes.push_back(new pcl::PolygonMesh());

	// goal at index 3
	meshes.push_back(new pcl::PolygonMesh());

	// previous world model at index 4
	meshes.push_back(new pcl::PolygonMesh());


	if (usingIPC) {
		// Load shared libraries.
		void* lib_handle = dlopen(comms_library_filename.c_str(), RTLD_LAZY);
		if (NULL == lib_handle) {
			printf("[%s:%d] ERROR: cannot load library %s\n",
				 __FILE__, __LINE__,
				 comms_library_filename.c_str());
			exit(-1);
		}

		construct_comms_interface = (RASM::CommsInterface* (*)(const char**, int))dlsym(lib_handle, "construct_comms_interface");
		if (NULL == construct_comms_interface) {
			printf("[%s:%d] ERROR: cannot load comms interface %s\n",
				__FILE__, __LINE__, 
				comms_library_filename.c_str());
			exit(-1);
		}

		// init IPC
		init_tmap((const char**)argv, argc);

		while (1) {
			idle_tmap();
			usleep(1000);
		}
	}
	// Todo: Clean this up but keep it compatible with Ames logs
	else {
		float x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, x1=0.0, y1=0.0, z1=0.0;
		unsigned int p1=0, p2=0, p3=0;
		double timestamp=0.0;

		// *** Update later with meshformat command line arg like above
		string filestr = meshpath + "/model.";
		string posestr = meshpath + "/raw.RasmDdsSensor.";

		bool viewing = true;

		while (viewing) {
			char ind[4];
			sprintf(ind, "%04d", frameNumber);
			string accend = ".obj";
			string poseend = ".obj";

			string filename = filestr + ind + accend;

			FILE *f;
			printf("[%s:%d] Attempting to read %s\n",
				__FILE__, __LINE__, filename.c_str());

			f = fopen(filename.c_str(), "r");
			if (f == NULL) {
				printf("[%s:%d] ERROR: Reached end of file list\n",
					__FILE__, __LINE__);
				viewing = false;
				continue;
			}

			bool have_pose = false;
			int num_read = 0;
			char buf [1024];

			pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
			std::vector<pcl::Vertices> polygons;

			printf("Loading accumulated mesh data from %s\n", filename.c_str());

			while(fgets(buf,sizeof(buf),f) != NULL) {
				//If we don't have a pose line yet read until we get one
				if(!have_pose) {
					num_read = sscanf(buf,"# x %f m y %f m z %f m roll %f rad pitch %f rad yaw %f rad timestamp %lf\n",	&x, &y, &z, &roll, &pitch, &yaw, &timestamp);

					if(7 == num_read) {
						// printf("[%s:%d] read pose from file x %f m y %f m z %f m roll %f rad pitch %f rad yaw %f rad\n",	__FILE__, __LINE__,	x, y, z, roll, pitch, yaw);
						have_pose=true;
					}
				}
				else {
					if (buf[0] == 'v') {
						if (3 == sscanf(buf,"v %f %f %f\n",&x1, &y1, &z1)) {
							pcl::PointXYZ basic_point;
							basic_point.x = x1;
							basic_point.y = y1;
							basic_point.z = z1;
							basic_cloud_ptr->points.push_back(basic_point);
						}
					}
					else if (buf[0] == 'f') {
						if (3 == sscanf(buf,"f %u %u %u\n",&p1, &p2, &p3)) {
							pcl::Vertices v;
							v.vertices.push_back(p1-1);
							v.vertices.push_back(p2-1);
							v.vertices.push_back(p3-1);
							polygons.push_back(v);
						}
					}
				}
			}
			fclose(f);

			string posename = posestr + ind + poseend;
			printf("Loading pose data from %s\n", posename.c_str());

			have_pose=false;
			FILE *fpose;
			fpose = fopen(posename.c_str(), "r");
			if (fpose == NULL) {
				printf("Couldn't open pose file %s\n", posename.c_str());
				exit(-1);
			}

			while(fgets(buf,sizeof(buf),fpose) != NULL) {
				//If we don't have a pose line yet read until we get one
				if (!have_pose) {
					num_read = sscanf(buf,"# x %f m y %f m z %f m roll %f rad pitch %f rad yaw %f rad timestamp %lf\n",	&x, &y, &z, &roll, &pitch, &yaw, &timestamp);

					if(7 == num_read) {
						printf("Read pose from file x %f m y %f m z %f m roll %f rad pitch %f rad yaw %f rad\n", x, y, z, roll, pitch, yaw);
						have_pose = true;

						RASM::point3d xyz(x, y, z);
						roverpose.set(xyz, roll, pitch, yaw, timestamp);
					}
				}
			}

			fclose(fpose);

			basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size();
			basic_cloud_ptr->height = 1;

			// Store the previous mesh for display
			if (frameNumber > 0) 
				meshes[4] = meshes[0];

			pcl::PolygonMesh *mesh = new pcl::PolygonMesh();
			sensor_msgs::PointCloud2 msg;
			pcl::toROSMsg(*basic_cloud_ptr, msg);
			mesh->cloud = msg;
			mesh->polygons = polygons;
			meshes[0] = mesh;

			printf("Loaded %u points and %u triangles.\n", (unsigned int)basic_cloud_ptr->width, (unsigned int)polygons.size());

			updateScene();

			frameNumber++;
		} // End while (viewing)

		viewer->removePolygonMesh("prevmesh");
		viewer->addText("Log completed", 10, 45, 14, 1, 1, 1, "donestr");
		viewer->spin();
	}

	return 0;
} 

