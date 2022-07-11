#ifndef PCL_VIEWER_H
#define PCL_VIEWER_H

void usage(const char *name);

// Converts received TMAPs to PCL meshes
void transferTMAP(const RASM::mesh &_mesh, unsigned int ind);

// Handles receiving of new tmaps over IPC
void transferTMAP();

/* prepares for receiving tmaps */
void init_tmap(const char** argv, int argc);

/* check on tmap data if needed */
void idle_tmap();

// Handles keypress registration
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

// Handles the drawing of the current scene and refreshes the display
void updateScene();

int main(int argc, char** argv);

#endif
