/*
 * Declaration of all global data
 * Also declares global_parseArgs which handles most of the options
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUD_GLOBALS_H
#define PLOT_CLOUD_GLOBALS_H

#include "helpers.h"
#include "camera.h"
#include <vector>

#include "plotClouds_color.h"
#include "plotClouds_file.h"
#include "plotClouds_path.h"

#include "wrl.h"
#include "shadowModel.h"
#include  "glLighting.h"

#include <comms_interface.h>
#include <tmap_full.h>

/* attempts to parse arguments starting at position i
 * returns the number of arguments consumed
 * returns 0 if the i'th parameter is not related
 */
unsigned int parseArgs_globals(unsigned int argc, char **argv,
			       unsigned int i);

void usage_globals();

/* handles a keypress, calls glutPostRedisplay() and returns true
 * returns false if the key is unrelated
 */
bool key_globals(unsigned char key,int x, int y);

/* cleanup functions */
void freedata();
void freemem();

void cycleDrawing();

extern bool useLighting;
extern lights *lightTable;

extern int verbose;

extern float maxDeviationsLong, maxDeviationsShort;
extern RADIANS minAngle;
extern float scaleX, scaleY, scaleZ;
extern char *singleFrameFile;

/* animation parameters */
extern int numFrames;
extern const char *frameName;
extern int final_target_set;
extern float final_cam_x, final_cam_y, final_cam_z;
extern int final_camera_set;
extern float final_cam_az, final_cam_el, final_cam_dist;

extern int widescreen;
extern int drawPathMode;
extern int drawMode;
extern bool useShadows;//note, this must be enabled BEFORE loading any files
extern float shadowX, shadowY, shadowZ;
extern bool showAxes;
extern bool firstIsWorld;
extern bool shouldDrawCloud[9];
extern bool shouldDrawPath[9];
extern int specialDrawMode[9];//-1 means no special mode
extern std::vector<bool> isTracks;
extern std::vector<METERS> pointSizes;
extern SimpleCamera *cam;

#if ENABLE_STEREO
extern unsigned int stereo_index;/* point cloud in clouds[stereo_index] */

/* data for live stereo acquisition */
extern struct xyz3f *stereoCloud;
extern struct timeval stereoTV;
extern unsigned char *stereoImgL;
extern unsigned char *stereoImgR;
extern unsigned char *stereoRectL;
extern unsigned char *stereoRectR;
extern unsigned char *stereoDisp;
extern int stereoNum;
extern unsigned int stereoWidth;
extern unsigned int stereoHeight;
extern unsigned int stereoBpp;
extern unsigned int stereoL;
extern unsigned int stereoR;
extern int stereoFlip;
extern int stereoReplay;
extern double stereoMountRoll;
extern double stereoMountPitch;
extern double stereoMountYaw;
extern double stereoMountX;
extern double stereoMountY;
extern double stereoMountZ;
#endif

#if ENABLE_TMAP
extern bool tmap_subscribe;
extern bool tmap_pose;
extern unsigned int tmap_index;
/* clouds[tmap_index+0] - world model
 * clouds[tmap_index+1] - path
 * clouds[tmap_index+2] - selected arc
 */
extern RASM::CommsInterface *ipc;
#endif

/* includes and variables for offscreen rendering */
/*
#include <X11/Xutil.h>
#include <GL/glx.h>
extern Display *Xdisplay;
extern GLXPixmap Xglpixmap;
extern Pixmap Xpixmap;
extern GLXContext Xcontext;
extern XVisualInfo *Xvi;
*/

extern int selectedMesh, selectedTriangle;//-1 means nothing selected

extern std::vector<color *> pathColors;

extern bool checkReload;

extern std::vector<loadedFile *>fileList;
extern std::vector<triangles *> triangleMeshes;
extern std::vector<wrl *> wrls;
extern std::vector<shadowModel *> shadows;
extern std::vector<float *> meshCosts;
extern std::vector<float *> meshPathCosts;
extern std::vector<points *> clouds;
extern std::vector<color *> colors;
extern point mean;
extern std::vector<path*> paths;

#endif
