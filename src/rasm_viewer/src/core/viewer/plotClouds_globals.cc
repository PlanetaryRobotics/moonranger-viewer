#include "plotClouds_globals.h"


/* for enums: */
#include "plotClouds_path.h"
#include "plotClouds_draw.h"

#include <strings.h>
#include <stdlib.h>

bool useLighting = true;
lights *lightTable=NULL;

int verbose=0;

float maxDeviationsLong=-1;
float maxDeviationsShort=-1;
RADIANS minAngle=-1;
float scaleX=1.0, scaleY=1.0, scaleZ=1.0;
char *singleFrameFile=NULL;

/* animation parameters */
int numFrames=-1;
const char *frameName = "frame.%04d";
int final_target_set=0;
float final_cam_x=0, final_cam_y=0, final_cam_z=0;
int final_camera_set=0;
float final_cam_az=0, final_cam_el=0, final_cam_dist=0;

int widescreen = 0;
int drawPathMode = DRAW_PATH_LINES;
int drawMode = DRAW_MODE_POINT;
bool useShadows = false;//note, this must be enabled BEFORE loading any files
float shadowX = 0, shadowY = 0, shadowZ = 0;
bool showAxes = true;
bool firstIsWorld = false;
bool shouldDrawCloud[9]={true,true,true,true,true,true,true,true,true};
bool shouldDrawPath[9]={true,true,true,true,true,true,true,true,true};
int specialDrawMode[9]={-1,-1,-1,-1,-1,-1,-1,-1,-1};
std::vector<bool> isTracks;
std::vector<METERS> pointSizes;
SimpleCamera *cam=NULL;

#if ENABLE_STEREO
unsigned int stereo_index=0;

/* data for live stereo acquisition */
struct xyz3f *stereoCloud=NULL;
struct timeval stereoTV;
unsigned char *stereoImgL  = NULL, *stereoImgR  = NULL;
unsigned char *stereoRectL = NULL, *stereoRectR = NULL;
unsigned char *stereoDisp  = NULL;
int stereoNum=-1;
unsigned int stereoWidth=0, stereoHeight=0, stereoBpp=0;
unsigned int stereoL=0, stereoR=0;/* camera IDs */
int stereoFlip=0, stereoReplay=0;
double stereoMountRoll=0.0, stereoMountPitch=0.0, stereoMountYaw=0.0;
double stereoMountX=0.0,  stereoMountY=0.0,  stereoMountZ=0.0;
#endif

#if ENABLE_TMAP
bool tmap_subscribe=false;
bool tmap_pose=false;
unsigned int tmap_index=0;
RASM::CommsInterface *ipc=NULL;
#endif

/*
Display *Xdisplay=NULL;
GLXPixmap Xglpixmap;
Pixmap Xpixmap;
GLXContext Xcontext;
XVisualInfo *Xvi;
*/

int selectedMesh = -1, selectedTriangle = -1;

std::vector<color *> pathColors;

bool checkReload=false;

std::vector<loadedFile *>fileList;
std::vector<triangles *> triangleMeshes;
std::vector<wrl *> wrls;
std::vector<shadowModel *> shadows;
std::vector<float *> meshCosts;
std::vector<float *> meshPathCosts;
std::vector<points *> clouds;
std::vector<color *> colors;
point mean(0,0,0);

std::vector<path*> paths;


#define freeVector(v) for(unsigned int i=0;i<(v).size();i++)delete (v)[i];v.clear()
void freedata(){
  freeVector(triangleMeshes);
  freeVector(wrls);
  freeVector(shadows);
  freeVector(meshCosts);
  freeVector(meshPathCosts);
  freeVector(clouds);
  freeVector(paths);
}

void freemem(){
  freedata();
  freeVector(colors);
  freeVector(pathColors);
  freeVector(fileList);
  delete lightTable;
}
#undef freeVector


void cycleDrawing(){
  useLighting = !useLighting;
  if(useLighting)drawMode++;
  if(drawMode>=NUM_DRAW_MODES)drawMode=0;
  glutPostRedisplay();
}

void usage_globals(){
  printf("  --point         point cloud (see also pointsize) (default)\n");
  printf("  --wire          wireframe\n");
  printf("  --pwire         wireframe and point cloud\n");
  printf("  --mesh          solid mesh\n");
  printf("  --contour       contour map\n");
  printf("  --pointSize     set the size of points in -point and -pwire\n");
  printf("                  (should be a function of viewing distance)\n");

  printf("  --markTracks    the last dataset is vehicle tracks\n");
  printf("                  (drawn as two separate lines in pwire mode)\n");
  printf("  --setCloudDraw  the last dataset is forced to the current draw mode\n");
  printf("  --firstIsWorld  first dataset is considered to be the world model\n");
  printf("                  (paths are snapped in altitude to lay on this model)\n");
  printf("  --autoReload    automatically reloads updated datasets\n");

  printf("  --pathPoint     evalutions are rendered as points\n");
  printf("  --pathLines     evalutions are rendered as fading lines (default)\n");
  printf("  --pathSolid     evalutions are rendered as solid lines\n");
  printf("  --noAxes        suppresses axes at the origin and mean point\n");
  printf("  -v              verbose output\n");

  printf("\nVisibility options:\n");
  printf("  --hideCloud n   suppresses the nth dataset (0 indexed)\n");
  printf("  --hidePath n    suppresses the nth evaluation (0 indexed)\n");
  printf("  --hideWRL n     suppresses the nth wrl model (0 indexed)\n");

  printf("  --scaleX X      sets the x-component scaling factor (default is 1)\n");
  printf("  --scaleY Y      sets the y-component scaling factor (default is 1)\n");
  printf("  --scaleZ Z      sets the z-component scaling factor (default is 1)\n");

}

unsigned int parseArgs_globals(unsigned int argc, char **argv,
			       unsigned int i){

  if(!strcasecmp("--markTracks", argv[i])){
    if(0 == isTracks.size()){
      printf("Error, specify --markTracks right after the data\n");
      exit(-1);
    }
    isTracks[isTracks.size()-1]=true;
    printf("Data %d is vehicle tracks\n", isTracks.size()-1);
    return 1;
  }

  if(!strcasecmp("--setCloudDraw", argv[i])){
    if(0 == clouds.size()){
      printf("Error, specify --setCloudDraw right after the data\n");
      exit(-1);
    }
    if(clouds.size()>=9){
      printf("Error, setCloudDraw only works on clouds 0-8\n");
      exit(-1);
    }
    specialDrawMode[clouds.size()-1]=drawMode + (useLighting?NUM_DRAW_MODES:0);
    printf("Set special draw mode for cloud %d\n", clouds.size()-1);
    return 1;
  }

  if(!strcasecmp("--firstIsWorld", argv[i])){
    printf("First cloud is world model...\n");
    firstIsWorld=true;
    return 1;
  }

  if(!strcasecmp("--autoReload", argv[i])){
    checkReload=true;
    return 1;
  }

  /* draw style for paths... */
  if(!strcasecmp("--pathPoint", argv[i])){
    printf("Evaluated terrain as points...\n");
    drawPathMode=DRAW_PATH_POINT;
    return 1;
  }

  if(!strcasecmp("--pathLines", argv[i])){
    printf("Evaluated terrain as paths...\n");
    drawPathMode=DRAW_PATH_LINES;
    return 1;
  }
  
  if(!strcasecmp("--pathSolid", argv[i])){
    printf("Evaluated terrain as solid path...\n");
    drawPathMode=DRAW_PATH_SOLID;
    return 1;
  }
  /* end of draw style for paths */

  if(!strcasecmp("--noAxes", argv[i])){
    printf("Surpressing axes\n");
    showAxes=false;
    return 1;
  }
   
  if(!strcasecmp("-v", argv[i])){
    printf("Changed verbose to %d\n", (verbose = !verbose));
    return 1;
  }

  /* start of visibility options */
  if(!strcasecmp("--hideCloud", argv[i])){
    int ind=atoi(argv[++i]);
    printf("Hiding cloud %d\n", ind);
    if(ind<0 || ind>=9){
      printf("Error, hideCloud requires index 0-8\n");
      exit(0);
    }
    shouldDrawCloud[ind]=false;
    return 2;
  }

  if(!strcasecmp("--hidePath", argv[i])){
    int ind=atoi(argv[++i]);
    printf("Hiding path %d\n", ind);
    if(ind<0 || ind>=9){
      printf("Error, hidePath requires index 0-8\n");
      exit(0);
    }
    shouldDrawPath[ind]=false;
    return 2;
  }
   
  if(!strcasecmp("--hideWRL", argv[i])){
    int index=atoi(argv[++i]);
    printf("Hiding wrl %d\n", index);
    wrls[index]->setVisible(false);
    return 2;
  }
  /* end of visibility options */

  /* start of scaling parameters */
  if(!strcasecmp("--scaleX", argv[i])){
    printf("Set X scaling to %f\n", (scaleX = atof(argv[++i])));
    return 2;
  }
  if(!strcasecmp("--scaleY", argv[i])){
    printf("Set Y scaling to %f\n", (scaleY = atof(argv[++i])));
    return 2;
  }
  if(!strcasecmp("--scaleZ", argv[i])){
    printf("Set Z scaling to %f\n", (scaleZ = atof(argv[++i])));
    return 2;
  }
  /* end of scaling parameters */

  /* start of draw style */
  if(!strcasecmp("--point",  argv[i])){drawMode=DRAW_MODE_POINT;     return 1;}
  if(!strcasecmp("--wire",   argv[i])){drawMode=DRAW_MODE_WIRE;      return 1;}
  if(!strcasecmp("--pwire",  argv[i])){drawMode=DRAW_MODE_POINT_WIRE;return 1;}
  if(!strcasecmp("--mesh",   argv[i])){drawMode=DRAW_MODE_MESH;      return 1;}
  if(!strcasecmp("--contour",argv[i])){drawMode=DRAW_MODE_CONTOUR;   return 1;}
  /* end of draw style */

  /* draw points this large */
  if(!strcasecmp("--pointSize", argv[i])){
    pointSizes.push_back(atof(argv[++i]));
    return 2;
  }

  return 0;
}

bool key_globals(unsigned char key,int x, int y){
  switch(key){
  case '1': shouldDrawCloud[0] = !shouldDrawCloud[0]; break;
  case '2': shouldDrawCloud[1] = !shouldDrawCloud[1]; break;
  case '3': shouldDrawCloud[2] = !shouldDrawCloud[2]; break;
  case '4': shouldDrawCloud[3] = !shouldDrawCloud[3]; break;
  case '5': shouldDrawCloud[4] = !shouldDrawCloud[4]; break;
  case '6': shouldDrawCloud[5] = !shouldDrawCloud[5]; break;
  case '7': shouldDrawCloud[6] = !shouldDrawCloud[6]; break;
  case '8': shouldDrawCloud[7] = !shouldDrawCloud[7]; break;
  case '9': shouldDrawCloud[8] = !shouldDrawCloud[8]; break;
  case '!': shouldDrawPath[0]  = !shouldDrawPath[0];  break;
  case '@': shouldDrawPath[1]  = !shouldDrawPath[1];  break;
  case '#': shouldDrawPath[2]  = !shouldDrawPath[2];  break;
  case '$': shouldDrawPath[3]  = !shouldDrawPath[3];  break;
  case '%': shouldDrawPath[4]  = !shouldDrawPath[4];  break;
  case '^': shouldDrawPath[5]  = !shouldDrawPath[5];  break;
  case '&': shouldDrawPath[6]  = !shouldDrawPath[6];  break;
  case '*': shouldDrawPath[7]  = !shouldDrawPath[7];  break;
  case '(': shouldDrawPath[8]  = !shouldDrawPath[8];  break;

  case 'd':
    cycleDrawing();
    break;

  default://unhandled key...
    return false;
  }

  glutPostRedisplay();
  return true;
}
