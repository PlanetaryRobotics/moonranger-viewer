/*
 * Dominic Jonak (dom@cmu.edu)
 * Jan 23, 2008
 *
 * Visualization tool for terrain meshes and evaluations.
 *
 * This program can read point clouds, meshes (.smf files) and 
 * terrain evaluations (.paths files). The various datasets can
 * be visualized as point clouds, wireframes, solid meshes and contours.
 * Point clouds are converted to meshes using Triangle 1.6 (see triangle.h).
 * Evaluations are specific to the farfield navigator; they show the optimal
 * paths to the goal as well as the terrain evaluations.
 *
 * Plotclouds is well-suited for rendering animations.  All camera parameters
 * and visualization properties can be set on the command line.  The
 * --singleFrame option will generate a single image and exit.
 *
 * It can also be used for realtime monitoring.  The --autoreload option
 * will refresh all datasets when necessary.
 */

#include  "plotClouds_globals.h"

#include "glObject.h"
#include <vector>
#include <unistd.h>
#include "smfUtil.h"
#include "spacial.h"
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <dlfcn.h>
#include <errno.h>

#include  "plotClouds_light.h"
#include  "plotClouds_path.h"
#include  "plotClouds_draw.h"
#include  "plotClouds_wrl.h"
#include  "plotClouds_selectTri.h"
#if ENABLE_TMAP
#include  "plotClouds_tmap.h"
#endif

#define ESCAPE_KEY 27

void wrlsToShadows();

void saveImage(int H, int W, const char *name){
  char *buf = new char[4*H*W];
  if(!buf){
    fprintf(stderr, "Out of memory?!?\n");
    return;
  }
  glReadPixels(0,0,W,H, GL_RGBA, GL_UNSIGNED_BYTE, buf);
  FILE *f = fopen(name, "w+");
  if(!f){
    fprintf(stderr, "Unable to save screenshot to %s\n", name);
  }else{
    printf("Saving screenshot to %s\n", name);
    fprintf(f, "P6 %d %d\n255\n", W, H);
    for(int i=0;i<H;i++){
      for(int j=0;j<W;j++){
        #define INDEX(r, c) ((W*(H-(1+r)))+(c))
        for(int k=0;k<3;k++)
          fputc(0xFF & (buf[4*INDEX(i,j)+k]), f);
        #undef INDEX
      }
    }
  }

  fclose(f);
  delete [] buf;
}

class cloudWindow: public GlutWindow {
protected:

  void takeScreenShot(const char *filename) const{
    saveImage(getHeight(), getWidth(), filename);
  }

public:

  SimpleCamera mycam;

  cloudWindow(int argc, char *argv[]):GlutWindow(argc, argv,
						 widescreen?1920/2:640,
						 widescreen?1080/2:480,
						 0,0){
    cam=&mycam;
    setCamera(&mycam);

    mycam.X=mean.x;
    mycam.Y=mean.y;
    mycam.Z=mean.z;
  }
  virtual ~cloudWindow(){}

  int key(unsigned char key,int x, int y){
    /* test which key... */
    switch(key){
    case 'q': case ESCAPE_KEY:
      exit(0);

    case ',':
      {
	double fov=cam->getFOV()-1;if(fov<5)fov=5;
	cam->setFOV(fov);
	reshape(width, height);
	printf("fov now %f degress\n", fov);
      }
      break;

    case '.':
      {
	double fov=cam->getFOV()+1;if(fov<90)fov=90;
	cam->setFOV(fov);
	reshape(width, height);
	printf("fov now %f degress\n", fov);
      }
      break;

    case 'p':
      {
	static int n=0;
	char buf[16];
	sprintf(buf, "scr%04d.ppm", ++n);
	takeScreenShot(buf);
      }
      break;

    case 'm':
      assert(cam);
      printf("--camAz %0.1f --camEl %0.1f --viewDist %0.1f --camLoc %0.1f %0.1f %0.1f\n", 
	     cam->azimuth, cam->elevation, cam->distance,
	     cam->X, cam->Y, cam->Z);
      break;

    default:

      /* unknown key, try the key_XXX functions to see if any other 
       * part of the code can handle this keypress
       */
      if(key_light  (key, x, y) ||
	 key_globals(key, x, y) ||
	 key_files  (key, x, y) ||
	 key_select (key, x, y) ||
	 key_color  (key, x, y) )return 1;

      /* no part of the code uses this key */
      return 0;
    }

    glutPostRedisplay();
    return 1;
  }

  void idle(void){
    if(checkReload && needReload()){
      printf("Reloading all data...\n");
      reloadFiles();
      glutPostRedisplay();
    }

    if(mycam.enableAmbient)
      glutPostRedisplay();

#if ENABLE_STEREO
    idle_stereo();
#endif

#if ENABLE_TMAP
    idle_tmap();
#endif

    usleep(10000);
  }

  /* takes a screenshot, or part of one 
   * returns true if this needs to be continued
   */
  int continueScreenShot(const char *file, int counter) const{
    /* just take a regular screenshot and call it a day */
    /*
    if(!(widescreen && !Xdisplay)){
      takeScreenShot(file);
      return 0;
    }
    */

    /* when taking a single image in widescreen,
     * cut the FOV in half and render the scene 4 times
     * also save a reference shot
     */
    assert(counter>=0 && counter<5);

    const char *ending[5] = {"0", "LL", "LR", "UL", "UR"};
    char buf[1024];
    sprintf(buf, "%s.%s.ppm", file, ending[counter]);
    takeScreenShot(buf);

    static float savedFOV = 0;
    if(0 == counter){
      savedFOV = cam->getFOV();
      cam->setFOV(savedFOV/2.0);//after the overview, cut the FOV
    }
    if(4 == counter)
      cam->setFOV(savedFOV);//after the last quadrant, restore the FOV

    //prepare for the next screenshot by updating the split
    //note that we reset the split to 0/0 after the last quadrant
    int splitX[5] ={-1,+1,-1,+1, 0};
    int splitY[5] ={-1,-1,+1,+1, 0};
    splitWindow(splitX[counter], splitY[counter]);

    //indicate if we're done or there are more quadrants
    return (4 != counter);
  }

  virtual void animate() const{
    /* check if we invoked for a single frame */
    if(singleFrameFile){
      static int counter=0;
      if(!continueScreenShot(singleFrameFile, counter++))
	exit(0);
      glutPostRedisplay();
      return;
    }

    /* check if we invoked for a full animation sequence */
    if(numFrames>0){
      static int counter=0;
      static int curFrame=0;

      /* store the initial camera parameters */
      static float init_cam_x = cam->X;
      static float init_cam_y = cam->Y;
      static float init_cam_z = cam->Z;
      static float init_cam_az = cam->azimuth;
      static float init_cam_el = cam->elevation;
      static float init_cam_dist = cam->distance;

      /* set the final parameters if needed */
      if(!final_target_set){
	final_cam_x = cam->X;
	final_cam_y = cam->Y;
	final_cam_z = cam->Z;
	final_target_set = 1;
      }
      if(!final_camera_set){
	final_cam_az = cam->azimuth;
	final_cam_el = cam->elevation;
	final_cam_dist = cam->distance;
	final_camera_set = 1;
      }

      char buf[100];
      sprintf(buf, frameName, curFrame);
      if(!continueScreenShot(buf, counter++)){
	//advance to next frame
	counter=0;
	curFrame++;

	//check if we're done
	if(curFrame>numFrames)exit(0);

	float p = ((float)curFrame)/((float)numFrames);
#define interp(p, a, b) ((a) + (p)*((b)-(a)))
	cam->setView(interp(p, init_cam_x, final_cam_x),
		     interp(p, init_cam_y, final_cam_y),
		     interp(p, init_cam_z, final_cam_z),
		     interp(p, init_cam_az, final_cam_az),
		     interp(p, init_cam_el, final_cam_el),
		     interp(p, init_cam_dist, final_cam_dist));
#undef interp
      }

      glutPostRedisplay();
      return;
    }
  }

  void draw(void){
    METERS pointsize = mycam.distance/500.0;

    plotClouds_draw(pointsize);

    if(mycam.enableAmbient)
      glutPostRedisplay();

#if ENABLE_STEREO
    draw_stereo(getWidth(), getHeight());
#endif

#if ENABLE_TMAP
    draw_tmap();
#endif

  }
};




/* sets up an off screen render context using glx pixmap
   apparently you still need to specify a rendering location though
   feel free to use NULL, :0.0, localhost:0.0, or a remote computer
*/
void offScreenRenderCleanup(){
  printf("Cleaning up\n");
  /*
  glXMakeCurrent(Xdisplay, None, NULL);
  glXDestroyGLXPixmap (Xdisplay, Xglpixmap);
  glXDestroyContext(Xdisplay, Xcontext);
  XFree(Xvi);
  XFreePixmap(Xdisplay, Xpixmap);
  XCloseDisplay(Xdisplay);
  */
}

void offScreenRender(char *location=NULL){
  if(!location)location = getenv("DISPLAY");
  assert(location);
  /*
  Xdisplay = XOpenDisplay(location);
  assert(Xdisplay);

  int screen = DefaultScreen(Xdisplay);
  Drawable d = RootWindow(Xdisplay, screen);
  int width = DisplayWidth(Xdisplay, screen);
  int height = DisplayHeight(Xdisplay, screen);
  int depth = DefaultDepth(Xdisplay, screen);
  printf("Rendering on \"%s\"; default screen %d is %dx%d and %d bit color\n", 
	 location, screen, width, height, depth);
  Xpixmap = XCreatePixmap(Xdisplay, d, width, height, depth);
  int attribList[2] = {GLX_RGBA, None};
  Xvi = glXChooseVisual(Xdisplay, screen, attribList);
  assert(Xvi);

  Xglpixmap = glXCreateGLXPixmap(Xdisplay, Xvi, Xpixmap);

  Xcontext = glXCreateContext(Xdisplay, Xvi, NULL, false);
  assert(glXMakeCurrent(Xdisplay, Xglpixmap, Xcontext));
  */

  atexit(offScreenRenderCleanup);
}

#define maxClipPlanes 6 /* openGL requires atleast this many */
int clipEnabled[maxClipPlanes]={0,0,0,0,0,0};
double clipPlanes[maxClipPlanes][4];


void wrlsToShadows(){
  for(unsigned int wrlIndex = 0;wrlIndex<wrls.size();wrlIndex++){
    wrl *w = wrls[wrlIndex];
    /* use the wrl model to create a shadow model */
    shadowModel *s = new shadowModel(w->getNumVertices(), w->getNumFaces());
    for(unsigned int i=0;i<w->getNumVertices();i++){
      s->v[i].x = w->getVertex(i).x;
      s->v[i].y = w->getVertex(i).y;
      s->v[i].z = w->getVertex(i).z;
    }

    for(unsigned int i=0;i<w->getNumFaces();i++){
      s->f[i].points[0] = w->getFace(i).x;
      s->f[i].points[1] = w->getFace(i).y;
      s->f[i].points[2] = w->getFace(i).z;
      s->f[i].neighbors[0]=-1;
      s->f[i].neighbors[1]=-1;
      s->f[i].neighbors[2]=-1;
    }
    s->computeNormals(0);
    s->computeNeighbors();
    shadows.push_back(s);
  }
}


void usage(const char *name){
  printf("Usage: %s filename [options]\n", name);
  printf("  Any unrecognized option is treated as a filename\n");
  printf("\nFile types and formats:\n");
  printf("  .smf    triangle mesh (each line is 'v X Y Z' or 'f I J K')\n");
  printf("  .paths  terrain eval  (each line is 'X Y X_2 Y_2 cell pathcost')\n");
  printf("  other   point cloud   (each line is 'X Y Z')\n");
  printf("\nRender options:\n");
  printf("All datasets and evaluations are drawn in the same mode.\n");
  printf("\nAnimation options:\n");
  printf("  --oneFrame file renders a single frame and saves a ppm image\n");
  printf("  --numFrames N     renders N frames\n");
  printf("  --frameName S     sets frame name (default \"frame.%%04d\"\n");
  printf("  --finalTarget x y z       sets the final camera target\n");
  printf("  --finalCamera az el dist  sets the final camera orientation\n");
  printf("  --white           set white background instead of black\n");
  printf("\nOpenGl options:\n");
  printf("  --fog           enable fog (that matches the background)\n");
  printf("  --clip a b c d  adds a clipping plane aX+bY+cZ+d>0 (6 are allowed)\n");
  printf("\nCamera options:\n");
  printf("The camera is initially focused on the mean of the datasets.\n");
  printf("The angles and distance specify the camera location from the focal point.\n");
  printf("  --camLoc X Y Z  sets the focal point\n");
  printf("  --camx X        sets only the x-component of the camera focal point\n");
  printf("  --camy Y        sets only the y-component of the camera focal point\n");
  printf("  --camz Z        sets only the z-component of the camera focal point\n");
  printf("  --camAz az      sets the camera azimuth (in degrees)\n");
  printf("  --camEl el      sets the camera elevation (in degrees)\n");
  printf("  --viewDist d    sets the camera viewing distance\n");
  printf("  --navCloud      sets the camera parameters for viewing a nav cloud\n");
  printf("  --ambientAz t r Enables ambient azimuth with period t and range r\n");
  printf("  --ambientEl t r Enables ambient elevation with period t and range r\n");

  printf("  --shadow X Y Z          render shadows from this point source\n");

  printf("\nMiscellaneous options:\n");
  printf("  --offscreen     when rendering a single frame, don't pop up a window\n");
  printf("  --widescreen    initialize window to 1920x1080 (instead of 640x480)\n");
  printf("      widescreen and oneFrame renders 4 quadrants that can be combined with:\n");
  printf("      convert \\( $i.U?.ppm +append \\) \\( $i.L?.ppm +append \\) -append $i.png\n");

  usage_color();
  usage_lighting();
  usage_wrl();
  usage_globals();
#if ENABLE_STEREO
  usage_stereo();
#endif
#if ENABLE_TMAP
  usage_tmap();
#endif


  exit(0);
}



int viewer(int argc, char **argv){

  initLights();

  /* camera position */
  METERS camx=1e10, camy=1e10, camz=1e10;
  double camAz = 45, camEl = 45;
  METERS viewDist = 1e10;

  double cam_ambientAzPeriod = 1.0, cam_ambientAzRange = 0.0;
  double cam_ambientElPeriod = 1.0, cam_ambientElRange = 0.0;
  bool cam_enableAmbient=false;

  bool navCloud=false;
  bool whiteBG=false;
  METERS farPlane = 1e10;
  METERS nearPlane = 1e10;
  bool useFog = 0;
  bool offscreen=0;

  if(argc == 1)usage(argv[0]);

  srand(time(NULL));

  /*
   * Parse command-line parameters
   */
  std::string comms_library_filename = "librasm_comms.so";

  /* load each point cloud */
  for(int i=1;i<argc;i++){

    unsigned int n=0;

    if(0==n)n = parseArgs_lighting(argc, argv, i);
    if(0==n)n = parseArgs_globals (argc, argv, i);
    if(0==n)n = parseArgs_color   (argc, argv, i);
    if(0==n)n = parseArgs_wrl     (argc, argv, i);
#if ENABLE_STEREO
    if(0==n)n = parseArgs_stereo  (argc, argv, i);
#endif
#if ENABLE_TMAP
    if(0==n)n = parseArgs_tmap    (argc, argv, i);
#endif

    if(n>0){
      i += (int)(n-1);
      continue;
    }


    if(!strcasecmp("-h", argv[i]) ||
       !strcasecmp("--help", argv[i])){
      usage(argv[0]);
      continue;
    }

    if(!strcasecmp("--deviationsLong", argv[i])){
      printf("Changed max deviations long to %f\n", (maxDeviationsLong = atof(argv[++i])));
      continue;
    }
    if(!strcasecmp("--deviationsShort", argv[i])){
      printf("Changed max deviations short to %f\n", (maxDeviationsShort = atof(argv[++i])));
      continue;

    }
    if(!strcasecmp("--minAngle", argv[i])){
      printf("Changed min angle to %f\n", (minAngle = atof(argv[++i])));
      continue;
    }

    /*** camera parameters ***/
    if(!strcasecmp("--navCloud", argv[i])){
      printf("Got navCloud\n");
      navCloud=true;
      camx = 0;
      camy = 4;
      camz = 0;
      camAz = 0;
      camEl = 65;
      viewDist = 10;
      continue;
    }

    if(!strcasecmp("--camAz", argv[i])){
      printf("Set camera azimuth to %f\n", (camAz = atof(argv[++i])));
      continue;
    }
    if(!strcasecmp("--camEl", argv[i])){
      printf("Set camera elevation to %f\n", (camEl = atof(argv[++i])));
      continue;
    }
    if(!strcasecmp("--viewDist", argv[i])){
      printf("Set camera view distance to %f\n", (viewDist = atof(argv[++i])));
      continue;
    }
    if(!strcasecmp("--camx", argv[i])){
      printf("Set camera x to %f\n", (camx = atof(argv[++i])));
      continue;
    }
    if(!strcasecmp("--camy", argv[i])){
      printf("Set camera y to %f\n", (camy = atof(argv[++i])));
      continue;
    }
    if(!strcasecmp("--camz", argv[i])){
      printf("Set camera z to %f\n", (camz = atof(argv[++i])));
      continue;
    }
    if(!strcasecmp("--camLoc", argv[i])){
      camx = atof(argv[++i]);
      camy = atof(argv[++i]);
      camz = atof(argv[++i]);
      printf("Set camera x,y,z to %f %f %f\n", camx, camy, camz);
      continue;
    }

    if(!strcasecmp("--ambientAz", argv[i])){
      cam_ambientAzPeriod = atof(argv[++i]);
      cam_ambientAzRange = atof(argv[++i])*M_PI/180.0;
      cam_enableAmbient = true;
      printf("Enabled ambient azimuth motion period %0.1fs range %0.1fdeg\n", 
             cam_ambientAzPeriod, cam_ambientAzRange*180.0/M_PI);
      continue;
    }
    if(!strcasecmp("--ambientEl", argv[i])){
      cam_ambientElPeriod = atof(argv[++i]);
      cam_ambientElRange = atof(argv[++i])*M_PI/180.0;
      cam_enableAmbient = true;
      printf("Enabled ambient elevation motion period %0.1fs range %0.1fdeg\n", 
             cam_ambientElPeriod, cam_ambientElRange*180.0/M_PI);
      continue;
    }

    /*** end of camera parameters ***/

    if(!strcasecmp("--clip", argv[i])){
      static int numClipPlanes=0;
      if(numClipPlanes>=maxClipPlanes){
	printf("Error, can only specify %d clipping planes\n", maxClipPlanes);
	exit(-1);
      }
      for(int j=0;j<4;j++)
	clipPlanes[numClipPlanes][j] = atof(argv[++i]);
      clipEnabled[numClipPlanes] = 1;

      printf("Set clip plane %d to %gX +%gY + %gZ + %g > 0\n", numClipPlanes,
	     clipPlanes[numClipPlanes][0], clipPlanes[numClipPlanes][1], 
	     clipPlanes[numClipPlanes][2], clipPlanes[numClipPlanes][3]);
      numClipPlanes++;
      continue;
    }

    if(!strcasecmp("--oneFrame", argv[i])){
      singleFrameFile=argv[++i];
      continue;
    }

    if(!strcasecmp("--numFrames", argv[i])){
      numFrames=atoi(argv[++i]);
      continue;
    }

    if(!strcasecmp("--frameName", argv[i])){
      frameName=argv[++i];
      continue;
    }

    if(!strcasecmp("--finalTarget", argv[i])){
      final_target_set = 1;
      final_cam_x = atof(argv[++i]);
      final_cam_y = atof(argv[++i]);
      final_cam_z = atof(argv[++i]);
      printf("Set final camera x,y,z to %f %f %f\n", 
	     final_cam_x, final_cam_y, final_cam_z);
      continue;
    }

    if(!strcasecmp("--finalCamera", argv[i])){
      final_camera_set = 1;
      final_cam_az = atof(argv[++i]);
      final_cam_el = atof(argv[++i]);
      final_cam_dist = atof(argv[++i]);
      printf("Set final camera az, el, dist to %f %f %f\n", 
	     final_cam_az, final_cam_el, final_cam_dist);
      continue;
    }

    /* use white background instead of black */
    if(!strcasecmp("--white", argv[i])){
      whiteBG=true;
      continue;
    }

    /* enable fog at this density */
    if(!strcasecmp("--fog", argv[i])){
      useFog = true;
      continue;
    }

    /* set widescreen mode */
    if(!strcasecmp("--widescreen", argv[i])){
      widescreen=1;
      continue;
    }

    /* render a single frame offscreen */
    if(!strcasecmp("--offscreen", argv[i])){
      offscreen=true;
      continue;
    }

    if(!strcasecmp("--shadow", argv[i])){
      if(i+3 >= argc){
	fprintf(stderr, "Insuffient parameters to --shadow\n");
	usage(argv[0]);	
      }
      if(!fileList.empty()){
	fprintf(stderr, "Error, shadows must be enabled before loading any files\n");
	exit(1);
      }
      useShadows = true;
      shadowX = atof(argv[++i]);
      shadowY = atof(argv[++i]);
      shadowZ = atof(argv[++i]);
      printf("Enabled shadows from %g, %g, %g\n",
	     shadowX, shadowY, shadowZ);
      continue;
    }

    /* this parameter is handled by the cloudWindow constructor,
     * passing it to glutInit()
     */
    if(!strcasecmp("-geometry", argv[i])){
      i++;
      continue;
    }

    if((strncmp(argv[i], "--comms", strlen("--comms")) == 0) &&
       (argc > i+1))
      {
	comms_library_filename = argv[i+1];
	i++;
	continue;
      }

    if((strncmp(argv[i], "--config", strlen("--config")) == 0) &&
       (argc > i+1))
      {
	// this param just gets passed to the CommsInterface, so ignore it here
	i++;
	continue;
      }
    
    /*** unknown parameter, must be a file to load ***/

    if(!fileExists(argv[i])){
      fprintf(stderr, "Error, %s (dataset %d, parameter %d) does not exist\n", 
	      argv[i], (int)fileList.size(), i);
      fprintf(stderr, "If this was an option, run \"%s --help\" for usage information.\n", argv[0]);
      exit(-1);
    }

    loadFile(argv[i]);
  }/* done parsing command line parameters */


  printf("[%s:%d] Command line was: %s ", __FILE__, __LINE__, argv[0]);
  for(int i=1; i < argc; i++)
    {
      printf("%s ", argv[i]);
    }
  printf("\n");
  
  printf("[%s:%d] Using comms library: %s\n",
	 __FILE__, __LINE__,
	 comms_library_filename.c_str());

/*
 * Functions to be implemented by deployment-specific code.
 */
RASM::CommsInterface* (*construct_comms_interface)(const char** argv, int argc);

  /*
   * Now actually load shared libraries.
   * TBD: replace this with a call to code in external that loads libs
   * in a platform independent way (think windows)
   */ 
  void* lib_handle = dlopen(comms_library_filename.c_str(), RTLD_LAZY);
  if(NULL == lib_handle) 
    {
      printf("[%s:%d] ERROR: cannot load %s\n",
	     __FILE__, __LINE__,
	     comms_library_filename.c_str());
      exit(-1);
    }
  construct_comms_interface = (RASM::CommsInterface* (*)(const char**, int))dlsym(lib_handle, "construct_comms_interface");
  if(NULL == construct_comms_interface)
    {
      printf("[%s:%d] ERROR: cannot load %s\n",
	     __FILE__, __LINE__, 
	     comms_library_filename.c_str());
      exit(-1);
    }

ipc = construct_comms_interface((const char**)argv, argc);

#if ENABLE_STEREO
  /* initialize stereo if needed */
  init_stereo();
#endif

#if ENABLE_TMAP
  /* initialize stereo if needed */
  init_tmap((const char**)argv, argc);
#endif

  /* compute the mean of each */
  points means;
  for(unsigned int i=0;i<clouds.size();i++){
    if(clouds[i] && (clouds[i]->size()>0))
      means.add(clouds[i]->mean());
  }

  /* compute the mean of all the point clouds
     this is where we move the camera to and draw the extra axes
   */
  mean = means.mean();
  if(camx>1e9)camx = mean.x;
  if(camy>1e9)camy = mean.y;
  if(camz>1e9)camz = mean.z;

  if(viewDist>1e9){
    METERS maxDistSq=0;
    for(unsigned int i=0;i<clouds.size();i++){
      if(!clouds[i])continue;
      for(unsigned int j=0;j<clouds[i]->size();j++){
	METERS distSq = distSqBetween(mean, clouds[i]->get(j));
	if(distSq > maxDistSq)maxDistSq = distSq;
      }
    }
    viewDist = sqrt(maxDistSq)*2.0;
  }
  if(farPlane>1e9)farPlane = viewDist*3.0;
  if(nearPlane>1e9)nearPlane = farPlane/50;

  /* create a distinct color for each cloud */
  createColors(0);

  atexit(&freemem);
  cloudWindow cw(argc, argv);

  if(whiteBG)glClearColor(1,1,1,1);

  if(useFog){
    /* set the fog color to make the backgound (ie, clear) color */
    GLfloat fogColor[4] = {0,0,0,1};
    if(whiteBG)fogColor[0] = fogColor[1] = fogColor[2] = 1.0;
    glFogfv(GL_FOG_COLOR, fogColor);

    //enable linear fog, parameters set in camera::setView()
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glEnable(GL_FOG);
  }

  cw.mycam.setView(camx, camy, camz, camAz,camEl, viewDist);
  if(cam_enableAmbient){
    cw.mycam.enableAmbient = 1;
    cw.mycam.ambientAzPeriod = cam_ambientAzPeriod;
    cw.mycam.ambientAzRange = cam_ambientAzRange;
    cw.mycam.ambientElPeriod = cam_ambientElPeriod;
    cw.mycam.ambientElRange = cam_ambientElRange;
  }

  for(int i=0;i<maxClipPlanes;i++){
    if(!clipEnabled[i])continue;
    glClipPlane(GL_CLIP_PLANE0+i, clipPlanes[i]);
    glEnable(GL_CLIP_PLANE0+i);
  }

  /* rendering off screen appears to cause the X server to leak memory...
     but we'll just ignore that for now
   */
  const bool ignoreMemoryLeak=true;
  if(offscreen){
    if(!ignoreMemoryLeak)
      printf("Not rendering offscreen: memory leak in X server\n");
    if(!singleFrameFile)
      printf("Not rendering offscreen: interactive (set an output file)\n");
  }

  if(singleFrameFile && ignoreMemoryLeak && offscreen){
    offScreenRender();
    cw.reshape(widescreen?1920:640,widescreen?1080:480);
    cw.draw();
    cw.display();
  }else{
    cw.draw();
    cw.run();
  }
}

/*
 * $Log: plotClouds.cc,v $
 * Revision 1.85  2010/10/12 21:03:51  dom
 * Added const qualifier
 *
 * Revision 1.84  2010/05/04 18:18:51  dom
 * Now fills in indices after triangulating, just like reading from a file.
 *
 * Revision 1.83  2010/04/06 17:33:03  dom
 * Added missing parameter in error message
 *
 * Revision 1.82  2010/03/17 14:50:21  dom
 * Added --setCloudDraw to force the previous cloud to keep the current drawing mode.  
 * If this option is used in point or pwire mode, then the cloud will always be rendered as large points.  
 * If this option is used in contour mode, the timeout is removed.  Also added some bounds checking and minor cleanup.
 *
 * Revision 1.81  2010/01/05 17:41:49  dom
 * Fixed some memory leaks/missing cleanup
 *
 * Revision 1.80  2010/01/05 15:49:28  dom
 * Added new parameters to drawSolidContour
 *
 * Revision 1.79  2009/03/17 14:51:07  dom
 * Added --offscreen
 *
 * Revision 1.78  2008/12/12 16:32:16  dom
 * Added --wrlTransR.  Prevent shadow generation if it won't be used to speed up file loading.
 *
 * Revision 1.77  2008/12/10 15:55:12  dom
 * Fixed print order of camera parameters.
 *
 *
 *
 */
