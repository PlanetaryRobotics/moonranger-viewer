#include "plotClouds_stereo.h"
#include "plotClouds_globals.h"

#include <stdlib.h>
#include <strings.h>

#include <firewireOpenCV.h>

void usage_stereo(){
  printf("\nStereo options:\n");
  printf("  --stereo width height bpp idL idR     acquire stereo data\n");
  printf("  --stereoMount roll pitch yaw  X Y Z   camera mounting parameters\n");
  printf("  --stereoReplay                        read from files specified over IPC\n");
}

extern void usage(const char *name);

unsigned int parseArgs_stereo(unsigned int argc, char **argv,
			      unsigned int i){

  if(!strcasecmp("--stereoReplay", argv[i])){
    stereoReplay=true;
    return 1;
  }

  if(!strcasecmp("--stereoMount", argv[i])){
    if(i+7 >= argc){
      fprintf(stderr, "Insuffient parameters to --stereoMount\n");
      usage(argv[0]); 
    }
    stereoMountRoll  = atof(argv[++i])*M_PI/180.0;
    stereoMountPitch = atof(argv[++i])*M_PI/180.0;
    stereoMountYaw   = atof(argv[++i])*M_PI/180.0;
    stereoMountX = atof(argv[++i]);
    stereoMountY = atof(argv[++i]);
    stereoMountZ = atof(argv[++i]);
    printf("Mounted at x/y/z=%0.2f %0.2f %0.2f r/p/y=%0.1f %0.1f %0.1fdeg\n",
	   stereoMountX, stereoMountY, stereoMountZ,
	   stereoMountRoll*180.0/M_PI, 
	   stereoMountPitch*180.0/M_PI, 
	   stereoMountYaw*180.0/M_PI);
    return 7;
  }


  if(!strcasecmp("--stereo", argv[i])){
    if(i+5 >= argc){
      fprintf(stderr, "Insuffient parameters to --stereo\n");
      usage(argv[0]); 
    }
    int sw = atoi(argv[++i]);
    int sh = atoi(argv[++i]);
    if((sw < -1) && (sh < -1)){
      stereoFlip = 1;
      sw = -sw;
      sh = -sh;
    }
    stereoWidth = (unsigned int)sw;
    stereoHeight = (unsigned int)sh;
 
    int sb = atoi(argv[++i]);
    if((1 != sb) && (3 != sb)){
      fprintf(stderr, "invalid bpp to --stereo\n");
      usage(argv[0]);
    }
    stereoBpp = (unsigned int)sb;
 
    stereoL = strtol(argv[++i], NULL, 0);
    stereoR = strtol(argv[++i], NULL, 0);
    if((0 == stereoL) || (0 == stereoR)){
      fprintf(stderr, "invalid camera IDs to --stereo\n");
      usage(argv[0]); 
    }
    if((stereoWidth<=0) || (stereoHeight<=0)){
      fprintf(stderr, "invalid dimensions to --stereo\n");
      usage(argv[0]); 
    }
    printf("Using stereo... camera IDs: %d %d at %dx%d flip=%d\n",
	   stereoL, stereoR, stereoWidth, stereoHeight, stereoFlip);
    return 6;
  }


  return 0;
}


static void rotImage(unsigned char *buf, unsigned int W, unsigned int H,
		     unsigned int bpp){
  for(unsigned int i=0;i<W;i++){
    for(unsigned int j=0;j<H/2;j++){
      /* get two opposing row,col pixel locations */
      unsigned int r1=j, r2 = (H-1)-j;
      unsigned int c1=i, c2 = (W-1)-i;
      unsigned int ind1 = r1*W + c1;
      unsigned int ind2 = r2*W + c2;
      
      /* swap them */
      for(unsigned int k=0;k<bpp;k++){
	unsigned char v=buf[bpp*ind1+k];
	buf[bpp*ind1+k]=buf[bpp*ind2+k];
	buf[bpp*ind2+k] = v;
      }
    }
  }
}

void idle_stereo(){
  if(!stereoCloud)return;

  int n = firewireOpenCVPollThread(stereoCloud, &stereoTV,
				   stereoImgL,  stereoImgR,
				   stereoRectL, stereoRectR,
				   stereoDisp);

  /* no new stereo data */
  if(n<0)return;

  if(verbose)printf("Got new stereo data %d points\n", n);
      
  if(stereoFlip){
    rotImage(stereoImgL,  stereoWidth, stereoHeight, stereoBpp);
    rotImage(stereoImgR,  stereoWidth, stereoHeight, stereoBpp);
    rotImage(stereoRectL, stereoWidth, stereoHeight, 1);
    rotImage(stereoRectR, stereoWidth, stereoHeight, 1);
    rotImage(stereoDisp,  stereoWidth, stereoHeight, 1);
  }
  
  /* clear the old cloud */
  delete clouds[stereo_index];
  
  /* add the new one */
  points *p = new points();
  for(int i=0;i<stereoNum;i++)
    p->add(point(stereoMountX + stereoCloud[i].x,
		 stereoMountY + stereoCloud[i].y,
		 stereoMountZ + stereoCloud[i].z));
  clouds[stereo_index] = p;
  
#if 0
  delete triangleMeshes[stereo_index];
  triangleMeshes[stereo_index] = createTriangles(p);
#endif
    
  stereoNum=n;
  glutPostRedisplay();
}


static unsigned char meanN(const unsigned char *arr, unsigned int len){
  unsigned int sum=0;
  for(unsigned int i=0;i<len;i++)
    sum += ((unsigned int)arr[i])&0xFF;
  return (unsigned char)(sum/len)&0xFF;
}


/* given an RGB image,
 * create a greyscale image showing the amount of color at each pixel
 */
extern void rgb_to_hsv(float r, float g, float b, float &h, float &s, float &v);
static void RGBtoCol(unsigned int width, unsigned int height,
                     const unsigned char *rgb, unsigned char *output){
  for(unsigned int i=0;i<width*height;i++){
    int r = ((int)rgb[3*i + 0])&0xFF;
    int g = ((int)rgb[3*i + 1])&0xFF;
    int b = ((int)rgb[3*i + 2])&0xFF;

    float h=0.0, s=0.0, v=0.0;
    rgb_to_hsv(((float)r)/255.0, ((float)g)/255.0, ((float)b)/255.0, h,s,v);
    if( (! (isfinite(h) && (h>=0.0) && (h<=360.0)) ) ||
        (! (isfinite(s) && (s>=0.0) && (s<=  1.0)) ) ||
        (! (isfinite(v) && (v>=0.0) && (v<=  1.0)) ) ){
      printf("Error, rgb %d %d %d -> hsv %0.1f %0.1f %0.1f\n", r, g, b, h, s, v);
      abort();
    }

    float blueness = 1.0 - fabs(h-240.0)/120.0;
    if(h<=120.0)blueness=0.0;
    assert((blueness>=0.0) && (blueness<=1.0));

    float res = s*(1.0-blueness);

    /* 0.3 removes most of the mountains, 
     * 0.5 removes most of the flags, 
     * 0.7 removes most of the tractor 
     */
    if(res<0.4)res=0.0;
    res = res*10.0;
    if(res<0.0)res=0.0;
    if(res>1.0)res=1.0;
    unsigned int res_i = ((unsigned int)floor(res*255.0));
    output[i] = ((unsigned char)res_i)&0xFF;
  }

  /* stretch out the signal */
  unsigned char meanOutput=meanN(output, width*height);
  for(unsigned int i=0;i<width*height;i++){
    if(output[i]<=meanOutput*4)continue;

    if(output[i] >= 0xFF/4)
      output[i] = 0xFF;
    else
      output[i] = 4*output[i];
  }
}


/* given an image of this size and depth
 * display it on the canvas at this x,y position with this width and height
 * the canvas is scaled to a unit square, 0,0 and 1,1 are the corners
 */
static void setImage(unsigned int width, unsigned int height, unsigned int bpp,
		     unsigned char *data,
		     float bx, float by, float bw, float bh){
  if((1!=bpp) && (3!=bpp)){
    printf("Error, bad bpp %d\n", bpp);
    assert(0);
  }

  glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  bool showOriginal=false;

  if(3==bpp)showOriginal=false;

  /* handle greyscale images */
  if((!showOriginal) && (1==bpp))showOriginal=true;

  showOriginal=true;

  if(showOriginal){
    glTexImage2D(GL_TEXTURE_2D, 0/* base map */, GL_RGB/*output format*/, 
                 width, height, 0/* no border */,
                 ((3==bpp)?GL_RGB:GL_LUMINANCE)/*input format*/,
                 GL_UNSIGNED_BYTE, data);
  }else{
    /* process the image and display the result */
    unsigned char *nonGrey = (unsigned char *)malloc(width*height);
    assert(nonGrey);
    RGBtoCol(width, height, data, nonGrey);
    glTexImage2D(GL_TEXTURE_2D, 0/* base map */, GL_RGB/*output format*/, 
                 width, height, 0/* no border */,
                 GL_LUMINANCE,
                 GL_UNSIGNED_BYTE, nonGrey);
    free(nonGrey);
  }

  glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0.0, 1.0, 0.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0, 0.0); glVertex3f(bx,    by+bh, 0.0);
  glTexCoord2f(1.0, 0.0); glVertex3f(bx+bw, by+bh, 0.0);
  glTexCoord2f(1.0, 1.0); glVertex3f(bx+bw, by,    0.0);
  glTexCoord2f(0.0, 1.0); glVertex3f(bx,    by,    0.0);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}



void draw_stereo(int width, int height){
  if(!stereoCloud)return;

  assert((width>0) && (height>0));

  static int first=1;
  static GLuint texL=0, texR=0, texRL=0, texRR=0, texD=0;
  if(first){
    glGenTextures(1, &texL);
    glGenTextures(1, &texR);
    glGenTextures(1, &texRL);
    glGenTextures(1, &texRR);
    glGenTextures(1, &texD);
    first=0;
  }
  
  const float dispWidth=0.3, dispHeight=0.2;
  bool onlyLeftImage=true;
  
  if(onlyLeftImage){
    const float py = ((float)height)*dispHeight;
    const float px = py*4.0/3.0;
    const float aspectW = px/((float)width);
    glBindTexture(GL_TEXTURE_2D, texL);
    setImage(stereoWidth, stereoHeight,  stereoBpp,  stereoImgL, 0.0,         0.0, aspectW, dispHeight);
    glBindTexture(GL_TEXTURE_2D, texRL);
    setImage(stereoWidth, stereoHeight,          1, stereoRectL, aspectW,     0.0, aspectW, dispHeight);
    glBindTexture(GL_TEXTURE_2D, texD);
    setImage(stereoWidth, stereoHeight,          1,  stereoDisp, 2.0*aspectW, 0.0, aspectW, dispHeight);

  }else{
    /* show both images */
    glBindTexture(GL_TEXTURE_2D, texL);
    setImage(stereoWidth, stereoHeight,  stereoBpp, stereoImgL, 0.0, 0.0, dispWidth, dispHeight);
    glBindTexture(GL_TEXTURE_2D, texR);
    setImage(stereoWidth, stereoHeight,  stereoBpp, stereoImgR, dispWidth, 0.0, dispWidth, dispHeight);
    glBindTexture(GL_TEXTURE_2D, texRL);
    setImage(stereoWidth, stereoHeight, 1, stereoRectL, 0.0, dispHeight, dispWidth, dispHeight);
    glBindTexture(GL_TEXTURE_2D, texRR);
    setImage(stereoWidth, stereoHeight, 1, stereoRectR, dispWidth, dispHeight, dispWidth, dispHeight);
    glBindTexture(GL_TEXTURE_2D, texD);
    setImage(stereoWidth, stereoHeight, 1, stereoDisp,  2.0*dispWidth, 0.0, 
	     1.0-(2.0*dispWidth), 2.0*dispHeight);
  }
}

void init_stereo(){
  if(!(stereoL && stereoR))return;

  printf("Initializing stereo...\n");
  firewireOpenCVsaveOptions(0,0,0,0,0,0);/* don't save anything */
  int useSharedMem = 1;
  if(-1 == firewireOpenCVInit(stereoWidth, stereoHeight, stereoBpp,
			      stereoL, stereoR,
			      useSharedMem, stereoReplay,
			      NULL, NULL, 
			      stereoFlip, stereoMountRoll,
			      stereoMountPitch, stereoMountYaw)){
    fprintf(stderr, "Error, unable to initialize stereo\n");
    abort();
  }
  if(-1 == firewireOpenCVRunThread(1/* use sgbm instead of regular bm */)){
    fprintf(stderr, "Error, unable to start acquisition thread\n");
    abort();
  }
  printf("\nWaiting for first cloud...\n");
  const unsigned int N = stereoWidth*stereoHeight;
  stereoCloud = ( struct xyz3f *)malloc(N*sizeof(struct xyz3f) );
  stereoImgL  = (unsigned char *)malloc(N*stereoBpp*sizeof(unsigned char));
  stereoImgR  = (unsigned char *)malloc(N*stereoBpp*sizeof(unsigned char));
  stereoRectL = (unsigned char *)malloc(N*sizeof(unsigned char));
  stereoRectR = (unsigned char *)malloc(N*sizeof(unsigned char));
  stereoDisp  = (unsigned char *)malloc(N*sizeof(unsigned char));
  stereoNum=-1;
  for(int i=0;(stereoNum<0) && (i<100);i++){
    printf(".");
    fflush(NULL);
    usleep(100000);
    stereoNum = firewireOpenCVPollThread(stereoCloud, &stereoTV,
					 stereoImgL,  stereoImgR,
					 stereoRectL, stereoRectR,
					 stereoDisp);
  }
  if(-1 == stereoNum){
    printf(" Unable to get images...\n");
    abort();
  }

  printf(" Got first cloud with %d points\n", stereoNum);

  /* create a virtual file name */
  fileList.push_back(new loadedFile("live_stereo"));

  /* convert the cloud to a points container */
  points *p = new points();
  for(int i=0;i<stereoNum;i++)
    p->add(point(stereoMountX + stereoCloud[i].x,
		 stereoMountY + stereoCloud[i].y,
		 stereoMountZ + stereoCloud[i].z));
  
  /* triangulate */
  //triangles *t = createTriangles(p);
  triangles *t = new triangles();

  /* store the data as if it came from a file */
  stereo_index = clouds.size();
  clouds.push_back(p);
  triangleMeshes.push_back(t);
  isTracks.push_back(false);
}
