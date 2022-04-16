
#include "plotClouds_draw.h"
#include "plotClouds_path.h"
#include "plotClouds_color.h"
#include "plotClouds_light.h"

#include "plotClouds_globals.h"

#include "camera.h"
#include "helpers.h"
#include "wrl.h"
#include "shadowModel.h"
#include "solidContours.h"

#include <GL/gl.h>

extern void wrlsToShadows();

static void drawCloud(const points *p, METERS size, 
		      bool forceLargePoints=false);
static void drawTracks(const triangles *t, const points *p, METERS size);
static void drawSolidMesh(const triangles *t, const float *costs=NULL);
static void drawWireframe(const triangles *t);
static int drawSolidContours(const triangles *t, SECONDS duration);

void plotClouds_draw(METERS pointsize){
  glPushMatrix();
  glScalef(scaleX, scaleY, scaleZ);
  glDisable(GL_LIGHTING);


  /* draw unit axes at the origin */
  drawAxes(1.0, 0.0, 0.0,   0.0, 1.0, 0.0,   0.0, 0.0, 1.0);

  /* draw unit axes at the focal point (the means) */
  /*
   glPushMatrix();
   glTranslatef(mean.x, mean.y, mean.z);
   drawAxes(0.0, 1.0, 1.0,   1.0, 0.0, 1.0,   1.0, 1.0, 0.0);
   glPopMatrix();
   */

  /* draw each path set */
  glDisable(GL_LIGHTING);
  for(unsigned int i=0;i<paths.size();i++){
    triangles *mesh=NULL;
    float *costs=NULL;
    float *pathcosts=NULL;
    if(1){
      if(!firstIsWorld && (paths.size()<triangleMeshes.size()))
	mesh = triangleMeshes[i];
      else
	mesh = triangleMeshes[0];
      assert(mesh);
      
      if(i>=meshCosts.size()){
	costs = (float *)malloc(sizeof(float) * mesh->size());
	meshCosts.push_back(costs);
	for(unsigned int j=0;j<mesh->size();j++)costs[j]=-1;
	pathcosts = (float *)malloc(sizeof(float) * mesh->size());
	meshPathCosts.push_back(pathcosts);
	for(unsigned int j=0;j<mesh->size();j++)pathcosts[j]=-1;
      }
      costs = meshCosts[i];
      pathcosts = meshPathCosts[i];
    }
    assert(costs);
    assert(pathcosts);
    assert(paths[i]);
    paths[i]->generateHeights(mesh, costs, pathcosts);
    if((i<9 && !shouldDrawPath[i]) || 
       (0 == triangleMeshes.size()))
      continue;
    paths[i]->draw(i);
  }

  /* draw each cloud */
  for(unsigned int i=0;i<clouds.size();i++){
    if(!clouds[i])continue;
    if(pointSizes.size()>i)pointsize=(pointSizes[i]/500.0);
    
    if(i<9 && !shouldDrawCloud[i])continue;
    colors[i]->glColor();

    /* save drawing state before this cloud
     * if this cloud has special drawing rules, apply those
     */
    SECONDS maxContourTime=10.0;
    int savedDrawMode = drawMode;
    bool savedLighting = useLighting;
    bool forceLargePoints = false;
    if((i<9) && (specialDrawMode[i]>=0) && !isTracks[i]){
      maxContourTime=-1.0;/* disable */
      useLighting = (specialDrawMode[i]>=NUM_DRAW_MODES);
      drawMode = specialDrawMode[i]%NUM_DRAW_MODES;
      forceLargePoints = true;
    }

    if(useLighting){
      /* place 3 lights around the camera
	 light 0 is above the camera (lower elevation)
	 lights 1 and 2 are to the sides (adjusted azimuth)
      */
      METERS cd = cam->distance;
      METERS d = sin(cam->elevation*M_PI/180.0)*cd;
      lightData *l;
      
      l = lightTable->get(0);
      if(l->stillDefault){
	l->on = 1;
	l->pos[0] = cam->X - sin((0.0+cam->azimuth)*M_PI/180.0)*d;
	l->pos[1] = cam->Y - cos((0.0+cam->azimuth)*M_PI/180.0)*d;
	l->pos[2] = cam->Z + cos((-45.0+-10.0+cam->elevation)*M_PI/180.0)*cd;
      }
      
      l = lightTable->get(1);
      if(l->stillDefault){
	l->on = 0;
	l->pos[0] = cam->X - sin((140.0+cam->azimuth)*M_PI/180.0)*d;
	l->pos[1] = cam->Y - cos((140.0+cam->azimuth)*M_PI/180.0)*d;
	l->pos[2] = cam->Z - cos((-45.0+0.0+cam->elevation)*M_PI/180.0)*cd;
      }

      l = lightTable->get(2);
      if(l->stillDefault){
	l->on = 0;
	l->pos[0] = cam->X - sin((-140.0+cam->azimuth)*M_PI/180.0)*d;
	l->pos[1] = cam->Y - cos((-140.0+cam->azimuth)*M_PI/180.0)*d;
	l->pos[2] = cam->Z - cos((-45.0+0.0+cam->elevation)*M_PI/180.0)*cd;
      }
      
      lighting();
    }
    else glDisable(GL_LIGHTING);

    if(isTracks[i]){
      drawTracks(triangleMeshes[i], clouds[i], pointsize);
      continue;
    }

    /* dispatch the appropriate drawing routine for this cloud */
    switch(drawMode){
    case DRAW_MODE_POINT:
      drawCloud(clouds[i], pointsize, forceLargePoints);
      break;
    case DRAW_MODE_WIRE:
      drawWireframe(triangleMeshes[i]);
      break;
    case DRAW_MODE_POINT_WIRE:
      if(pointsize>0.000001){
	drawCloud(clouds[i], pointsize, forceLargePoints);
	if(forceLargePoints || clouds[i]->size()<10000)
	  drawWireframe(triangleMeshes[i]);
      }
      break;
    case DRAW_MODE_MESH:
      drawSolidMesh(triangleMeshes[i], (i<meshCosts.size())?meshCosts[i]:NULL);
      break;
    case DRAW_MODE_CONTOUR:
      if(-1 == drawSolidContours(triangleMeshes[i], maxContourTime)){
	printf("Drawing contours took too long, aborting\n");
	cycleDrawing();
	cycleDrawing();
	glPopMatrix();
	return;
      }
      break;
    }

    /* restore drawing state 
     * this may have been clobbered by special rules for this cloud
     */
    drawMode = savedDrawMode;
    useLighting = savedLighting;
    
    /* show the selected triangle 
     * (if there is a selected triangle and it is in this cloud
     */
    glDisable(GL_LIGHTING);
    if(selectedMesh==(int)i && selectedTriangle>=0){
      colors[i]->glInvertColor();
      glDisable(GL_DEPTH_TEST);
      triangleMeshes[selectedMesh]->get(selectedTriangle).draw();
      glEnable(GL_DEPTH_TEST);
    }
  }

  /* draw each wrl file */
  if(useLighting)
    lighting();
  else
    glDisable(GL_LIGHTING);

  glPushMatrix();
  glScalef(1.0/scaleX, 1.0/scaleY, 1.0/scaleZ);
  for(unsigned int i=0;i<wrls.size();i++)
    wrls[i]->draw();
  glPopMatrix();

  /* draw all shadows */
  if(useShadows){

    static int first=1;
    if(first){
      first=0;
      
      //convert all wrl models into shadow models
      wrlsToShadows();

      //determine which faces of every model are lit
      shadowModel::vertex shadowSource(shadowX, shadowY, shadowZ);
      float shadowDist = cam->distance*10.0;
      for(unsigned int i=0;i<shadows.size();i++)
	shadows[i]->setSource(shadowDist, shadowSource);
    }
    
    float nearPlane = cam->getNear();
    glColor4f(0,0,0,0.5);
    for(unsigned int i=0;i<shadows.size();i++)
      shadows[i]->shadow(nearPlane);
  }
  glPopMatrix();
}
/* end of draw function */

void drawAxes(float r1, float g1, float b1,
	      float r2, float g2, float b2,
	      float r3, float g3, float b3,
	      bool forceDisplay,
	      double scale){
  if(!forceDisplay && !showAxes)return;
  glLineWidth(3.0);
  glBegin(GL_LINES);
  glColor3f(r1, g1, b1);
  glVertex3f(0.0,0.0,0.0);glVertex3f(1.0*scale,0.0,0.0);
  glColor3f(r2, g2, b2);
  glVertex3f(0.0,0.0,0.0);glVertex3f(0.0,1.0*scale,0.0);
  glColor3f(r3, g3, b3);
  glVertex3f(0.0,0.0,0.0);glVertex3f(0.0,0.0,1.0*scale);
  glEnd();
  glLineWidth(1.0);
}

static void drawCloud(const points *p, METERS size, bool forceLargePoints){
  if(!forceLargePoints && (p->size() > 1000) && 
     (pointSizes.size()<=0 || size<0)){
    glBegin(GL_POINTS);
    for(unsigned int i=0;i<p->size();i++)
      p->get(i).glVertex();
    glEnd();
  }else{
    int res = 3;
    if(forceLargePoints || p->size() < 10000)res++;
    if(forceLargePoints || p->size() < 1000)res++;
    if(forceLargePoints || p->size() < 100)res++;

    GLUquadric *quad = gluNewQuadric();
    for(unsigned int i=0; i<p->size(); i++){
      glPushMatrix();
      glTranslatef(p->get(i).x, p->get(i).y, p->get(i).z);
      float red = 1.0 - p->get(i).m_confidence;
      float green = p->get(i).m_confidence;
      assert(red >= 0 && red <= 1.0);
      assert(green >= 0 && green <= 1.0);
      glColor3f(red, green, /*blue=*/0);
      gluSphere(quad, size * p->get(i).m_num_original_points, res, res);
      glPopMatrix();
    }
    gluDeleteQuadric(quad);
  }
}

static void drawTracks(const triangles *t, const points *p, METERS size){
  drawCloud(p, size);
  glDisable(GL_LIGHTING);
  //glDisable(GL_DEPTH_TEST);
  glLineWidth(2.0);
  if(t && (t->size()>2)){
    for(int j=0;j<2;j++){
      glBegin(GL_LINE_STRIP);
      point p = t->get(j).getA();
      p.glVertex();
      for(unsigned int i=j;i<t->size();i+=2){
	point p2 = t->get(i).getC();
	/* don't connect discontinuities */
	if(distSqBetween(p, p2) > (10.0*distSqBetween(t->get(i).getA(), p2))){
	  glEnd();
	  glBegin(GL_LINE_STRIP);
	}
	p = p2;
	p.glVertex();
      }
      glEnd();
    }
  }else{
    for(int j=0;j<2;j++){
      glBegin(GL_LINE_STRIP);
      for(unsigned int i=j;i<p->size();i+=2)
	p->get(i).glVertex();
      glEnd();
    }
  }
  glLineWidth(1.0);
  //glEnable(GL_DEPTH_TEST);
}

static void drawSolidMesh(const triangles *t, const float *costs){
  for(unsigned int i=0;i<t->size();i++){
#define COLOR_MESH_WITH_CELL_COST 1
#if COLOR_MESH_WITH_CELL_COST
    if(costs){
      if(costs[i]<0){
	glColor4f(0,0,1,1);
      }else{
	float p = (costs[i]-5.0)/(255.0-5.0);
	glColor4f(p,1.0-p,0,1);
      }
    }
#endif
    t->get(i).draw();
  }
}

static void drawWireframe(const triangles *t){
  for(unsigned int i=0;i<t->size();i++)
    t->get(i).drawWire();
}

static int drawSolidContours(const triangles *t, SECONDS duration){
  const METERS spacing=0.1;
  const double numContours=10.0;/* number of contours in a color scale */
  const int rainbowMode = 0;/* rainbow or grayscale? */
  SECONDS startTime = getCurTime();
  for(unsigned int i=0;i<t->size();i++){
    if(!singleFrameFile && duration>0 &&
       getCurTime() > startTime+duration)return -1;
    drawSolidContour(t->get(i).getLow(), t->get(i).getMid(),
		     t->get(i).getHigh(),
		     spacing, numContours*spacing, rainbowMode);
  }
  return 0;
}
