#include "plotClouds_selectTri.h"
#include "plotClouds_globals.h"

#include <GL/gl.h>
#include <GL/glu.h>

void findTri(const point &origin, const point &pointOnLine){
  METERS closestDistSq = -1;
  int prevMesh = selectedMesh;
  int prevTriangle = selectedTriangle;
  selectedMesh = selectedTriangle = -1;
  point p(0,0,0);
  for(unsigned int i=0;i<triangleMeshes.size();i++){
    if((i<9) && !shouldDrawCloud[i])continue;
    for(unsigned int j=0;j<triangleMeshes[i]->size();j++){
      if(triangleMeshes[i]->get(j).containsRay(origin, pointOnLine, p)){
	METERS dist = distSqBetween(p, origin);
	if(closestDistSq<0 || dist<closestDistSq){
	  closestDistSq = dist;
	  selectedMesh = (int)i;
	  selectedTriangle = (int)j;
	  /*
	    printf("potential triangle in mesh %d, index %d: distance: %g\n",
	    selectedMesh, selectedTriangle, closestDistSq);
	  */
	}
      }
    }
  }

  if(closestDistSq!=-1){
    if((selectedMesh == prevMesh) && (selectedTriangle == prevTriangle)){
      selectedMesh = selectedTriangle = -1;
    }else{
      printf("Closest triangle in mesh %d, index %d, distance, %g\n",
	     selectedMesh, selectedTriangle, sqrt(closestDistSq));
      triangleMeshes[selectedMesh]->get(selectedTriangle).print();
      if(selectedMesh < (int)paths.size())
	printf("Cell cost %f, path cost %f\n",
	       meshCosts[selectedMesh][selectedTriangle],
	       meshPathCosts[selectedMesh][selectedTriangle]);
    }
  }
}

void selectTriangle(int mouseX, int mouseY){
  GLdouble wx, wy, wz;/* point coordinates */
  
  /* data needed for gluUnProject() */
  GLdouble modelview[16];
  GLdouble projection[16];
  GLint viewport[4];

  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetIntegerv(GL_VIEWPORT, viewport);
  
  /* make sure the cursor is on the screen */
  if((mouseX < viewport[0]) || (mouseY < viewport[1]) ||
     (mouseX > viewport[0]+viewport[2]) ||
     (mouseY > viewport[1]+viewport[3]) ){
    selectedMesh = selectedTriangle = -1;
    return;
  }
    
  /* the x and y are with respect to the top left of the window,
   * we want from the bottom left, so flip by the y coordinate
   * the height of the viewport is
   */
  mouseY = (viewport[1]+viewport[3]) - mouseY;

  /* unproject a point on the screen (z=0) */
  if(GL_FALSE == gluUnProject((GLdouble)(mouseX), (GLdouble)(mouseY), 0.0,
			      modelview, projection, viewport,
			      &wx, &wy, &wz)){
    selectedMesh = selectedTriangle = -1;
    return;
  }
  point screen(wx, wy, wz);

  /* unproject a point in front of the camera (z>0) */
  if(GL_FALSE == gluUnProject((GLdouble)(mouseX), (GLdouble)(mouseY), 1.0,
			      modelview, projection, viewport,
			      &wx, &wy, &wz)){
    selectedMesh = selectedTriangle = -1;
    return;
  }
  point forward(wx, wy, wz);

  /* figure out what was clicked on */
  findTri(screen, forward);
}

bool key_select(unsigned char key,int x, int y){
  switch(key){
  case 's':
    selectTriangle(x, y);
    break;

  default:
    return false;
  }

  glutPostRedisplay();
  return true;
}
