#include "plotClouds_path.h"
#include "plotClouds_color.h"
#include "plotClouds_globals.h"

#include "spacial.h"

#include <GL/gl.h>

void path::add(METERS x1, METERS y1, METERS x2, METERS y2,
	       float cellcost, float pathcost){
  if(0 == data.size())minPathCost=maxPathCost=pathcost;
  else{
    if(pathcost < minPathCost)minPathCost=pathcost;
    if(pathcost > maxPathCost)maxPathCost=pathcost;
  }

  sumPathCosts+=(double)pathcost;

  data.push_back(pathPoint(x1, y1, x2, y2, cellcost, pathcost));
}

void path::simpleLinearSearch(point &p, const triangles *mesh){
  for(unsigned int j=0;j<mesh->size();j++){
    triangle tri = mesh->get(j);
    if(tri.contains2D(p)){
      tri.getHeight(p);
      return;
    }
  }
}

void path::linearHeightSearch(point &p, const triangles *mesh){
  const float ep = 0.0001;
  p.x+=ep;
  if(p.z<=-1000)simpleLinearSearch(p, mesh);
  p.x-=ep;
  p.x-=ep;
  if(p.z<=-1000)simpleLinearSearch(p, mesh);
  p.x+=ep;
  p.y-=ep;
  if(p.z<=-1000)simpleLinearSearch(p, mesh);
  p.y+=ep;
  p.y+=ep;
  if(p.z<=-1000)simpleLinearSearch(p, mesh);
  p.y-=ep;
}

void path::generateHeights(const triangles *mesh,
			   float *costs, float *pathcosts){
  if(haveHeights)return;
  assert(mesh);
  assert(costs);
  assert(pathcosts);
  points tmppoints;
  for(unsigned int i=0;i<data.size();i++){
    /* any parts of paths that are outside will be drawn at altitude 0 */
    //data[i].start.z = data[i].end.z = 0;
    /* only show parts of paths that fall on the mesh */
    data[i].start.z = data[i].end.z = -1000;
    tmppoints.add(data[i].start);
    tmppoints.add(data[i].end);
  }
  quadtree qt(tmppoints, 10);
  
  //printf("Generating heights for this path from a mesh...\n");
  for(unsigned int j=0;j<mesh->size();j++){
    triangle tri = mesh->get(j);
    points dummy;
    std::vector<int> indices;
    qt.getInterior(tri, dummy, indices);

    for(unsigned int i=0;i<indices.size();i++){
      int index = indices[i]/2;
      if(indices[i]%2 == 0){
	tri.getHeight(data[index].start);
	costs[j] = data[index].cellcost;
	pathcosts[j] = data[index].pathcost;
      }else
	tri.getHeight(data[index].end);
    }
  }

  for(unsigned int i=0;i<data.size();i++){
    if(data[i].start.z <= -1000)
      linearHeightSearch(data[i].start, mesh);
    if(data[i].end.z <= -1000)
      linearHeightSearch(data[i].end, mesh);
  }
  
  //printf("Done initialization\n");
  haveHeights=true;
}

void path::draw(int useColor){
  assert(haveHeights);

#define DRAW_CELL_COST 1
  switch(drawPathMode){
  case DRAW_PATH_POINT:
    glBegin(GL_POINTS);
    break;
  default:
    glBegin(GL_LINES);
  }
  //printf("Drawing path with %d points\n", data.size());
  for(unsigned int i=0;i<data.size();i++){
    if(data[i].start.z == -1000 || data[i].end.z == -1000){
#if 0
      printf("Unable to draw %g,%g (%g) -> %g,%g (%g)\n",
	     data[i].start.x, data[i].start.y, data[i].start.z,
	     data[i].end.x, data[i].end.y, data[i].end.z);
#endif
      continue;
    }
    //printf("Drawing %g,%g -> %g,%g\n", data[i].start.x, data[i].start.y, data[i].end.x, data[i].end.y);
    /* pull out the current, min, and max costs */
#if DRAW_CELL_COST
    float maximumVal = 255;
    float minimumVal = 5.0;
    float currentVal = data[i].cellcost;
#else
    float meanPathCost=(float)(sumPathCosts/((double)data.size()));
    float maximumVal = meanPathCost*2.0 - minPathCost;
    if(maximumVal > maxPathCost)maximumVal = maxPathCost;
    float minimumVal = minPathCost;
    float currentVal = data[i].pathcost;
#endif
    /* set the color linearly based on the cost */
    if(currentVal>maximumVal)currentVal=maximumVal;
    if(currentVal<minimumVal)currentVal=minimumVal;
    float p = (currentVal-minimumVal)/(maximumVal-minimumVal);
    if(useColor<0 || useColor>=(int)pathColors.size())
      glColor4f(p,1.0-p,0,1);
    else
      pathColors[useColor]->glColor4();

    data[i].start.glVertex();
    switch(drawPathMode){
    case DRAW_PATH_LINES:
      glColor4f(p,1.0-p,0,0);//fade out
      //glColor4f(0,0,1,1);//fade to blue
    case DRAW_PATH_SOLID:
      data[i].end.glVertex();
      break;
    default:
      break;
    }
  }
  glEnd();
}

int readPaths(const char *file){
  FILE *f=fopen(file, "r");
  if(!f)return -1;

  path *p = new path();

  METERS a, b, c, d;
  float cellcost, pathcost;
  char buf[128];
  while(!feof(f) && fgets(buf, sizeof(buf), f)){
    if(6 == sscanf(buf, "%lg %lg %lg %lg %f %f\n", &a, &b, &c, &d, &cellcost, &pathcost)){
      p->add(a, b, c, d, cellcost, pathcost);
      continue;
    }
    if(2 == sscanf(buf, "%lg %lg ? ? ? ?\n", &a, &b))continue;
    printf("Warning Skipping bad line \"%s\"\n", buf);
  }
  fclose(f);

  printf("Read path with %d steps from %s\n", p->size(), file);

  paths.push_back(p);

  return 0;
}
