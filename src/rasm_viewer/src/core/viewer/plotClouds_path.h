/*
 * Draws evaluated paths (eg, from a-star) showing the shortest distance
 * to a goal location.
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_PATH_H
#define PLOT_CLOUDS_PATH_H

#include "helpers.h"

enum drawPathModes{
  DRAW_PATH_POINT=0,
  DRAW_PATH_LINES,
  DRAW_PATH_SOLID,
  NUM_DRAW_PATH_MODES
};

class pathPoint{
public:
  float cellcost, pathcost;
  point start, end;
  pathPoint(METERS ax, METERS ay, METERS bx, METERS by, float ccost, float pcost)
    :cellcost(ccost),pathcost(pcost),start(ax, ay, 1000), end(bx, by, 1000){}
};
class path{
  bool haveHeights;
public:
  std::vector<pathPoint> data;
  float minPathCost, maxPathCost;
  double sumPathCosts;
  path():haveHeights(0), sumPathCosts(0){}
  unsigned int size()const {return data.size();}
  void add(METERS x1, METERS y1, METERS x2, METERS y2,
	   float cellcost, float pathcost);

  void simpleLinearSearch(point &p, const triangles *mesh);
  void linearHeightSearch(point &p, const triangles *mesh);

  void generateHeights(const triangles *mesh, float *costs, float *pathcosts);

  void draw(int useColor=-1);
};

int readPaths(const char *file);

#endif
