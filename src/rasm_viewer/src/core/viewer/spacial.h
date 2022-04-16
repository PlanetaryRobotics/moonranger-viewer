#ifndef SPACIAL_H
#define SPACIAL_H

#include "helpers.h"

class quadnode{

  struct corner{
    quadnode *node;
    METERS x, y;
    corner(METERS _x, METERS _y):node(NULL),x(_x),y(_y){}
  };

  points data;
  std::vector<int> originalIndices;
  corner *NN, *NP, *PN, *PP;
  METERS centerX, centerY;
 public:
  quadnode(METERS minx, METERS miny, METERS maxx, METERS maxy);
  int insert(point p, int index, int level);
  void addInterior(point minP, point maxP,
		   points &interior, std::vector<int> &indices);
  void addInterior(point minP, point maxP, triangle t,
		   points &interior, std::vector<int> &indices);
  void print(int level);
};

class quadtree{
  quadnode *head;
  int numpoints;
  int maxlevels;
  void initialize(point minP, point maxP, int levels);

 public:
  quadtree(points p, int levels);
  void add(point p);

  void getInterior(triangle t, points &p, std::vector<int> &indices);
  void getInterior(point minP, point maxP, points &p, std::vector<int> &indices);
  void print();
};

#endif
