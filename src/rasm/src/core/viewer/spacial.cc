#include "spacial.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

/* helper function, tests if p falls inside this bounding box */
int inside2D(METERS minx, METERS miny, METERS maxx, METERS maxy,
	     METERS px, METERS py){
  return (px>=minx && py>=miny && px<=maxx && py<=maxy);
}
int inside2D(const point &minP, const point &maxP, const point &p){
  return inside2D(minP.x, minP.y, maxP.x, maxP.y, p.x, p.y);
}

quadnode::quadnode(METERS minx, METERS miny, METERS maxx, METERS maxy)
  :NN(new corner(minx, miny)),
   NP(new corner(minx, maxy)),
   PN(new corner(maxx, miny)),
   PP(new corner(maxx, maxy)),
   centerX((minx+maxx)/2.0),
   centerY((miny+maxy)/2.0){}

int quadnode::insert(point p, int index, int level){
  /* sanity check the input */
  assert(inside2D(NN->x, NN->y, PP->x, PP->y, p.x, p.y));
  assert(level>=0);

  /* base case, insert */
  if(0 == level){
    data.add(p);
    originalIndices.push_back(index);
    return 0;
  }

  /* otherwise, recurse on appropriate child (creating it if needed) */
  if(p.x < centerX && p.y < centerY){
    if(!NN->node)NN->node = new quadnode(NN->x, NN->y, centerX, centerY);
    assert(NN->node);
    return NN->node->insert(p, index, level-1);
  }

  if(p.x < centerX && p.y >= centerY){
    if(!NP->node)NP->node = new quadnode(NP->x, centerY, centerX, NP->y);
    assert(NP->node);
    return NP->node->insert(p, index, level-1);
  }

  if(p.x >= centerX && p.y < centerY){
    if(!PN->node)PN->node = new quadnode(centerX, PN->y, PN->x, centerY);
    assert(PN->node);
    return PN->node->insert(p, index, level-1);
  }

  if(p.x >= centerX && p.y >= centerY){
    if(!PP->node)PP->node = new quadnode(centerX, centerY, PP->x, PP->y);
    assert(PP->node);
    return PP->node->insert(p, index, level-1);
  }

  printf("error, could not select child!\n");
  return -1;
}


void quadtree::initialize(point minP, point maxP, int levels){
  assert(!head);
  head = new quadnode(minP.x, minP.y, maxP.x, maxP.y);
  numpoints = 0;
  maxlevels = levels;
}

quadtree::quadtree(points p, int levels):head(NULL){
  point minP(0,0,0);
  point maxP(0,0,0);
  p.boundingBox(minP, maxP);

#if 0
  printf("Creating quadtree for %d points with %d levels\n", p.size(), levels);
  minP.print("minPoint: ","\n");
  maxP.print("maxPoint: ","\n");
#endif

  initialize(minP, maxP, levels);
  for(unsigned int i=0;i<p.size();i++)add(p.get(i));

#if 0
  printf("Final tree\n");
  print();
#endif
}

void quadtree::add(point p){
  //p.print("Adding ","\n");
  assert(0 == head->insert(p, numpoints++, maxlevels));
}

void quadtree::getInterior(triangle t, points &p, std::vector<int> &indices){
  point minP(0,0,0);
  point maxP(0,0,0);
  t.boundingBox(minP, maxP);
  head->addInterior(minP, maxP, t, p, indices);
}


void quadnode::addInterior(point minP, point maxP,
			   points &interior, std::vector<int> &indices){
  /* sanity check that there is some overlap within our corners */
  if(maxP.x < NN->x || maxP.y < NN->y || minP.x > PP->x || minP.y > PP->y)
    return;

  /* add any points in this node */
  for(unsigned int i=0;i<data.size();i++){
    if(!inside2D(minP, maxP, data.get(i)))continue;
    interior.add(data.get(i));
    indices.push_back(originalIndices[i]);
  }

  /* recurse on appropriate children */  
  if(NN->node && minP.x < centerX && minP.y < centerY)
    NN->node->addInterior(minP, maxP, interior, indices);
  if(NP->node && minP.x < centerX && maxP.y > centerY)
    NP->node->addInterior(minP, maxP, interior, indices);
  if(PN->node && maxP.x > centerX && minP.y < centerY)
    PN->node->addInterior(minP, maxP, interior, indices);
  if(PP->node && maxP.x > centerX && maxP.y > centerY)
    PP->node->addInterior(minP, maxP, interior, indices);
}

void quadnode::addInterior(point minP, point maxP, triangle t,
			   points &interior, std::vector<int> &indices){
  /* sanity check that there is some overlap within our corners */
  if(maxP.x < NN->x || maxP.y < NN->y || minP.x > PP->x || minP.y > PP->y)
    return;

  /* add any points in this node */
  for(unsigned int i=0;i<data.size();i++){
    if(!inside2D(minP, maxP, data.get(i)))continue;
    if(!t.contains2D(data.get(i)))continue;
    interior.add(data.get(i));
    indices.push_back(originalIndices[i]);
  }

  /* recurse on appropriate children */  
  if(NN->node && minP.x < centerX && minP.y < centerY)
    NN->node->addInterior(minP, maxP, t, interior, indices);
  if(NP->node && minP.x < centerX && maxP.y > centerY)
    NP->node->addInterior(minP, maxP, t, interior, indices);
  if(PN->node && maxP.x > centerX && minP.y < centerY)
    PN->node->addInterior(minP, maxP, t, interior, indices);
  if(PP->node && maxP.x > centerX && maxP.y > centerY)
    PP->node->addInterior(minP, maxP, t, interior, indices);
}

void quadtree::getInterior(point minP, point maxP, points &p, std::vector<int> &indices){
  head->addInterior(minP, maxP, p, indices);
}

void quadtree::print(){
  printf("This quad tree contains %d points at %d levels\n", numpoints, maxlevels);
  //head->print(0);
}

void quadnode::print(int level){
  assert(originalIndices.size() == data.size());
  printf("This level %d node has %d points:", level, data.size());
#if 0
  for(int i=0;i<data.size();i++)
    printf("\nPoint %d: %g %g %g\n", originalIndices[i], 
	   data.get(i).x, data.get(i).y, data.get(i).z);
#endif
  printf("\n");
  printf("The bounds are X= %g to %g, Y= %g to %g, center= %g,%g\n", 
	 NN->x, PP->x, NN->y, PP->y, centerX, centerY);
  printf("Children: NN: %d, NP: %d, PN: %d, PP: %d\n",
	 !!NN->node, !!NP->node, !!PN->node, !!PP->node);
  if(NN->node)NN->node->print(level+1);
  if(NP->node)NP->node->print(level+1);
  if(PN->node)PN->node->print(level+1);
  if(PP->node)PP->node->print(level+1);
}
