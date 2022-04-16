#include "plotClouds_edge.h"
#include "plotClouds_globals.h"

#include <math.h>

METERS meanEdgeLength(const triangles *t){
  if(!t || t->size()<=0)return 0;
  METERS acc=0;

  for(unsigned int i=0;i<t->size();i++)
    acc += t->get(i).getPerimeter();

  return acc/((METERS)(3*t->size()));
}

METERS stdEdgeLength(const triangles *t, METERS m){
  if(!t || t->size()<=0)return 0;
  METERS acc=0;

  METERS l[3]={0};
  for(unsigned int i=0;i<t->size();i++){
    t->get(i).getLengths(l);
    for(int j=0;j<3;j++){
      l[j] -= m;
      l[j] = l[j] * l[j];
      acc += l[j];
    }
  }

  return sqrt(acc/((METERS)(3*t->size())));
}


bool shouldPruneLarge(const triangle &t, METERS mean, METERS allowedDeviation){
  METERS l[3]={0};
  t.getLengths(l);
  for(int j=0;j<3;j++){
    if(fabs(l[j] - mean) > allowedDeviation)return true;
  }
  return false;
}

bool shouldPruneSmall(const triangle &t, METERS mean, METERS allowedDeviation){
  METERS l[3]={0};
  t.getLengths(l);
  for(int j=0;j<3;j++){
    if(fabs(l[j] - mean) < allowedDeviation)return true;
  }
  return false;
}

bool shouldPruneVertical(const triangle &t, RADIANS minAngle){
  point n = t.getNorm();
  n /= n.norm();
  METERS m = sin(minAngle);
  return (fabs(n.z) < fabs(m));
}

triangles *pruneLongEdges(const triangles *input, float deviations){
  METERS m = meanEdgeLength(input);
  METERS std = stdEdgeLength(input, m);
  METERS allowedDeviation = std*deviations;
  triangles *ret=new triangles();
  for(unsigned int i=0;i<input->size();i++){
    if(!shouldPruneLarge(input->get(i), m, allowedDeviation))
      ret->add(input->get(i));
    else if(verbose)
      printf("Pruned triangle %d\n", i);
  }

  printf("Pruned long edges (mean %g, std %g, times %g = allowed %g)\n",
	 m, std, maxDeviationsLong, allowedDeviation);
  printf("(Mesh went from %d triangles to %d)\n", input->size(), ret->size());

  return ret;
}

triangles *pruneShortEdges(const triangles *input, float deviations){
  METERS m = meanEdgeLength(input);
  METERS std = stdEdgeLength(input, m);
  METERS allowedDeviation = std*deviations;
  triangles *ret=new triangles();
  for(unsigned int i=0;i<input->size();i++){
    if(!shouldPruneSmall(input->get(i), m, allowedDeviation))
      ret->add(input->get(i));
    else if(verbose)
      printf("Pruned triangle %d\n", i);
  }

  printf("Pruned short edges (mean %g, std %g, times %g = allowed %g)\n",
	 m, std, maxDeviationsShort, allowedDeviation);
  printf("(Mesh went from %d triangles to %d)\n", input->size(), ret->size());

  return ret;
}

triangles *pruneVertical(const triangles *input, RADIANS minAngle){
  triangles *ret=new triangles();
  for(unsigned int i=0;i<input->size();i++){
    if(!shouldPruneVertical(input->get(i), minAngle))
      ret->add(input->get(i));
    else if(verbose)
      printf("Pruned triangle %d (below %g radians)\n", i, minAngle);
  }

  printf("Pruned vertical faces\n");
  printf("(Mesh went from %d triangles to %d)\n", input->size(), ret->size());

  return ret;
}
