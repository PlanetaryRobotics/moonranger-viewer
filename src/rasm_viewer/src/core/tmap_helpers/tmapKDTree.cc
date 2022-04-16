#include <rasm_common_types.h>
#include <tmapKDTree.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

void TMAP::kdnode_interior::print(FILE *f, unsigned int level) const{
  for(unsigned int i = 0;i<level; i++)
    fprintf(f, " ");
  fprintf(f, "Interior node split on axis %d, val ", axis);
  fprintf(f, RASM_UNITS_FORMAT, val);
  if(!L)fprintf(f, " missing L child");
  if(!R)fprintf(f, " missing R child");
  fprintf(f, "\n");
  if(L)
    L->print(f, level+1);
  if(R)
    R->print(f, level+1);
  fprintf(f, "\n");
}

void TMAP::kdnode_leaf::print(FILE *f, unsigned int level) const{
  for(unsigned int i = 0;i<level; i++)
    fprintf(f, " ");
  fprintf(f, "Leaf node has %d of %d entries:\n", N, MAX_KD_LEAF);
  for(unsigned int j=0;j<N;j++){
    for(unsigned int i = 0;i<1+level; i++)
      fprintf(f, " ");
    points[j].print(f);
    fprintf(f, " (%d)\n", indices[j]);
  }
}

int TMAP::kdnode_interior::lookup(const RASM::point3d &target) const{
  TMAP::kdnode *child = ((target.coord3d[axis] < val)?L:R);
  if(!child)return -1;
  return child->lookup(target);
}

int TMAP::kdnode_interior::lookup(const RASM::point3d &target,
				  RASM_UNITS *mindist, double &distSq) const{
  TMAP::kdnode *first  = ((target.coord3d[axis] < val)?L:R);
  TMAP::kdnode *second = ((target.coord3d[axis] < val)?R:L);

  assert(L);
  assert(R);

  int ret = -1;

  /* check the better child */
  ret = first->lookup(target, mindist, distSq);

  RASM_UNITS saved = mindist[axis];

  /* see if there is a chance that the child will have anything better */
  mindist[axis]=target.coord3d[axis]-val;
  double minDistSq = 0;
  for(int i=0;i<3;i++){
    double d = RASM_TO_METERS(mindist[i]);
    minDistSq += (d*d);
  }
  if(distSq > minDistSq){
    /* check the other child */
    int better = second->lookup(target, mindist, distSq);
    if(better != -1)
      ret = better;/* found something better */
  }

  mindist[axis]=saved;

  return ret;
}

int TMAP::kdnode_leaf::lookup(const RASM::point3d &target) const{
  /* linear search, stop if we find the target */
  for(unsigned int i=0;i<N;i++)
    if(target == points[i])
      return indices[i];
  return -1;
}

int TMAP::kdnode_leaf::lookup(const RASM::point3d &target,
			      RASM_UNITS *mindist, double &distSq) const{
  int ret = -1;
  /* linear search through all the stored points */
  for(unsigned int i=0;i<N;i++){
    double dsq = distSq3d(target, points[i]);
    /*
    for(int j=0;j<level;j++)printf(" ");
    printf("leaf has index %d ", indices[i]);
    points[i].print(stdout);
    printf("(%0.3fm), current best at %0.3fm\n",
	   dsq, distSq);
    */
    if(distSq<0 || dsq < distSq){
      distSq = dsq;
      ret = (int)(indices[i]);
    }
  }
  return ret;
}

void TMAP::kdnode_leaf::addToLeaf(const RASM::point3d &p, unsigned int ind){
  for(unsigned int i=0;i<N;i++){assert(!(points[i] == p));}
  assert(N<MAX_KD_LEAF);
  indices[N] = ind;
  points[N] = p;
  N++;
}

/*** end of kdnode functions ***/


unsigned int TMAP::kdTree::splitList(unsigned int *indices,
				     unsigned int len,
				     unsigned int axis,
				     RASM_UNITS &value) const{
  assert((len)>1);

  double sum=0.0;
  for(unsigned int i=0;i<len;i++)
    sum += RASM_TO_METERS(points[indices[i]].coord3d[axis]);
  value = METERS_TO_RASM(sum/((double)len));
  unsigned int l=0;
  unsigned int r=len-1;
  while(l<r){
    /* advance l */
    while(l<r && points[indices[l]].coord3d[axis]<value)l++;
    /* advance r */
    while(l<r && !(points[indices[r]].coord3d[axis]<value))r--;
    if(l<r){
      /* l is the first index greater than value
       * r is the last index less than value
       * swap them
       */
      unsigned int swap = indices[l];
      indices[l] = indices[r];
      indices[r] = swap;
    }
  }

  return l;
}

TMAP::kdTree::kdTree():points(NULL),numPoints(0),root(NULL){}
TMAP::kdTree::~kdTree(){if(root)delete root;root=NULL;numPoints=0;}


static unsigned int effectiveAxis(unsigned int axis){
  /* just cycle through the axes: 0,1,2,0,1,2,0... */
  //return axis%3;

  /* only use axis 2 every other time:  0,1,0,1,2,0,1,0,1,2... */
  //unsigned int ret[5]={0,1,0,1,2};
  //return ret[axis%5];

  /* only use axis 2 every third time:  0,1,0,1,0,1,2... */
  unsigned int ret[7]={0,1,0,1,0,1,2};
  return ret[axis%7];

  /* only use axis 2 every fourth time: 0,1,0,1,0,1,0,1,2... */
  //unsigned int ret[9]={0,1,0,1,0,1,0,1,2};
  //return ret[axis%9];
}

TMAP::kdnode *TMAP::kdTree::insert(unsigned int *indices,
				   unsigned int len,
				   unsigned int axis){
  if(len<=TMAP::kdnode_leaf::MAX_KD_LEAF){
    TMAP::kdnode_leaf *l = new TMAP::kdnode_leaf();
    for(unsigned int i=0;i<len;i++)
      l->addToLeaf(points[indices[i]], indices[i]);
    return l;
  }else{
    RASM_UNITS val = 0;
    unsigned int N = 0;

    int splits=0;
    while(1){
      N = splitList(indices, len, effectiveAxis(axis), val);
      if(N>0 && N<len)break;
      axis++;
      splits++;
      assert(splits<10);
    }

    TMAP::kdnode_interior *ret=new TMAP::kdnode_interior(effectiveAxis(axis),
							 val);
    axis++;
    ret->L = insert(indices, N, axis);
    ret->R = insert(indices+N, len-N, axis);
    return ret;
  }
}

void TMAP::kdTree::insert(const RASM::point3d *_points,
			  unsigned int _numPoints){
  assert(!points && 0==numPoints);
  points = _points;
  numPoints = _numPoints;

  /* create the indices */
  unsigned int *indices = (unsigned int *)malloc(_numPoints * sizeof(unsigned int));
  for(unsigned int i=0;i<_numPoints;i++)indices[i]=i;

  root = insert(indices, _numPoints, 0);

  free(indices);
}

void TMAP::kdTree::print(FILE *f) const{
  fprintf(f, "kdtree with %d points:\n", numPoints);
  if(root)root->print(f, 1);
}

int TMAP::kdTree::lookup(const RASM::point3d &target) const{
  if(!root)return -1;
  return root->lookup(target);
}

/* finds the closest match */
int TMAP::kdTree::lookup(const RASM::point3d &target, double &distSq) const{
  distSq = -1;
  if(!root)return -1;
  RASM_UNITS mindist[3]={0,0,0};
  return root->lookup(target, mindist, distSq);
}
