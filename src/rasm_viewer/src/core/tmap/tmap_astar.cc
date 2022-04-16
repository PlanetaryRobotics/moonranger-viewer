#include <stdlib.h>
#include <stdio.h>
#include <string.h>/* for memset() */
#include <assert.h>
#include <math.h>
#include <planeHelpers.h> /* for getHeightAt() */
#include <lineHelpers.h>  /* for intersects() */
#include <tmap_astar.h>

/* whether or not to print lots of debugging output
 * also enables extra tests
 */
#define DEBUG 0

/*
 * The following config variables are global so they can be overridden, for
 * example in main().
 */

/* whether or not to use a heurstic cost function
 * without this feature the search is not directed to the target
 * (ie, breadth first search)
 */
bool TMAP::USE_HEURISTIC=true;

/* use this option to disable path smoothing
 * instead, merely use paths that transition along triangle centers
 * path smoothing should significantly reduce path lengths resulting
 * in path lengths closer to optimal
 * also with a heuristic this will allow the search to terminate faster
 *
 * shorter path lengths are important because arc selection requires
 * an accurate/unbiased estimate of the cost from the end of each arc
 * to the goal
 */
bool TMAP::NO_PATH_SMOOTHING=false;

/* triangle cost in astar search, based on z component of unit normal */
float TMAP::astarZmin = 0.7;
float TMAP::astarZmax = 0.95;

/* altitude cost based on vertical distance from a target */
float TMAP::minAltitude=1.0;
float TMAP::maxAltitude = 5.0;

/* look at the total altitude change of this triangle
 * if it's a tiny change (ie, a tiny triangle)
 * then ignore the slope component or atleast scale it down
 */
float TMAP::minPeakTrough = 0.1;
float TMAP::maxPeakTrough = 0.3;

/* use this option to do very high resolution path searching
 * this is very slow so it should not be used in normal operation,
 * but is useful as a baseline
 */
#define USE_EDGE_SEARCH 0
#if USE_EDGE_SEARCH
#include "edgeContainer.h"
static edgeDB *edgeSearch;
#endif

/* this data structure stores all the nodes that we want to look at
 * it stores the node index and the minimum possible distance to the
 * goal
 *
 * the minimum cost to the goal is atleast the euclidean distance,
 * which is computed by dist3d (or dist2d in case of problems)
 *
 * another way of computing minimum cost is by looking at neighbors,
 * and using costs computed by estimatedCostThroughNeighbor()
 */

namespace TMAP {
  class minHeap : public openListInterface{
    astarNode *data;
    /* data[lookupTable[i]].triangleIndex == i */
    int *lookupTable;
    unsigned int size, capacity;

  public:
    virtual unsigned int getsize()const{return size;}
    virtual float minCost() const{
      assert(size>0);
      if(USE_HEURISTIC)
	return data[0].pathcost+data[0].heuristicCost;
      else
	return data[0].pathcost;
    }
    virtual int getTriangle(unsigned int i)const{
      assert(i<size);
      return data[i].triangleIndex;
    }
    virtual void setHeuristic(unsigned int i, float h){
      assert(i<size);
      data[i].heuristicCost=h;
    }
    virtual void fixHeap(){
      for(unsigned int i=1;i<size;i++)
	fixUp(i);
    }
    minHeap(){
      data = NULL;
      lookupTable = NULL;
      size=capacity=0;
    }
    virtual ~minHeap(){
      if(capacity>0){
	free(data);
	data=NULL;
	free(lookupTable);
	lookupTable=NULL;
      }
      size=capacity=0;
    }

    virtual void reset(){
      size=0;
      for(unsigned int i=0;i<capacity;i++)lookupTable[i]=-1;
    }

    void inline printNode(const struct astarNode &an)const{
      printf("index %d, neighbor %d, pathcost %0.5f, heuristic %0.5f",
	     an.triangleIndex, an.neighborToGoal,
	     an.pathcost, an.heuristicCost);
    }

    virtual void print() const{
      printf("Heap with %d items:\n", size);
      for(unsigned int i=0;i<size;i++){
	printf(" %03d: ", i);
	printNode(data[i]);
	printf("\n");
      }
      printf("\n");

      for(unsigned int i=1;i<size;i++)
	assert(!compare(data[i], data[getParent(i)]));
      /*
	printf("LookupTable:\n");
	for(int i=0;i<capacity;i++)
	if(lookupTable[i]!=-1)
	printf(" index %d in heap slot %d\n", i, lookupTable[i]);
	printf("\n");
      */
      for(unsigned int i=0;i<capacity;i++)
	assert((-1 == lookupTable[i]) || 
	       ((int)i == data[lookupTable[i]].triangleIndex));
    }

    static unsigned int inline getParent(unsigned int ind){return (ind-1)/2;}
    static unsigned int inline getLChild(unsigned int ind){return (ind+1)*2-1;}
    static unsigned int inline getRChild(unsigned int ind){return (ind+1)*2;}

    /* test if A is less than B */
    static int inline compare(const struct astarNode &A,
			      const struct astarNode &B){
      if(USE_HEURISTIC){
	if(A.heuristicCost<0){
	  assert(B.heuristicCost<0);
	  return (A.pathcost<B.pathcost);
	}
	assert(B.heuristicCost>=0);
	return ((A.heuristicCost+A.pathcost)<(B.heuristicCost+B.pathcost));
      }
      return (A.pathcost<B.pathcost);
    }

    unsigned int fixUp(unsigned int ind){
      struct astarNode newData = data[ind];

      /* initially the new data was in [ind],
       * treat that as an empty spot,
       * keep moving the parent data into the empty spot
       * until the parent data is better than the new data, 
       * then we've found the right spot for the new data and commit it
       */
      while(ind>0){
	/* compare with parent,
	 * if the parent is better, then we're done
	 * if the new data is better, then move the parent
	 */
	int parent = getParent(ind);
	if(compare(data[parent], newData))break;

	data[ind] = data[parent];
	lookupTable[data[ind].triangleIndex] = ind;

	/* advance to the parent */
	ind = parent;
      }

      /* found the right place for the new data */
      data[ind] = newData;
      lookupTable[data[ind].triangleIndex] = ind;
      return ind;
    }

    unsigned int fixDown(){
      unsigned int ind = 0;
      struct TMAP::astarNode newData = data[ind];

      while(1){
	unsigned int L = getLChild(ind);
	if(L >= size)break;/* no children */
	unsigned int R = getRChild(ind);
	if(R >= size){
	  /* special case, only have one child (left child) 
	   * if the child is better, promote it
	   * either way, we're done with the loop
	   */
	  if(compare(data[L], newData)){
	    data[ind] = data[L];
	    lookupTable[data[ind].triangleIndex] = ind;
	    ind = L;
	  }
	  break;
	}

	/* pick the child that might get promoted */
	int betterChild = L;
	if(compare(data[R], data[L]))
	  betterChild = R;

	/* if the parent/new data is better than the child, then we're done */
	if(compare(newData, data[betterChild]))
	  break;

	/* promote the child */
	data[ind] = data[betterChild];
	lookupTable[data[ind].triangleIndex] = ind;
	ind = betterChild;
      }

      /* found the right place for the initial node */
      data[ind] = newData;
      lookupTable[data[ind].triangleIndex] = ind;
      return ind;
    }

    void insert(unsigned int ind, double cost, int neighbor, float heuristic){

      assert(ind>=0);

      struct astarNode newData;
      memset(&newData, 0, sizeof(newData));
      newData.pathcost = cost;
      newData.heuristicCost = heuristic;
      newData.neighborToGoal = neighbor;
      newData.triangleIndex = ind;

      /* check if we already inserted this one */
      if((ind<capacity) && (-1 != lookupTable[ind])){
	/* check if we already have a better one */

#if DEBUG
	printf(" Duplicate entry... old: ");
	printNode(data[lookupTable[ind]]);
	printf(" new: ");
	printNode(newData);
#endif

	if(compare(data[lookupTable[ind]], newData)){
#if DEBUG
	  printf(" old copy is better\n");
#endif
	  return;
	}

#if DEBUG
	printf(" Improving entry\n");
#endif

	/* decrease the cost and fix the data structure */
	data[lookupTable[ind]] = newData;
	fixUp(lookupTable[ind]);
	return;
      }

      /* make sure there is room */
      if((size>=capacity) || (ind>=capacity)){
	int prevCapacity = capacity;
	capacity=((size>ind)?size:ind)*2 + 1;
	data = (astarNode *)realloc(data, capacity*sizeof(astarNode));
	lookupTable = (int *)realloc(lookupTable, capacity*sizeof(int));
	assert(data && lookupTable);
	for(unsigned int i=prevCapacity;i<capacity;i++)lookupTable[i]=-1;
      }

      /* insert a fresh entry */
      data[size] = newData;
      lookupTable[ind] = size;

#if DEBUG
      printf("New entry: ");
      printNode(newData);
      printf("\n");
#endif

      fixUp(size);
      size++;
    }

    void removeFirst(int &ind, float &cost, int &neighbor){
      assert(size>0);
      ind = data[0].triangleIndex;
      cost = data[0].pathcost;
      neighbor = data[0].neighborToGoal;
      lookupTable[ind] = -1;
      size--;

      if(size>0){
	data[0] = data[size];
	lookupTable[data[0].triangleIndex] = 0;
	fixDown();
      }
    }
  }; /*** end of class minHeap ***/

} /* end of namespace TMAP */

int TMAP::tmap_astar::findRemainingTriangleClosestToGoal(){
  assert(centers);
  int closestTriangle=-1;
  float bestDistSq=-1;
  for(unsigned int i=0;i<numTriangles();i++){
    /* this triangle has already been filled in, don't consider it */
    if(astarNodes[i].pathcost > 0)continue;

    float distSq = distSq2d(goalPoint, centers[i]);
    if(bestDistSq<0 || distSq<bestDistSq){
      closestTriangle = (int)i;
      bestDistSq = distSq;
    }
  }
  return closestTriangle;
}

static inline float max3(float a, float b, float c){
  if((a>b)&&(a>c))return a;
  if(b>c)return b;
  return c;
}
static inline float min3(float a, float b, float c){
  if((a<b)&&(a<c))return a;
  if(b<c)return b;
  return c;
}

float TMAP::tmap_astar::getCost(int tInd) const{
  /* get the triangle normal */
  RASM::point3d p = vertices[faces[tInd].points[0]];
  RASM::point3d a = vertices[faces[tInd].points[1]] - p;
  RASM::point3d b = vertices[faces[tInd].points[2]] - p;
  RASM::point3d norm = a*b;

  float x = RASM_TO_METERS(norm.X());
  float y = RASM_TO_METERS(norm.Y());
  float z = RASM_TO_METERS(norm.Z());
  float magSq = x*x + y*y + z*z;
  if(magSq<=0){
    a.print();
    printf(" cross ");
    b.print();
    printf(" = ");
    norm.print();
    printf("\n");

    printf("Magnitude: %f\n", magSq);
    assert(0);
  }

  z = fabs(RASM_TO_METERS(norm.Z())/sqrt(magSq));

  /* a function of slope */
  const float minZ = astarZmin;
  const float maxZ = astarZmax;
  float slopePercent = 1.0;
  if(z < minZ){
    //return OBST;
    slopePercent = 0.0;
  }else if(z > maxZ){
    //return FREE;
    slopePercent = 1.0;
  }else{
    slopePercent = (z - minZ)/(maxZ - minZ);
    slopePercent = 1.0 - sqrt(1.0-slopePercent);
  }
  /* 0 percent is obstacle, 1.0 percent is free */

  /* now we add the height, abs(height)<minAltitude is ok so we set percent to 1.0
     abs(height) >maxAltitude is bad so we set percent to 0
     otherwise just scale linearly in between
   */

  float peak = max3(RASM_TO_METERS(vertices[faces[tInd].points[0]].Z()),
		    RASM_TO_METERS(vertices[faces[tInd].points[1]].Z()),
		    RASM_TO_METERS(vertices[faces[tInd].points[2]].Z()));
  float trough = min3(RASM_TO_METERS(vertices[faces[tInd].points[0]].Z()),
		      RASM_TO_METERS(vertices[faces[tInd].points[1]].Z()),
		      RASM_TO_METERS(vertices[faces[tInd].points[2]].Z()));
  const float targetAltitude=0.0;
  float deltaAltitude = fabs(peak-targetAltitude);
  if(fabs(trough-targetAltitude)>deltaAltitude)
    deltaAltitude = fabs(trough-targetAltitude);
  float altitudePercent=0;
  if(deltaAltitude > maxAltitude){
    //return OBST;
    altitudePercent = 0.0;
  }else if(deltaAltitude < minAltitude){
    //return FREE;
    altitudePercent = 1.0;
  }else{
    altitudePercent = (deltaAltitude-minAltitude)/(maxAltitude-minAltitude);
    altitudePercent = 1.0 - altitudePercent*altitudePercent;
  }

  if(peak-trough<minPeakTrough){
    /*negligible altitude change, ignore slope */
    slopePercent = 1.0;
  }else if(peak-trough<maxPeakTrough){
    /* small altitude change, don't count the slope much... */
    float slopePercentPercent = 1.0 - (((peak-trough)-minPeakTrough)/(maxPeakTrough - minPeakTrough));
    /* slopePercentPercent 1 = small change, 0 = big enough change 
       adjust slope percent accordingly (it has 0 = obstacle, 1 = free)
     */
    slopePercent = slopePercent * slopePercentPercent + 
      1.0*(1.0 - slopePercentPercent);
  }


  /* combine the percentages */
  //float percent = 0.5*slopePercent  + 0.5*altitudePercent;
  float percent = sqrt(slopePercent*altitudePercent);

  float ret = (((1.0-percent) * (float)OBST) + (percent*(float)FREE));
  return ret;
}

void TMAP::tmap_astar::astarReset(){
  openList->reset();
  if(astarNodes){
    free(astarNodes);
    goalIndex = -1;
  }

  astarNodes = (TMAP::astarNode *)malloc(numTriangles() * sizeof(TMAP::astarNode));
  for(unsigned int i=0;i<numTriangles();i++){
    astarNodes[i].cellcost=getCost(i);
    astarNodes[i].pathcost=-1;
    astarNodes[i].neighborToGoal=-1;
    astarNodes[i].triangleIndex = -1;/* this isn't used by astarTMAP, only TMAP_minHeap */
    /* initially the transition point is undefined and
     * the two neighbors other than neighborToGoal are not known
     * the goal triangle will not set these
     */
    for(unsigned int j=0;j<2;j++)
      astarNodes[i].neighborFromGoal[j] = -1;
  }
}

void TMAP::tmap_astar::addExteriorGoal(){
  goalIndex = numTriangles();
  printf("add exterior goal, set goal to %d\n", goalIndex);

  astarReset();
}

void TMAP::tmap_astar::setGoal(const RASM::point2d &goal){

  goalPoint = goal;
#if DEBUG
  printf("setting goal to ");
  goalPoint.print();
  printf("\n");
#endif

  goalIndex = findTriangle2d(goalPoint);

  if(goalIndex<0){
    printf("Warning, goal is off the map\n");
    findTriangle2d(goalPoint, 1);
    assert(0);
    addExteriorGoal();
  }else{
    assert(goalIndex < (int)numTriangles());

    int g = goalIndex;
    astarReset();
    goalIndex = g;

    astarNodes[goalIndex].pathcost = 0;
    openList->insert(goalIndex, 0, -1, -1);
#if DEBUG
    printf("\nAfter setting goal:\n");
    openList->print();
#endif
  }

#if USE_EDGE_SEARCH
  if(edgeSearch){
    free(edgeSearch);
    edgeSearch = NULL;
  }
#endif
}


float TMAP::tmap_astar::minCostToTriangle(int ind, int targetTriangle){
  if(ind == targetTriangle)return 0.0;
  assert((targetTriangle>=0) && (targetTriangle<(int)numTriangles()));
  assert(centers);
  float dist = dist3d(centers[ind], centers[targetTriangle]);
  //TODO Why is this assert here?  What's the right thing to do?
  //  assert(dist>0);
  if (dist>0) {
  // printf("[%s:%d %f] assert(dist>0)\n",__FILE__, __LINE__); 
    return(0.0);
  }

  float triangleSize = -1;
  for(int i=0;i<3;i++){
    float d = dist3d(vertices[faces[ind].points[i]],
		     vertices[faces[ind].points[(i+1)%3]]);
    if((triangleSize<0) || (d<triangleSize))
      triangleSize = d;
  }
  for(int i=0;i<3;i++){
    float d = dist3d(vertices[faces[targetTriangle].points[i]],
		     vertices[faces[targetTriangle].points[(i+1)%3]]);
    if((triangleSize<0) || (d<triangleSize))
      triangleSize = d;
  }
  assert(triangleSize>=0);
  if(triangleSize<dist)
    dist -= triangleSize;
  else
    dist = 0;

  float c = ((float)FREE)*dist;
  assert(c>=0);
  return c;
}

float TMAP::tmap_astar::minCostToGoal(int ind){
  assert(centers);
  assert(goalIndex>=0);
  if(astarNodes[ind].pathcost>=0)return astarNodes[ind].pathcost;

  if((int)numTriangles() != goalIndex)
    return minCostToTriangle(ind, goalIndex);

  float dist = dist2d(goalPoint, centers[ind]);
  assert(dist>=0);
  return ((float)FREE)*dist;
}

float TMAP::tmap_astar::estimatedCostThroughNeighbor(unsigned int triangleIndex,
						     unsigned int neighbor){

  assert((faces[triangleIndex].neighbors[0] == (int)neighbor) ||
	 (faces[triangleIndex].neighbors[1] == (int)neighbor) ||
	 (faces[triangleIndex].neighbors[2] == (int)neighbor) );
  assert((faces[neighbor].neighbors[0] == (int)triangleIndex) ||
	 (faces[neighbor].neighbors[1] == (int)triangleIndex) ||
	 (faces[neighbor].neighbors[2] == (int)triangleIndex) );
  assert(astarNodes[neighbor].pathcost>=0);
  int neighbor2 = astarNodes[neighbor].neighborToGoal;

  if((-1 == neighbor2) || NO_PATH_SMOOTHING){
    /* the neighbor is the goal */

    float d = dist3d(centers[triangleIndex], centers[neighbor])/2.0;
    
    /*
      printf("From %d to %d: %f*%f + %f*%f\n", triangleIndex, neighbor,
      d, astarNodes[triangleIndex].cellcost, 
      d, astarNodes[neighbor].cellcost);
    */

    /* report the total cost for travelling this distance */
    return /* ( 0.01*(float)FREE) + */
      d*astarNodes[triangleIndex].cellcost + 
      d*astarNodes[neighbor].cellcost + 
      astarNodes[neighbor].pathcost;

  }else{

    /* make sure the path cost was evaluated */
    assert(astarNodes[neighbor].transitionPathCost>=0);
    if(!(((int)triangleIndex == astarNodes[neighbor].neighborFromGoal[0]) ||
	 ((int)triangleIndex == astarNodes[neighbor].neighborFromGoal[1]) )){
      printf("Error!\n");
      printf("estimatedCostThroughNeighbor(%d, neighbor=%d)\n",
	     triangleIndex, neighbor);
      printf("This triangle has neighbors %d,%d,%d\n",
	     faces[triangleIndex].neighbors[0],
	     faces[triangleIndex].neighbors[1],
	     faces[triangleIndex].neighbors[2]);
      printf("Neighbor has neighbors %d,%d,%d\n",
	     faces[neighbor].neighbors[0],
	     faces[neighbor].neighbors[1],
	     faces[neighbor].neighbors[2]);
      printf("Neighbor has neighborsFromGoal %d,%d\n",
	     astarNodes[neighbor].neighborFromGoal[0],
	     astarNodes[neighbor].neighborFromGoal[1]);
      exit(-1);
    }
  
    unsigned int i=0;
    if((int)triangleIndex == astarNodes[neighbor].neighborFromGoal[1])
      i=1;

    float d = dist3d(centers[triangleIndex],
		     astarNodes[neighbor].transitionPoint[i])/2.0;
    float neighborPathCost = astarNodes[neighbor].transitionPathCost[i];

    /*
    printf("From %d to %d: %f*%f + %f*%f + %f\n", triangleIndex, neighbor, 
	   d, astarNodes[triangleIndex].cellcost,
	   d, astarNodes[neighbor].cellcost,
	   neighborPathCost);
    */

    /* report the total cost for traveling this distance */
    return (0.01*(float)FREE) + d*astarNodes[triangleIndex].cellcost + d*astarNodes[neighbor].cellcost + neighborPathCost;
  }
}

TMAP::tmap_astar::tmap_astar()
  : tmap_searchable()
{
  openList = new TMAP::minHeap();
  astarNodes = NULL;
  goalIndex = -1;

}

TMAP::tmap_astar::~tmap_astar(){
  if(openList){
    delete openList;
    openList=NULL;
  }

  if(astarNodes){
    free(astarNodes);
    astarNodes=NULL;
  }
#if USE_EDGE_SEARCH
  if(edgeSearch){
    delete edgeSearch;
    edgeSearch=NULL;
  }
#endif
}

static int getCommonIndex(const unsigned int a[3], 
			  const unsigned int b[3]){
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      if(a[i]==b[j])return (int)(a[i]);
  return -1;
}


float getCostOf(const RASM::point3d &start, const RASM::point3d &end,
		float priorPathCost,float cellcostA, float cellcostB){
  float d = dist3d(start, end);
  float ret = d*0.5*cellcostA + d*0.5*cellcostB + priorPathCost;

#if 0
  start.print();
  printf(" (cell %0.1f)", cellcostA);
  printf(" through ");
  end.print();
  printf(" (cell %0.1f, prior %0.1f, dist %0.1f)", cellcostB, priorPathCost, d);
  printf(" is %0.1f\n", ret);
#endif

  return ret;
}

void TMAP::tmap_astar::fillInTransitionPoints(unsigned int triangleIndex){
  const unsigned int i=triangleIndex;
  const int neighborToGoal = astarNodes[triangleIndex].neighborToGoal;
  if(-1 == neighborToGoal){
    /* there is no transition through this triangle, 
       ie, it is the goal */
    astarNodes[i].neighborFromGoal[0] = -1;
    astarNodes[i].neighborFromGoal[1] = -1;
    return;
  }

  /* fill in both neighborFromGoal entries */
  if(faces[i].neighbors[0] == neighborToGoal){
    astarNodes[i].neighborFromGoal[0] = faces[i].neighbors[1];
    astarNodes[i].neighborFromGoal[1] = faces[i].neighbors[2];
  }else if(faces[i].neighbors[1] == neighborToGoal){
    astarNodes[i].neighborFromGoal[0] = faces[i].neighbors[0];
    astarNodes[i].neighborFromGoal[1] = faces[i].neighbors[2];
  }else{
    astarNodes[i].neighborFromGoal[0] = faces[i].neighbors[0];
    astarNodes[i].neighborFromGoal[1] = faces[i].neighbors[1];
  }

  /* make sure one of these entries is the current triangle */
  if( ! ((-1 == astarNodes[neighborToGoal].neighborToGoal)          ||
	 ((int)i == astarNodes[neighborToGoal].neighborFromGoal[0]) ||
	 ((int)i == astarNodes[neighborToGoal].neighborFromGoal[1]) )){
    printf("Error filling in transition points of triangle %d\n", i);
    printf("Neighbor to goal is %d, which has neighbors from goal %d and %d\n",
	   neighborToGoal,
	   astarNodes[neighborToGoal].neighborFromGoal[0],
	   astarNodes[neighborToGoal].neighborFromGoal[1]);
    assert(0);
  }

  /* now compute the transition points and the cost of
     the path from the transition points to the goal */
  for(unsigned int j=0;j<2;j++){
    const int neighborFromGoal = astarNodes[i].neighborFromGoal[j];
    if(-2 == neighborFromGoal)continue;/* no neighbor */

    /* the vertex shared by both the neighborToGoal and neighborFromGoal[j] */
    const int sharedVertex = getCommonIndex(faces[neighborToGoal].points,
					    faces[neighborFromGoal].points);
    if(-1 == sharedVertex){
      printf("Error getting common index...\n");
      printf("At triangle %d, points %d,%d,%d\n", i,
	     faces[i].points[0],
	     faces[i].points[1],
	     faces[i].points[2]);
      printf("To goal %d, points %d,%d,%d\n", neighborToGoal,
	     faces[neighborToGoal].points[0],
	     faces[neighborToGoal].points[1],
	     faces[neighborToGoal].points[2]);
      printf("From goal %d, points %d,%d,%d\n", neighborFromGoal,
	     faces[neighborFromGoal].points[0],
	     faces[neighborFromGoal].points[1],
	     faces[neighborFromGoal].points[2]);

      assert(0);
    }

    /* pick the targetPoint
     * the path goes from the center of this triangle, 
     * through a transition point,
     * then to the targetPoint for which we already have a pathcost
     */
    RASM::point3d targetPoint = centers[neighborToGoal];
    float priorCost = astarNodes[neighborToGoal].pathcost;
    if(-1 != astarNodes[neighborToGoal].neighborToGoal){
      /* neighbor is not the goal, use the transition point */
      if((int)i == astarNodes[neighborToGoal].neighborFromGoal[0]){
	targetPoint = astarNodes[neighborToGoal].transitionPoint[0];
	priorCost = astarNodes[neighborToGoal].transitionPathCost[0];
      }else{
	targetPoint = astarNodes[neighborToGoal].transitionPoint[1];
	priorCost = astarNodes[neighborToGoal].transitionPathCost[1];
      }
    }


    /* check for a large angle situation
       between triangleIndex and neighborFromGoal
       eg, make sure the line between the centers actually crosses
       the edge instead of skirting around it.
       in that case, use the shared vertex as the transition point
     */
    unsigned int v1=faces[triangleIndex].points[0],
      v2=faces[triangleIndex].points[2];
    if(v1 != faces[neighborFromGoal].points[0] &&
       v1 != faces[neighborFromGoal].points[1] &&
       v1 != faces[neighborFromGoal].points[2] )
      v1 = faces[triangleIndex].points[1];
    else if(v2 != faces[neighborFromGoal].points[0] &&
	    v2 != faces[neighborFromGoal].points[1] &&
	    v2 != faces[neighborFromGoal].points[2] )
      v2 = faces[triangleIndex].points[1];

    if(!intersects(centers[triangleIndex], centers[neighborFromGoal],
		   vertices[v1], vertices[v2])){
      astarNodes[i].transitionPoint[j] = vertices[sharedVertex];
    }else{

      /* the transition point is the intersection of two lines:
	 1: from center of triangleIndex to sharedVertex
	 2: from center of neighborFromGoal to targetPoint
	    (targetPoint is in triangle neighborToGoal,
	    it is either the center or the relevant transition point)
      */
      if(-1 == getIntersection(centers[triangleIndex],  vertices[sharedVertex],
			       centers[neighborFromGoal], targetPoint,
			       astarNodes[i].transitionPoint[j])){
	printf("Error getting intersection\n");
	printf("fillInTransitionPoints(%d)\n", triangleIndex);
	printf("Triangle center and the shared vertex (point %d):\n",
	       sharedVertex);
	centers[triangleIndex].print();
	printf(" and ");
	vertices[sharedVertex].print();
	printf("\n");

	printf("center of neighbor (triangle %d) and target point:\n", 
	       neighborFromGoal);
	centers[neighborFromGoal].print();
	printf(" and ");
	targetPoint.print();
	printf("\n");
	
	writeToFile("badTransitionPoints.smf");
	
	assert(0);
      }
    }

#if DEBUG
    printf("Triangle %d, transition from %d to %d: ", 
	   i, neighborFromGoal, neighborToGoal);
    astarNodes[i].transitionPoint[j].print();
    printf("\n");
    /*
    printf(" intersection of nominal line from ");
    centers[neighborFromGoal].print();
    printf(" to ");
    targetPoint.print();
    printf("\n");
    printf(" and center/vertex ");
    centers[triangleIndex].print();
    printf("/");
    vertices[sharedVertex].print();
    printf("\n");
    */
#endif

    /* now that we have the transition point,
       compute the cost from there to the target point and
       add the cost from the target point to the goal
    */
    astarNodes[i].transitionPathCost[j] = getCostOf(astarNodes[i].transitionPoint[j],
						    targetPoint, priorCost,
						    astarNodes[i].cellcost,
						    astarNodes[neighborToGoal].cellcost);
  }
}

const TMAP::obstacleMAP *savedObstacles=NULL;
int TMAP::tmap_astar::astar(int triangleIndex, int maxSteps,
			    const obstacleMAP &obstacles){
  savedObstacles = &obstacles;
  int numSteps=0;
  const int TOO_MANY_STEPS=3*numTriangles()+1;

#if USE_EDGE_SEARCH
  if(triangleIndex<0){
    for(unsigned int i=0;i<numTriangles();i++)
      readpathcost(i, obstacles);
  }else{
    readpathcost(triangleIndex, obstacles);
  }
  return 1;
#endif

#if DEBUG
  if(triangleIndex>=0)
    printf("astar for triangle %d, prior pathcost %f, open list has %d items\n", 
	   triangleIndex, astarNodes[triangleIndex].pathcost, 
	   openList->getsize());
#endif

  if(USE_HEURISTIC && (triangleIndex>=0)){
#if DEBUG
    printf("Updating heuristic costs\n");
    openList->print();
#endif
    /* update the heuristic cost of each */
    for(unsigned int i=0;i<openList->getsize();i++){
      int t = openList->getTriangle(i);
      float h = minCostToTriangle(t, triangleIndex);
      openList->setHeuristic(i, h);
    }
    openList->fixHeap();
#if DEBUG
    printf("Fixed heap:\n");
    openList->print();
#endif
  }

  while(maxSteps<0 || numSteps<maxSteps){
    if(triangleIndex>=0 && astarNodes[triangleIndex].pathcost>=0 &&
       (openList->getsize()==0 || 
	openList->minCost()>=astarNodes[triangleIndex].pathcost)
       ){
#if DEBUG
      printf("Success after %d steps\n", numSteps);
#endif
      return 1;
    }

    numSteps++;
#if DEBUG
    printf("Step %d, astar for triangle %d, open list has %d items\n", 
	   numSteps, triangleIndex, openList->getsize());
#endif

    assert(openList->getsize()>=0);

    if(numSteps>TOO_MANY_STEPS){
      /*
	  printf("Error, infinite loop in search\n");
      writeToFile("infiniteLoop.smf");
      obstacles.writeToFile("obstacles.smf");
      printf("Saved infiniteLoop.smf and obstacles.smf\n");
	   */
      printf("Search from triangle %d\n", triangleIndex);
      printf("Goal at %g,%g (in triangle %d)\n", 
	     RASM_TO_METERS(goalPoint.X()),
	     RASM_TO_METERS(goalPoint.Y()),
	     goalIndex);
      abort();
    }

    /* try to re-seed the open list */
    if(0 == openList->getsize()){
      if(triangleIndex>=0){
#if DEBUG
	printf("triangle %d can't reach the goal, ran out of points!\n",
	       triangleIndex);
#endif
	return 0;
      }
      int minIndex = findRemainingTriangleClosestToGoal();
      if(minIndex<0)break;/* ran out of points! */

      float h = -1;
      if(USE_HEURISTIC){
	h = minCostToTriangle(minIndex, triangleIndex);
	assert(h>=0);
      }
      float c = minCostToGoal(minIndex);
      openList->insert(minIndex, c, -1, h);
      assert(c>0);

#if DEBUG
      printf("Re-seeded with triangle index %d, path cost %f, heuristic %f\n", 
	     minIndex, c, h);
      openList->print();
#endif
    }
    assert(openList->getsize()>0);

    /* evaluate the first item in the open list */
    int ind;
    float cost;
    int cachedNeighbor;
    openList->removeFirst(ind, cost, cachedNeighbor);
#if DEBUG
    printf("astar: index %d cost %f cached pathcost %f from neighbor %d\n",
	   ind, cost, astarNodes[ind].pathcost, cachedNeighbor);
#endif

    /* already have the best path... */
    if((ind != goalIndex) &&
       (astarNodes[ind].pathcost>=0) &&
       (astarNodes[ind].pathcost <= cost))continue;

    /* commit this node by setting it's pathcost and neighbor */
    astarNodes[ind].pathcost = cost;
    astarNodes[ind].neighborToGoal = cachedNeighbor;
#if DEBUG
    printf("Committed element %d, path cost %f", ind, astarNodes[ind].pathcost);
    printf(", best neighbor %d\n", cachedNeighbor);
#endif

    /* if using path smoothing, fill in transition points */
    if(!NO_PATH_SMOOTHING){
      fillInTransitionPoints(ind);
    }

    /* add any untouched neighbors, or ones with worse paths */
    for(int i=0;i<3;i++){
      int addIndex = faces[ind].neighbors[i];

      /* check for no neighbor */
      if(addIndex < 0){
#if DEBUG
	printf("Not adding %d (neighbor[%d] of %d): it doesn't exist\n", 
	       addIndex, i, ind);
#endif
	continue;
      }

      /* check for neighbor better than current node */
      if((astarNodes[addIndex].pathcost>=0) &&
	 (astarNodes[addIndex].pathcost <= astarNodes[ind].pathcost)){
#if DEBUG
	printf("Not adding %d (neighbor[%d] of %d): it is better than current node\n", 
	       addIndex, i, ind);
#endif
	continue;
      }

      /* check for back-and-forth */
      if(astarNodes[ind].neighborToGoal == addIndex){
#if DEBUG
	printf("Not adding %d (neighbor[%d] of %d): it is the neighbor to goal\n", 
	       addIndex, i, ind);
#endif
	continue;
      }

#if DEBUG
      printf("Checking out %d (neighbor[%d] of %d): prior cost %f\n", 
	     addIndex, i, ind, astarNodes[addIndex].pathcost);
#endif

      double c = estimatedCostThroughNeighbor(addIndex, ind);

      /* check for already expanded nodes */
      if(astarNodes[addIndex].pathcost>=0){
	if(astarNodes[addIndex].pathcost<c){
#if DEBUG
	  printf("Not adding %d (neighbor[%d] of %d): already has a better path\n", 
		 addIndex, i, ind);
#endif
	  continue;
	}
#if DEBUG
	printf("Re-adding %d (neighbor[%d] of %d): prior cost %f through triangle %d, new cost %f\n", 
	       addIndex, i, ind, astarNodes[addIndex].pathcost, 
	       astarNodes[addIndex].neighborToGoal, c);
#endif
      }

      if(c <= astarNodes[ind].pathcost){
#if DEBUG
	int n = astarNodes[ind].neighborToGoal;
	printf("Avoiding negative cycle at triangle %d/path cost %f\n",
	       index, astarNodes[ind].pathcost);
	printf("Neighbor %d has an estimated cost of %f\n", addIndex, c);
	printf("Neighbor to goal is %d/path cost %f\n",
	       n, astarNodes[n].pathcost);
#endif
	continue;
      }

      /* check for obstacles */
      if(obstacles.crossesObstacle(centers[ind], centers[addIndex])){
#if DEBUG
	printf("Not adding %d (neighbor[%d] of %d): it crosses an obstacle\n", 
	       addIndex, i, ind);
#endif
	continue;
      }

#if DEBUG
      /* check for cycles... */
      int cycleIndex=ind;
      for(int cycleCount=0;cycleIndex>=0 && cycleCount<1000;cycleCount++){
	/* keep advancing index closer to the goal,
	 * if we ever reach the addIndex then a loop would be formed
	 */
	int nextIndex = astarNodes[cycleIndex].neighborToGoal;
	if(addIndex == nextIndex)break;/* this would create a loop!! */
	if(-1 == nextIndex){/* reached goal */
	  cycleIndex = -1;
	  break;
	}
	/* make sure the pathcost keeps decreasing */
	if(astarNodes[nextIndex].pathcost > astarNodes[cycleIndex].pathcost){
	  printf("Error, triangle %d has pathcost %f\n", 
		 cycleIndex, astarNodes[cycleIndex].pathcost);
	  printf("Neighbor to goal is %d with pathcost %f\n", 
		 nextIndex, astarNodes[nextIndex].pathcost);
	  exit(-1);
	}
	/* we can stop early if the path cost is less than this threshold */
	if(astarNodes[cycleIndex].pathcost < astarNodes[ind].pathcost){
	  cycleIndex = -1;
	  break;
	}
	cycleIndex = nextIndex;
      }
      if(cycleIndex>=0){
	printf("Error, have a cycle at triangle %d when adding triangle %d\n",
	       ind, addIndex);
	exit(-1);
      }
#endif

      /* this neighbor is ok, add it */
      double h = -1;
      if(USE_HEURISTIC && triangleIndex>=0){
	h = minCostToTriangle(addIndex, triangleIndex);
	assert(h>=0);
      }
#if DEBUG
      printf("Adding neighbor of %d: %d path cost %0.5f, heuristic %0.5f\n",
	     index, addIndex, c, h);
#endif
      openList->insert(addIndex, c, ind, h);
      assert(c>0);

#if DEBUG
      printf("\nAfter inserting neighbor %d:\n", addIndex);
      openList->print();
#endif
    }

    if(triangleIndex == ind){
#if DEBUG
      printf("Got an estimate after %d steps but there might be something better\n", numSteps);
#endif
    }
  }
  printf("Out of loop after %d steps\n", numSteps);
  return 0;
}

void TMAP::tmap_astar::evaluatePaths(){
  obstacleMAP o;
  astar(-1, -1, o);
}


float TMAP::tmap_astar::readpathcost(int triangleIndex,
				     const obstacleMAP &obstacles){
#if USE_EDGE_SEARCH
  if(!edgeSearch)transferToEdges();
  return edgeSearch->getPathCost(centers[triangleIndex], 
				 triangleIndex, 
				 obstacles);
#else
  astar(triangleIndex, -1/* no limit on node expansions */, obstacles);
  return astarNodes[triangleIndex].pathcost;
#endif
}

void TMAP::tmap_astar::astarPrint() const{
  assert(centers);
  printf("%d vertices < %d faces\n", 
	 numPoints(), numTriangles());
  printf("Goal at ");
  goalPoint.print();
  printf(" (triangle %d)\n", goalIndex);
  for(unsigned int i=0;i<numPoints();i++){
    printf("Point %d ", i);
    vertices[i].print();
    printf("\n");
  }
  for(unsigned int i=0;i<numTriangles();i++){
    printf("Face %d==%d Points %d,%d,%d with center \n", 
	   i, astarNodes[i].triangleIndex,
	   faces[i].points[0], faces[i].points[1], faces[i].points[2]);

    centers[i].print();
    printf("\n");
  }

  for(unsigned int i=0;i<numTriangles();i++){
    printf("Element %d==%d cost %f", i, astarNodes[i].triangleIndex, astarNodes[i].cellcost);
    printf(", best neighbor %d path %f\n", astarNodes[i].neighborToGoal, astarNodes[i].pathcost);
  }
}

void TMAP::tmap_astar::writeEvaluationToFile(const char *filename){
#if USE_EDGE_SEARCH
  edgeSearch->writeEvaluation(filename);
  return;
#endif
  assert(centers);

  FILE *f = fopen(filename, "w");

  for(unsigned int i=0;i<numTriangles();i++){
    int nextTriangle = astarNodes[i].neighborToGoal;
    float x = RASM_TO_METERS(centers[i].X());
    float y = RASM_TO_METERS(centers[i].Y());

    if(-1 == nextTriangle){
      fprintf(f, "%g %g ? ? ? ?\n", x, y);
    }else{
      float nx = RASM_TO_METERS(centers[nextTriangle].X());
      float ny = RASM_TO_METERS(centers[nextTriangle].Y());
      float cellcost = (float)astarNodes[i].cellcost;
      float pathcost = (float)astarNodes[i].pathcost;
      fprintf(f, "%g %g %g %g %g %g\n", x, y, nx, ny, cellcost, pathcost);
    }
  }

  fclose(f);

}

void TMAP::tmap_astar::writeEvaluationToFile(unsigned int triangleIndex,
					     const RASM::point2d &start,
					     FILE *f){
#if USE_EDGE_SEARCH
  RASM::point3d tri_s[3] = {
    vertices[faces[goalIndex].points[0]],
    vertices[faces[goalIndex].points[1]],
    vertices[faces[goalIndex].points[2]]
  };
  RASM_UNITS h_s = METERS_TO_RASM(getHeightAt(start, tri_s));
  RASM::point3d s(start.X(), start.Y(), h_s);

  edgeSearch->writeEvaluation(s, triangleIndex, f);
  return;
#endif

  assert(centers);
#if DEBUG
  printf("Writing evaluation from %g,%g (in triangle %d)\n",
	 RASM_TO_METERS(start.X()),
	 RASM_TO_METERS(start.Y()),
	 triangleIndex);
#endif

  int nextTriangle = -1;
  int prevTriangle = -1;
  int n=0;
  const int TOO_MANY_STEPS=3*numTriangles()+1;
  while(1){

    if(n++>TOO_MANY_STEPS){
      printf("Error, infinite loop in writeEvaluationToFile()\n");
      writeToFile("infiniteLoop.smf");
      savedObstacles->writeToFile("savedObstacles.smf");
      printf("Saved infiniteLoop.smf and savedObstacles.smf\n");
      printf("Search from %g,%g... now in loop at triangle %d\n",
	     RASM_TO_METERS(start.X()),
	     RASM_TO_METERS(start.Y()),
	     triangleIndex);
      printf("Goal at %g,%g (in triangle %d)\n", 
	     RASM_TO_METERS(goalPoint.X()),
	     RASM_TO_METERS(goalPoint.Y()),
	     goalIndex);
      abort();
    }

    float x = RASM_TO_METERS(centers[triangleIndex].X());
    float y = RASM_TO_METERS(centers[triangleIndex].Y());
    float cellcost = (float)astarNodes[triangleIndex].cellcost;
    float pathcost = (float)astarNodes[triangleIndex].pathcost;

    /* initial segment */
    if(-1 == nextTriangle){
      fprintf(f, "%g %g %g %g %g %g\n", 
	      RASM_TO_METERS(start.X()), RASM_TO_METERS(start.Y()),
	      x, y, cellcost, pathcost);
    }

    nextTriangle = astarNodes[triangleIndex].neighborToGoal;

    if(nextTriangle<0 || nextTriangle>= (int)numTriangles()){
      /* no next triangle, this one is the goal */
      fprintf(f, "%g %g ? ? ? ?\n", x, y);
      break;
    }

    if(NO_PATH_SMOOTHING){
      float nx = RASM_TO_METERS(centers[nextTriangle].X());
      float ny = RASM_TO_METERS(centers[nextTriangle].Y());
      fprintf(f, "%g %g %g %g %g %g\n", x, y, nx, ny, cellcost, pathcost);
    }else{
      /* write out the path following the transition points */
      /* pick the arrival point, 
	 either the center of the goal triangle or the transition point 
      */
      float nx, ny;
      int i=-1;
      if((int)triangleIndex == astarNodes[nextTriangle].neighborFromGoal[0])
	i=0;
      if((int)triangleIndex == astarNodes[nextTriangle].neighborFromGoal[1])
	i=1;
      if(-1==i){/* next triangle is the goal, use the center */
	nx = RASM_TO_METERS(centers[nextTriangle].X());
	ny = RASM_TO_METERS(centers[nextTriangle].Y());
      }else{/* next triangle is not the goal, use the transition point */
	nx = RASM_TO_METERS(astarNodes[nextTriangle].transitionPoint[i].X());
	ny = RASM_TO_METERS(astarNodes[nextTriangle].transitionPoint[i].Y());
      }

      /* pick the departure point,
	 should be the same as the arrival point from the last loop iteration
	 this is either the center of the starting triangle, 
	 or the transition point for arriving from the previous triangle
      */
      i=-1;
      if(prevTriangle == astarNodes[triangleIndex].neighborFromGoal[0])i=0;
      if(prevTriangle == astarNodes[triangleIndex].neighborFromGoal[1])i=1;
      if(-1==i){/* previous triangle does not exist, use the center */
	x = RASM_TO_METERS(centers[triangleIndex].X());
	y = RASM_TO_METERS(centers[triangleIndex].Y());
	pathcost = astarNodes[triangleIndex].pathcost;
      }else{/* use the transition point relevant to the previous triangle */
	x = RASM_TO_METERS(astarNodes[triangleIndex].transitionPoint[i].X());
	y = RASM_TO_METERS(astarNodes[triangleIndex].transitionPoint[i].Y());
	pathcost = astarNodes[triangleIndex].transitionPathCost[i];
      }
      fprintf(f, "%g %g %g %g %g %g\n", x, y, nx, ny, cellcost, pathcost);    
    }/* end of !NO_PATH_SMOOTHING, ie with path smoothing */

    /*
    printf("At triangle %d, came from %d and going to %d\n",
	   triangleIndex, prevTriangle, nextTriangle);
    */

    prevTriangle = (int)triangleIndex;
    triangleIndex = (unsigned int)nextTriangle;
  }

}

float TMAP::tmap_astar::costToTriangleCenter(const RASM::point2d &p, 
					     unsigned int tri) const{
  RASM::point3d pointOnSurface;
  findSurfaceAt(p, pointOnSurface);
  float dist = dist3d(pointOnSurface, centers[tri]);
  return dist*astarNodes[tri].cellcost;
}

int TMAP::tmap_astar::getContainingTriangle(const RASM::point2d &p, 
					    float &costToCenter) const{
  int i = findTriangle2d(p);
  if(-1 == i){
    costToCenter = -1;
    return -1;
  }
  assert(i>=0 && i<(int)numTriangles());
  costToCenter = costToTriangleCenter(p, (unsigned int)i);
  return i;
}


void TMAP::tmap_astar::transferToEdges()const{
#if USE_EDGE_SEARCH
  if(edgeSearch)delete edgeSearch;
  edgeSearch = new edgeDB(numPoints(), vertices, numTriangles(), faces, this);

  RASM::point3d tri[3] = {
    vertices[faces[goalIndex].points[0]],
    vertices[faces[goalIndex].points[1]],
    vertices[faces[goalIndex].points[2]]
  };
  RASM_UNITS h = METERS_TO_RASM(getHeightAt(goalPoint, tri));
  RASM::point3d g(goalPoint.X(), goalPoint.Y(), h);

  edgeSearch->setGoal(g, goalIndex, new TMAP::minHeap());
#endif
}

void TMAP::tmap_astar::writeEdgeEvaluation(const RASM::point3d &p, 
					   int triangle,
					   const char *file)const{
#if USE_EDGE_SEARCH
  edgeSearch->writeEvaluation(p, triangle, file);
#endif
}

void TMAP::tmap_astar::writeEdgeEvaluation(const char *file)const{
#if USE_EDGE_SEARCH
  edgeSearch->writeEvaluation(file);
#endif
}

