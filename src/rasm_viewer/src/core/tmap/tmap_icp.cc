#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <tmap_icp.h>
#include <matrixClass.h>
#include <planeHelpers.h>
#include <tmapKDTree.h>
#define ENABLE_KD 1

#define DEBUG_ICP 0

/* how many overlapping points are needed for ICP... 
 * technically we only need 3 non-colinear points, 
 * but if we only have 3 points we probably don't have very good data
 */
const unsigned int minICPpoints=10;

/* when a vertex is associated with another, how close must the two be?
 */
const double icpNearPointThresh = 0.5;
const double icpNearPointThreshSq = icpNearPointThresh*icpNearPointThresh;

static void setRow(matrix &m, const RASM::point3d &p, unsigned int row){
  for(int i=0;i<3;i++)
    m[row][i] = (double)(p.coord3d[i]);
}

static void setCol(matrix &m, const RASM::point3d &p, unsigned int col){
  for(int i=0;i<3;i++)
    m[i][col] = (double)(p.coord3d[i]);
}

static matrix pointToMatrix(const RASM::point3d &p){
  matrix ret(3,1);
  for(int i=0;i<3;i++)
    ret[i][0] = p.coord3d[i];
  return ret;
}

static RASM::point3d matrixToPoint(const matrix &m){
  RASM::point3d ret;
  for(int i=0;i<3;i++)
    ret.coord3d[i] = (RASM_UNITS)m[i][0];
  return ret;
}

#if DEBUG_ICP
static bool verboseColinear=false;

static bool colinear(unsigned int numPoints,
		     const RASM::point3d *p, bool verbose=false){
  double sumXY=0, sumX=0, sumY=0, sumX2=0, sumY2=0;
  double n = (double)numPoints;

  if(verbose && verboseColinear)printf("\n");
  for(unsigned int i=0;i<numPoints;i++){
    double x = RASM_TO_METERS(p[i].X());
    double y = RASM_TO_METERS(p[i].Y());
    sumXY+= x*y;
    sumX += x;
    sumY += y;
    sumX2 += x*x;
    sumY2 += y*y;
    if(verbose && verboseColinear){
      printf("  %d %d %f %f\n", i, numPoints, x, y);
    }
  }

  double nX = n*sumX2 - sumX*sumX;
  double nY = n*sumY2 - sumY*sumY;
  if(fabs(nX)<1e-3 || fabs(nY)<1e-3){
    if(verbose){
      printf("Horizontal or vertical\n");
    }
    return 1;
  }

  if(verbose){
    double slope = (n*sumXY - sumX*sumY)/nX;
    double intercept = (sumY - slope*sumX)/n;
    printf("Y=%fX + %f\n", slope, intercept);
  }

  assert(nX>0);
  assert(nY>0);

  double r = (n*sumXY - sumX*sumY)/(sqrt(nX)*sqrt(nY));
  assert(fabs(r)<=1.0);
  if(verbose){
    printf("Computed r = %f = (%0.2f*%0.2f - %0.2f^2)/(sqrt(%0.2f)*sqrt(%0.2f))\n", 
	   r, n, sumXY, sumX, nX, nY);
  }

  return (fabs(r)>0.97);
}

static bool colinear(unsigned int numPoints,
		     const RASM::point3d *local,
		     const RASM::point3d *world){
  if(colinear(numPoints, local)){
    printf("Warning, colinear points ");
    colinear(numPoints, local, true);
    /*
    for(unsigned int i=0;i<numPoints;i++){
      printf(" %f,%f  %f vs %f\n", RASM_TO_METERS(local[i].X()),
	     RASM_TO_METERS(local[i].Y()),
	     RASM_TO_METERS(local[i].Z()),
	     RASM_TO_METERS(world[i].Z()));
    }
    */

    return true;
  }
  if(colinear(numPoints, world)){
    printf("error, world points are colinear but local points are not?!\n");
    colinear(numPoints, local, true);
    colinear(numPoints, world, true);
    assert(0);
    return true;
  }
  return false;
}
#endif /* DEBUG_ICP */

#include <sys/time.h>
FILE *ICP_DEBUG_FILE=NULL;
#define debugMSG(msg)                                      \
do{                                                        \
  if(!ICP_DEBUG_FILE)                                      \
    ICP_DEBUG_FILE = fopen("../icpDebugFile.txt", "w");    \
  struct timeval tv;                                       \
  gettimeofday(&tv, NULL);                                 \
  fprintf(ICP_DEBUG_FILE, "%10ld.%06ld %s:%d %s\n",        \
          tv.tv_sec, tv.tv_usec, __FILE__, __LINE__, msg); \
  fflush(NULL);                                            \
}while(0)
#undef debugMSG
#define debugMSG(msg) do{}while(0)

/* 
   0 - ok
   1 - not enough points
   2 - flipped Z
*/
int TMAP::tmap_icp::getTransformation(unsigned int numPoints,
				      const RASM::point3d *local,
				      const RASM::point3d *world,
				      float transform[4][4],
				      bool shouldFindMean,
				      bool shouldRotate,
				      bool shouldTranslate, 
				      bool shouldTranslateXY){
  int flippedZ=0;
  //printf("getTransformation(... %d,%d,%d)\n", shouldFindMean, shouldRotate, shouldTranslate);

  if(numPoints < minICPpoints){
    printf("Warning, need atleast %d points of overlap, have %d\n", 
	   minICPpoints, numPoints);
    for(int i=0;i<4;i++)
      for(int j=0;j<4;j++)
	transform[i][j]=0.0;
    for(int i=0;i<4;i++)
      transform[i][i]=1.0;
    return 1;
  }

  /*
  if(shouldRotate && colinear(numPoints, local, world)){
    printf("icp rotation not possible, points are colinear\n");
    //assert(0);
    shouldRotate = 0;
  }
  */

  /* get the mean of both sets of points */
  RASM::point3d meanA(0,0,0), meanB(0,0,0);
  if(shouldFindMean){
    debugMSG("Computing mean");
    for(unsigned int i=0;i<numPoints;i++){
      meanA+=local[i];
      meanB+=world[i];
    }
    meanA/=(RASM_UNITS)numPoints;
    meanB/=(RASM_UNITS)numPoints;
  }

  matrix rotation = matrix::eye(3);
  if(shouldRotate){
    debugMSG("Prepping matrices");
    /* get the vector to each point from the mean */
    matrix mA(numPoints, 3);
    matrix mB(3, numPoints);
    mA.disableSafetyChecks();
    mB.disableSafetyChecks();
    for(unsigned int i=0;i<numPoints;i++){
      setRow(mA, local[i] - meanA, i);
      setCol(mB, world[i] - meanB, i);
    }

    debugMSG("Matrix multiplication");
    //matrix M = mB * mA;
    matrix M(mB);
    M.disableSafetyChecks();
    M *= mA;

#if DEBUG_ICP
    printf("Multiplied %d by %d times %d by %d, got %d by %d\n", 
	   mB.rows(), mB.cols(), mA.rows(), mA.cols(), M.rows(), M.cols());
#endif


    debugMSG("SVD");
    rotation = M.svdRotation();
    rotation.transpose();
    if(rotation[2][2]<0){
      flippedZ=1;
      for(int i=0;i<3;i++)
	rotation[i][2] = fabs(rotation[i][2]);
    }
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
	if(i==j)
	  rotation[i][j] = (1.0+rotation[i][j])/2.0;
	else
	  rotation[i][j] = rotation[i][j]/2.0;
      }
    }
  }

  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      transform[i][j] = (float)rotation[i][j];




  //right column is the translation, based on the means
  RASM::point3d translation(0,0,0);
  if(shouldTranslate){
    if(!shouldFindMean){/* now compute the means */
      for(unsigned int i=0;i<numPoints;i++){
	meanA+=local[i];
	meanB+=world[i];
      }
      meanA/=(RASM_UNITS)numPoints;
      meanB/=(RASM_UNITS)numPoints;
      printf("Computed means... ");
      meanA.print();
      printf(" and ");
      meanB.print();
      printf("\n");
    }

    translation = meanB - matrixToPoint((rotation*pointToMatrix(meanA)));
    if(!shouldTranslateXY){
      translation.coord3d[0]=0;
      translation.coord3d[1]=0;
    }
  }

  for(int i=0;i<3;i++)
    transform[i][3] = (float)(translation.coord3d[i]);

  //bottom row is 0,0,0,1
  for(int i=0;i<3;i++)
    transform[3][i] = 0.0;
  transform[3][3] = 1.0;

  if(flippedZ)return 2;
  return 0;
}


void TMAP::tmap_icp::associatePoints(const tmap_searchable &world,
				     const RASM::point3d &targetPoint,
				     double maxDistSq,
				     RASM::point3d *interiorPoints, /* points in this map */
				     RASM::point3d *closestPoints,  /* points in the world map */
				     RASM::point3d *strayPoints,    /* points that don't match the world */
				     RASM::point3d *exteriorPoints, /* points outside the world */
				     unsigned int &numInteriorPoints,
				     unsigned int &numStrayPoints,
				     unsigned int &numExteriorPoints,
				     bool shouldFindMean,
				     bool useSurface,
				     bool vertex3d,
				     double distThreshSq, double verticalDist /* distance threshold for stray points */
				     ) const{

  numInteriorPoints=numStrayPoints=numExteriorPoints=0;

  TMAP::kdTree *kd=NULL;
  if(ENABLE_KD){
    if(!useSurface && vertex3d){
      debugMSG("Building KD tree");
      kd = new TMAP::kdTree();
      kd->insert(world.getPoints(), world.numPoints());
    }
  }

  debugMSG("Associating points");
  /* select the points that fall in the world model and their counterparts */
  for(unsigned int i=0;i<numPoints();i++){
    if(!shouldFindMean && i+1 == numPoints())break;/* don't consider the fake point */
    const RASM::point3d p = vertices[i];

    /* skip points too far from the target point */
    if(maxDistSq>0 && distSq2d(p, targetPoint)>maxDistSq){
      exteriorPoints[ numExteriorPoints++ ] = p;
      continue;
    }

    /* skip points outside the world */
    if(-1 == world.findTriangle2d(RASM::point2d(p))){
      exteriorPoints[ numExteriorPoints++ ] = p;
      continue;      
    }

    /* fill in the point p_world that matches p */
    RASM::point3d p_world;
    double distSq=-1;

    if(!useSurface){
      if(vertex3d){
	if(ENABLE_KD){
	  int ind = kd->lookup(p, distSq);
	  p_world = world.getPointRef(ind);
	}else{
	  int ind = (int)world.findClosestVertex(p);
	  p_world = world.getPointRef(ind);
	  distSq = distSq3d(p, p_world);
	}
      }else{
	RASM::point2d p2d(p);
	int ind = (int)world.findClosestVertex(p2d);
	p_world = world.getPointRef(ind);
	distSq = distSq3d(p, p_world);
      }

    }else{/* useSurface */

      int ind = world.findTriangle2d(p);
      if(-1 != ind){
	if(ICP_USE_FAST_SURFACE){
	  if(-1 == world.findSurfaceAt(p, p_world)){
	    assert(0);
	    abort();
	  }
	}else{
	  p_world = world.findClosestSurface(p);
	}

	distSq = distSq3d(p, p_world);
      }

    }

    /* accept or reject p_world */

    if(distSq < distThreshSq){
      closestPoints[numInteriorPoints] = p_world;
      interiorPoints[ numInteriorPoints++ ] = p;

    }else{
      /* might be a stray point 
       * see if altitude was a big part of the error
       */
      const RASM_UNITS distZ = p.Z() - p_world.Z();
      const float distZf = RASM_TO_METERS(distZ);
      if(fabs(distZf) > verticalDist)
	strayPoints[ numStrayPoints++ ] = p;
      else
	exteriorPoints[ numExteriorPoints++ ] = p;

    }
  }

  /* cleanup */
  if(kd)delete kd;
}



void TMAP::tmap_icp::associatePointsBatch(const tmap_searchable &world,
					  const RASM::point3d &targetPoint,
					  double maxDistSq,
					  unsigned int *interiorPoints, /* points in this map */
					  RASM::point3d *closestPoints,  /* points in the world map */
					  unsigned int *strayPoints,    /* points that don't match the world */
					  unsigned int *exteriorPoints, /* points outside the world */
					  unsigned int &numInteriorPoints,
					  unsigned int &numStrayPoints,
					  unsigned int &numExteriorPoints,
					  bool shouldFindMean,
					  bool useSurface,
					  bool vertex3d,
					  double distThreshSq,
					  double verticalDist /* distance threshold for stray points */
					  ) const{

  assert(triangleAssociations);

  numInteriorPoints=numStrayPoints=numExteriorPoints=0;

  const unsigned int N = (shouldFindMean?(numPoints()-1):numPoints());

  /* for each point in this mesh, 
   * identify the containing triangle in the other mesh
   * -1 indicates there is no such triangle
   */
  int *matchedTriangle = (int *)alloca(N*sizeof(int));
  for(unsigned int i=0;i<N;i++)
    matchedTriangle[i] = -2;

  struct entry{
    unsigned int vertexIndex;/* index of vertex in this mesh */
    unsigned int prevVertex;/* index of neighbor that was expanded
		    * use the matchedTriangle array to see where it expanded to
		    */
  };
  struct entry *worklist = (struct entry *)alloca(N*sizeof(struct entry));
  unsigned int queueStart=0;/* where in the array we are */
  unsigned int queueLen=0;/* how many entries are valid */
  int *enqueued = (int *)alloca(N*sizeof(int));/* flag indicating if this vertex is in the queue */
  memset(enqueued, 0, N*sizeof(int));

  int numUnmatched=N;
  while(numUnmatched>0){

    if(0 == queueLen){
      /* find the first vertex that falls in the other mesh
       * (ie, the first one for which findTriangle2d returns a valid triangle
       */
      for(unsigned int i=0;i<N;i++){
	if(-2 != matchedTriangle[i])continue;

	matchedTriangle[i] = world.findTriangle2d(vertices[i]);
	assert(-2 != matchedTriangle[i]);
	numUnmatched--;
	enqueued[i]=1;/* mark that point i does not need to be added again */

	/* can we initialize the queue with this? */
	if(matchedTriangle[i] >= 0){
	  worklist[0].vertexIndex = i;
	  worklist[0].prevVertex = N; /* invalid entry, 
				       * not used because matchedTriangle[i] is already set
				       */
	  queueStart=0;
	  queueLen=1;
	  break;
	}
      }
      if(0 == numUnmatched)break;
    }
    assert(queueLen>0);

    unsigned int vind = worklist[queueStart].vertexIndex;
    assert(enqueued[vind]);

    /* match if necessary */
    if(-2 == matchedTriangle[vind]){
      assert(worklist[queueStart].prevVertex < N);
      unsigned int tind = matchedTriangle[ worklist[queueStart].prevVertex ];
      matchedTriangle[vind] = world.centerSearch(getPoint(vind), tind, 0);
      assert(-2 != matchedTriangle[vind]);
      numUnmatched--;
    }

    /* pop this vertex off */
    queueStart++;
    queueLen--;

    /* see if this vertex falls outside */
    if(-1 == matchedTriangle[vind])continue;

    /* add the neighbors */
    assert(triangleAssociations[vind]);
    for(unsigned int i=0;i<triangleAssociations[vind][0];i++){
      /* add all points of this triangle */
      unsigned int t = triangleAssociations[vind][i+1];
      for(unsigned int j=0;j<3;j++){
	/* this point p shared a triangle with vind, add if needed */
	unsigned int p = faces[t].points[j];
	if(-2 == matchedTriangle[p] && !enqueued[p]){
	  /* append p to the queue */
	  worklist[queueStart+queueLen].vertexIndex = p;
	  worklist[queueStart+queueLen].prevVertex = vind;
	  enqueued[p] = 1; /* mark that point p does not need to be added again */
	  queueLen++;
	}
      }
    }
  }

  for(unsigned int i=0;i<N;i++){
    assert(-2 != matchedTriangle[i]);
  }

  /* use the matched triangles to fill in the arrays */
  for(unsigned int i=0;i<N;i++){
    const RASM::point3d p = vertices[i];

    /* skip points too far from the target point */
    if(maxDistSq>0 && distSq2d(p, targetPoint)>maxDistSq){
      exteriorPoints[ numExteriorPoints++ ] = i;
      continue;
    }

    int t = matchedTriangle[i];
    /* see if we failed to match a triangle for this point */
    if(-1 == t){
      exteriorPoints[ numExteriorPoints++ ] = i;
      continue;
    }
    assert(t>=0);

    /* get p_world, the point in the other mesh that matches this one
     * use the matched triangle t to make this efficient
     */
    RASM::point3d tri[3];
    world.getTrianglePoints(t, tri);
    RASM::point3d p_world;
    double distSq;

    if(useSurface){
      /* evaluate the altitude at the matched triangle */
      p_world = evaluatePoint(p, tri);
      distSq = distSq3d(p, p_world);

    }else if(vertex3d){/* pick the closest triangle point in 3d */
      /* macro accepts a three element array and returns 0,1 or 2 */
#define MIN3_index(arr) (((arr[0]<arr[1]) && (arr[0]<arr[2]))?0:( (arr[1]<arr[2])?1:2  ))
      const float distsq[3] = {distSq3d(p, tri[0]), distSq3d(p, tri[1]), distSq3d(p, tri[2])};
      unsigned int k = MIN3_index(distsq);
      p_world = tri[k];
      distSq = distsq[k];

    }else{/* pick the closest triangle point in 2d */
      const float distsq[3] = {distSq2d(p, tri[0]), distSq2d(p, tri[1]), distSq2d(p, tri[2])};
      p_world = tri[ MIN3_index(distsq) ];
      distSq = distSq3d(p, p_world);
    }

    /* accept or reject p_world */

    if(distSq < distThreshSq){
      closestPoints[numInteriorPoints] = p_world;
      interiorPoints[ numInteriorPoints++ ] = i;

    }else{
      /* might be a stray point 
       * see if altitude was a big part of the error
       */
      const RASM_UNITS distZ = p.Z() - p_world.Z();
      const float distZf = fabs(RASM_TO_METERS(distZ));
      if((distZf > verticalDist) && (distZf > dist2d(p, p_world)))
	strayPoints[ numStrayPoints++ ] = i;
      else
	exteriorPoints[ numExteriorPoints++ ] = i;

    }
  }/* done filling in arrays */

}

/*
  0 - ok
  1 - not enough points
  2 - excessive rotation
  3 - flipped Z
  4 - excessive motion
 */
int TMAP::tmap_icp::icpOneStep(const tmap_searchable &world,
			       const RASM::point3d &targetPoint,
			       double maxDistSq,
			       bool shouldFindMean,  bool shouldRotate, 
			       bool shouldTranslate, bool shouldTranslateXY,
			       bool useSurface,      bool vertex3d,
			       float transform[4][4]) const{
  bool flippedZ=false;
  unsigned int numInteriorPoints, numStrayPoints, numExteriorPoints;

  assert(world.numPoints()>0);

  const unsigned int size = numPoints()*sizeof(RASM::point3d);
  RASM::point3d *interiorPoints=(RASM::point3d *)alloca(size);
  RASM::point3d *closestPoints =(RASM::point3d *)alloca(size);
  RASM::point3d *strayPoints   =(RASM::point3d *)alloca(size);
  RASM::point3d *exteriorPoints=(RASM::point3d *)alloca(size);

#if DEBUG_ICP
  if(maxDistSq>0)
    printf("Only considering points within %f of %f,%f,%f\n",
	   sqrt(maxDistSq),
	   RASM_TO_METERS(targetPoint.X()),
	   RASM_TO_METERS(targetPoint.Y()),
	   RASM_TO_METERS(targetPoint.Z()));
#endif

  associatePoints(world, targetPoint, maxDistSq,
		  interiorPoints, closestPoints,
		  strayPoints, exteriorPoints,
		  numInteriorPoints, numStrayPoints, numExteriorPoints,
		  shouldFindMean, useSurface, vertex3d, 
		  icpNearPointThreshSq,
		  0.0 /* stray vs exterior doesn't matter */);

  debugMSG("Associated points, calling getTransformation()");

  /*!
   * 12/3/2020 note:
   * Before calling @function getTransformation, the 'transform'
   * argument is an arbitrary 4x4 matrix. The function 'fills'
   * that matrix with values corresponding to a transformation
   * matrix with respect to the list of closest points.
   * 
   * Note that @arg interiorPoints is the local mesh; @arg
   * closestPoints is the global mesh.
   */

  //printf("Getting next transformation based on %d of %d points\n", 
  //numInteriorPoints, numPoints());
  int ret = getTransformation(numInteriorPoints, interiorPoints, closestPoints,
			      transform,
			      shouldFindMean, shouldRotate, 
			      shouldTranslate, shouldTranslateXY);

  if(1 == ret)return 1;
  if(2 == ret)flippedZ=true;

  /*!
   * 12/3/2020 note:
   * Now, using the filled @variable 'transform,' check if icp is
   * safe to use w.r.t. excessive motion that would be caused by
   * the transform:
   */
#define MAX_MOTION ((float)(METERS_TO_RASM(1.0)))
  if(fabs(transform[0][3])>MAX_MOTION ||
     fabs(transform[0][3])>MAX_MOTION ||
     fabs(transform[0][3])>MAX_MOTION){
    char buf[1024];
    sprintf(buf, "Got excessive icp motion!\n");
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
	sprintf(buf+strlen(buf), "%f ", transform[i][j]);
      }
      sprintf(buf+strlen(buf), "\n");
    }
    sprintf(buf+strlen(buf), "Replacing with identity transform\n");

    printf("%s", buf);
    //    FILE *f = fopen("badicp.txt", "a");
    //    fprintf(f, "%s", buf);
    //    fclose(f);

    for(int i=0;i<4;i++)
      for(int j=0;j<4;j++)
	transform[i][j] = ((i==j)?1.0:0.0);

    //    writeToFile("local.smf");
    //    world.writeToFile("world.smf");

    return 4;
  }

  /*!
   * In the same vein,  check for excessive rotation:
   */
#define MAX_ROT 0.20
  if(fabs(transform[0][1])>MAX_ROT ||
     fabs(transform[0][2])>MAX_ROT ||
     fabs(transform[1][2])>MAX_ROT ||
     fabs(transform[1][0])>MAX_ROT ||
     fabs(transform[2][0])>MAX_ROT ||
     fabs(transform[2][1])>MAX_ROT){
    char buf[1024];
    sprintf(buf, "Got excessive icp rotation!\n");
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
	sprintf(buf+strlen(buf), "%f ", transform[i][j]);
      }
      sprintf(buf+strlen(buf), "\n");
    }
    sprintf(buf+strlen(buf), "Replacing with identity rotation\n");
    
    printf("%s", buf);
    //    FILE *f = fopen("badicp.txt", "a");
    //    fprintf(f, "%s", buf);
    //    fclose(f);
    
    for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
	transform[i][j] = ((i==j)?1.0:0.0);

    //    writeToFile("local.smf");
    //    world.writeToFile("world.smf");

    return 2;
  }

  if(flippedZ)return 3;
  return 0;
}

/*
 * Return values of ::icp():
  0 - ok
  1 - not enough points
  2 - excessive rotation
  3 - excessive motion
*/
/*!
 * 12/3/2020:
 * Despite the name of the parameter, icp is called BY a global/world
 * mesh; tmap_searchable &world is a LOCAL mesh.
 */
int TMAP::tmap_icp::icp(const tmap_searchable &world,
			unsigned int maxIterations,
			const RASM::point3d &targetPoint,
			double maxDist,
			bool shouldFindMean,  bool shouldRotate,
			bool shouldTranslate, bool shouldTranslateXY,
			const char *fileBase,
			bool useSurface){
#if DEBUG_ICP
  double roll=0.0, pitch=0.0, yaw=0.0;
  TMAP_RASM::point3d trans(0,0,0);
#endif

  const double maxDistSq = maxDist*fabs(maxDist);

  debugMSG("icp start");

  int ret=0;
#if DEBUG_ICP
  char buf[1024];
  //  sprintf(buf, "tmp.icp.%s.world.smf", (fileBase?fileBase:"_"));
  world.writeToFile(buf);
#endif

  for(unsigned int counter=0;0==ret && counter<maxIterations;counter++){
#if DEBUG_ICP
    sprintf(buf, "tmp.icp.%s.%d.smf", (fileBase?fileBase:"_"), counter);
    writeToFile(buf);
#endif

    debugMSG("calling icpOneStep");

    float transform[4][4];
    ret = icpOneStep(world, targetPoint, maxDistSq,
		     shouldFindMean, shouldRotate, 
		     shouldTranslate, shouldTranslateXY, 
		     useSurface, true/* use 3d */, transform);
#if DEBUG_ICP
    moveableTMAP::recoverTransform(transform, trans, roll, pitch, yaw);
    printf("icpOneStep() returned %d rot: %0.1f %0.1f %0.1f trans: %0.3f %0.3f %0.3f\n",
	   ret, roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI,
	   RASM_TO_METERS(trans.X()),
	   RASM_TO_METERS(trans.Y()),
	   RASM_TO_METERS(trans.Z()));
    
    for(int i=0;i<4;i++){
      for(int j=0;j<4;j++){
	printf("   %f", transform[i][j]);
      }
      printf("\n");
    }
#endif

    if(1==ret || 4==ret)debugMSG("icp premature end");
    if(1==ret)return ret;
    if(4==ret)return 3;

    debugMSG("Applying transform");

    /* apply the transformation */
    freeTransform(transform);

    debugMSG("Applied transform");

    if(0 == ret){
      /* check for convergence */
#define MIN_ROT 0.001
      if(fabs(transform[0][2])<MIN_ROT &&
	 fabs(transform[2][0])<MIN_ROT){
	debugMSG("ICP converged");
	//printf("ICP converged after %d of %d iterations\n", counter, maxIterations);
	break;
      }
    }
  }

#if DEBUG_ICP
  sprintf(buf, "tmp.icp.%s.%d.smf", (fileBase?fileBase:"_"), maxIterations);
  writeToFile(buf);
#endif

  debugMSG("icp end");
  return ret;
}



int TMAP::tmap_icp::getTransform(const tmap_searchable &world,
				 float transform[4][4],
				 bool shouldFindMean) const{
  //verboseColinear=true;
  RASM::point3d dummy(0,0,0);
  int ret = icpOneStep(world, 
		       dummy, -1.0,
		       shouldFindMean,
		       true /* should rotate */,
		       true /* should translate */,
		       true, //0 /* should NOT translate in XY */,
		       false /* should NOT use surface */,
		       false /* NOT 3d */,
		       transform /* resulting transform */);
#if DEBUG_IPC
  verboseColinear=false;
#endif
  return ret;
}


void TMAP::tmap_icp::getInteriorAndStrayPoints(const tmap_searchable &world,
					       unsigned int *interiorPoints,
					       unsigned int *strayPoints,
					       unsigned int &numInteriorPoints,
					       unsigned int &numStrayPoints,
					       double nearDist,
					       double verticalDist,
					       bool shouldFindMean)const{

  assert(triangleAssociations);

  RASM::point3d *dummy1 = (RASM::point3d *)alloca(numPoints()*sizeof(RASM::point3d));
  unsigned int *dummy2  = (unsigned int  *)alloca(numPoints()*sizeof(unsigned int));
  unsigned int dummy;

  associatePointsBatch(world, 
		       RASM::point3d(0,0,0), -1,
		       interiorPoints,
		       dummy1,
		       strayPoints,
		       dummy2,
		       numInteriorPoints,
		       numStrayPoints,
		       dummy,
		       shouldFindMean,
		       false/* do NOT use surface */,
		       false/* do NOT use 3d points */,
		       nearDist*nearDist, verticalDist);
}
