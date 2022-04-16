#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <rasm_common_types.h>
#include <tmap_arc.h>
#include <string.h>
#include <util.h>

/*
 * These are global so they can be overridden, for example in main().
 */
unsigned int TMAP::numAxleChecks = 10; /* check this many points between the wheels for clearance */
float TMAP::wheelDist = 0.0;   // read in config file
float TMAP::axleHeight = 0.0;  // read in config file
float TMAP::roverLength = 0.0; // read in config file

/* don't allow roll/pitch over this amount (in radians) */
float TMAP::dangerousRoll = 0*M_PI/180.0;  // read in config file
float TMAP::dangerousPitch = 0*M_PI/180.0; // read in config file

/* roll/pitch under this amount are treated as flat (in radians) */
float TMAP::safeRoll = 0*M_PI/180.0;  // read in config file
float TMAP::safePitch = 0*M_PI/180.0; // read in config file

/* arc evaluation = arc cost * arcToPathFactor + path cost */
float TMAP::arcToPathFactor = 1.5;

/* whether or not intersection tests should be verbose (for debugging) */
bool TMAP::verboseIntersection = false;



void TMAP::tmap_path::createPath(PathModel *path_model,
				 unsigned int path_index, 
				 float pathLength,
				 float pathResolution, 
				 bool forward, 
				 float max_heading_change, 
				 float headingResolution)
{
  assert(pathResolution > 0);
  assert(pathLength > roverLength);
/*
 * Create path model
 */
  path_model->set(path_index, 
		  pathLength, 
		  pathResolution, 
		  forward,
		  max_heading_change,
		  headingResolution, 
		  wheelDist, 
		  roverLength,
		  pointsPerPath);

  //numStepsToSkip = path_interface->m_num_steps_to_skip;
  memcpy(&numStepsToSkip,
	 &path_model->m_num_steps_to_skip, 
	 sizeof(unsigned int));
/*
 * Set Vertices - deep copy
 */
  if(faces){free(faces);faces=NULL;}
  if(vertices){free(vertices);vertices=NULL;}
  resizeTriangles(0, 0);
  //  sizeFaces = capacityFaces = 0;
  resizeVertices(0, 0);
  //  sizeVertices = capacityVertices = 0;

  //sizeVertices = path_model->m_size_vertices;
  //  memcpy(&sizeVertices,
  //	 &path_model->m_size_vertices, 
  //	 sizeof(unsigned int));
  resizeVertices(path_model->m_size_vertices);

  vertices = (RASM::point3d *)malloc(numPoints() * sizeof(RASM::point3d));

  for(unsigned int i = 0;i<numPoints();i++)
    {
      vertices[i] = path_model->m_vertices[i];
    }

  //  memcpy(&sizeFaces,&path_model->m_size_faces, sizeof(unsigned int));
  resizeTriangles(path_model->m_size_faces);

  faces = (RASM::triangle *)malloc(numTriangles() * sizeof(RASM::triangle));
  
  for(unsigned int i = 0;i<numPoints();i++){
    faces[i] = path_model->m_faces[i];
  }
  
  forwardPath = forward;
  
//    printf("radius %f arcLength %f arcResolution %f max_heading_change %f headingResolution %f vertices: %u faces: %u \n",
//    path_model->m_paths[path_index], arcLength, arcResolution, max_heading_change, headingResolution,
//    		  numPoints(), numTriangles());

}


/* Given two points A and B where A is located in triangleIndex
 * project the line between them onto the ground
 * then find the highest point on the ground relative to that line
 * and return clearance between the line and point
 *
 * this value will be positive if the line segment AB is floating
 * above the ground, in this case the clearance value is clamped to
 * the axle height even if the clearance is larger
 *
 * it will be negative if the line intersects the ground,
 * in this case the function will return early instead of looking for
 * the worst spot
 *
 * and will be zero if the ground only touches the line (for example
 * if the end points are touching the ground but straddling a hole
 */
RASM_UNITS TMAP::tmap_path::getMinClearance(const TMAP::tmap_astar &model, 
					    const RASM::point3d &A,
					    const RASM::point3d &B,
					    unsigned int triangleIndex,
					    bool extraWide)const{
  RASM::point3d D = B - A;

  RASM::point3d minAxlePoint(0,0,0);
  RASM_UNITS minclearance = METERS_TO_RASM(axleHeight);

  int tIndex=(int)triangleIndex;

  for(unsigned int i=0;i<numAxleChecks;i++){
    float percent = (((float)i)+1.0) / (((float)numAxleChecks)+1.0);
    if(extraWide)
      percent = (((float)i)-1.0) / (((float)numAxleChecks)-3.0);
    const RASM::point3d p = A + (D*percent);

    RASM::point3d modelPoint;
    if(-1 == model.findSurfaceAt(p, modelPoint, tIndex)){
      printf("Unable to lookup point ");
      p.print();
      printf(" which is %f%% between ", percent);
      A.print();
      printf(" and ");
      B.print();
      printf("\n");
      abort();
    }
    RASM_UNITS clearance = p.Z() - modelPoint.Z();

    /* check for ground intersection */
    if(clearance < 0)
      return clearance;

    if(clearance < minclearance){
      minclearance = clearance;
      minAxlePoint = p;
    }
  }
  return minclearance;
}

/* given 2 points vertices[a] and vertices[b] representing the front wheels
 * and 2 points (tailA, tailB) representing the rear wheels
 * find the cost of the vehicle sitting there
 * returns <0 if it can not be evaluated
 * otherwise returns a value between FREE and OBST
 */
float TMAP::tmap_path::evaluate(const TMAP::tmap_astar &model, 
			       unsigned int a, int &a_tri,
			       unsigned int b, int &b_tri,
			       unsigned int tailA, int &tailA_tri,
			       unsigned int tailB, int &tailB_tri,
			       TMAP::obstacleMAP &obstacles, 
			       bool extraWide, 
			       FILE *debugOutput){

  /* get the locations of the wheel hubs
   * by raising a set height above the front wheels
   */
  RASM::point3d frontA, frontB, rearA, rearB;
  if((-1 == model.findSurfaceAt(vertices[a],     frontA, a_tri    ) ) || 
     (-1 == model.findSurfaceAt(vertices[b],     frontB, b_tri    ) ) ||
     (-1 == model.findSurfaceAt(vertices[tailA], rearA,  tailA_tri) ) || 
     (-1 == model.findSurfaceAt(vertices[tailB], rearB,  tailB_tri) ) ){
    if(debugOutput){
      fprintf(debugOutput, "Unable to evaluate wheels.");
      fprintf(debugOutput, "Front at: ");
      vertices[a].print(debugOutput);
      fprintf(debugOutput, " and ");
      vertices[b].print(debugOutput);
      fprintf(debugOutput, "Rear at: ");
      vertices[tailA].print(debugOutput);
      fprintf(debugOutput, " and ");
      vertices[tailB].print(debugOutput);
      fprintf(debugOutput, "\n");
    }
    return -1.0;
  }
  /* save the 3d locations of the wheel points */
  vertices[a].coord3d[2]     = frontA.Z();
  vertices[b].coord3d[2]     = frontB.Z();
  vertices[tailA].coord3d[2] = rearA.Z();
  vertices[tailB].coord3d[2] = rearB.Z();

  /* save copies of the triangle indices for use below */
  int lrIndex = tailA_tri, rrIndex = tailB_tri;
  frontA.coord3d[2] += METERS_TO_RASM(axleHeight);
  frontB.coord3d[2] += METERS_TO_RASM(axleHeight);
  rearA.coord3d[2]  += METERS_TO_RASM(axleHeight);
  rearB.coord3d[2]  += METERS_TO_RASM(axleHeight);

  /* get the front and rear axle pivots */
  RASM::point3d frontPivot = frontA+frontB;
  RASM::point3d rearPivot  = rearA+rearB;
  frontPivot /= 2;
  rearPivot  /= 2;

  /* get the roll angle */
  RASM::point3d frontD = frontA - frontB;
  float rise = RASM_TO_METERS(frontD.Z());
  float run = sqrt(RASM_TO_METERS(frontD.X())*RASM_TO_METERS(frontD.X()) + 
		   RASM_TO_METERS(frontD.Y())*RASM_TO_METERS(frontD.Y()));
  float rollAngle = atan(rise/run);

  if(fabs(rollAngle)>dangerousRoll){
    if(debugOutput){
      fprintf(debugOutput, "Angle %f deg exceeds max roll %f deg with axle between ", 
	     rollAngle*180.0/M_PI, dangerousRoll*180.0/M_PI);
      frontA.print(debugOutput);
      fprintf(debugOutput, " and ");
      frontB.print(debugOutput);
      fprintf(debugOutput, "\n");
    }
    obstacles.insert(frontA, frontB);
    return OBST;
  }

  /* get the pitch angle */
  rise = RASM_TO_METERS(rearPivot.Z()) - RASM_TO_METERS(frontPivot.Z());
  run = roverLength;
  float pitchAngle = atan(rise/run);

  if(fabs(pitchAngle)>dangerousPitch){
    if(debugOutput){
      fprintf(debugOutput, "Angle %f deg exceeds max pitch %f deg with front pivot at ", 
	     pitchAngle*180.0/M_PI, dangerousPitch*180.0/M_PI);
      frontPivot.print(debugOutput);
      fprintf(debugOutput, " and rear pivot at ");
      rearPivot.print(debugOutput);
      fprintf(debugOutput, "\n");
    }
    obstacles.insert(frontPivot, rearPivot);
    return OBST;
  }

#if 0
  RASM_UNITS minclearance = getMinClearance(model, frontA, frontB, a_tri, extraWide);
#else
  /* check the front axle clearance at a few points */
  int triangleIndex = a_tri;
  RASM_UNITS minclearance = METERS_TO_RASM(axleHeight);
  RASM::point3d minAxlePoint(0,0,0);
  for(unsigned int i=0;i<numAxleChecks;i++){
    float percent = (((float)i)+1.0) / (((float)numAxleChecks)+1.0);
    if(extraWide)
      percent = (((float)i)-1.0) / (((float)numAxleChecks)-3.0);
    RASM::point3d axlePoint = frontD;
    for(int j=0;j<3;j++)axlePoint.coord3d[j] = (RASM_UNITS)(percent*((float)axlePoint.coord3d[j]));
    axlePoint += frontB;

    RASM::point3d modelPoint;
    triangleIndex = -1;

    if(-1 == model.findSurfaceAt(axlePoint, modelPoint, triangleIndex)){
		/* Verbose mode not supported in this file
		 if(verbose){
		 printf("Find Surface At failed\n");
		 printf("At point ");
		 axlePoint.print();
		 printf(" which is %f%% between ", percent);
		 frontA.print();
		 printf(" and ");
		 frontB.print();
		 printf(", skipping\n");
		 model.writeToFile("findSurfaceFailed.obj");
		 writeToFile("arc.obj");
		 printf("Saved findSurfaceFailed.obj and arc.obj");
		 //      abort();
		 }
		 */
		continue;
    }

    RASM_UNITS clearance = axlePoint.Z() - modelPoint.Z();
    if(clearance < minclearance){
      minclearance = clearance;
      minAxlePoint = axlePoint;
    }
  }
#endif

  if(minclearance<=0){
    if(debugOutput){
      fprintf(debugOutput, "Front axle collision between ");
      frontA.print(debugOutput);
      fprintf(debugOutput, " and ");
      frontB.print(debugOutput);
      fprintf(debugOutput, "\n");
    }
    obstacles.insert(frontA, frontB);
    return -1.0;
  }


  /* check the side clearance at the middle of the body
     this the midpoint of the line between the wheels
   */
  RASM::point3d bodyL = frontA + rearA;
  RASM::point3d bodyR = frontB + rearB;
  bodyL/=2;
  bodyR/=2;

  RASM::point3d modelLPoint, modelRPoint;
  if(-1 == model.findSurfaceAt(bodyL, modelLPoint, lrIndex) ||
     -1 == model.findSurfaceAt(bodyR, modelRPoint, rrIndex) ){
    if(debugOutput){
      fprintf(debugOutput, "Unable to evaluate left or right body point ");
      bodyL.print(debugOutput);
      fprintf(debugOutput, " and ");
      bodyR.print(debugOutput);
      fprintf(debugOutput, "\n");
    }
    obstacles.insert(bodyL, bodyR);
    return -1.0;
  }

  RASM_UNITS clearanceL = bodyL.Z() - modelLPoint.Z();
  RASM_UNITS clearanceR = bodyR.Z() - modelRPoint.Z();
  RASM_UNITS bodyClearance = clearanceL;
  if(clearanceR<clearanceL)bodyClearance = clearanceR;

  if(bodyClearance<=0){
    if(debugOutput){
      fprintf(debugOutput, "Body collision, left clearance %f, right clearance %f\n",
	     RASM_TO_METERS(clearanceL), RASM_TO_METERS(clearanceR));
    }
    obstacles.insert(bodyL, bodyR);
    return -1.0;
  }

  if(debugOutput){
    fprintf(debugOutput, "Axle clearance %0.2fm Body clearance %0.2f roll: %0.1fdeg (safe is %0.3f unsafe is %0.3f) pitch: %0.1f deg (safe is %0.3f unsafe is %0.3f)\n",
	    RASM_TO_METERS(minclearance),
	    RASM_TO_METERS(bodyClearance),
	    rollAngle*180.0/M_PI, safeRoll*180.0/M_PI, dangerousRoll*180.0/M_PI, 
	    pitchAngle*180.0/M_PI, safePitch*180.0/M_PI, dangerousPitch*180.0/M_PI);
  }

  /* 1 = safe, 0 = dangerous */

  rollAngle  = fabs(rollAngle);
  pitchAngle = fabs(pitchAngle);

  rollAngle   = ((rollAngle >safeRoll )?(rollAngle  -safeRoll ):0.0);
  pitchAngle  = ((pitchAngle>safePitch)?(pitchAngle -safePitch):0.0);

  /* if the clearance is half the axle height or more, then we're good
   * if the clearance is zero or less, then we'll hit something
   * otherwise scale in between
   */
  float safetyClearance = RASM_TO_METERS(minclearance);
  if(safetyClearance>(axleHeight/2.0))
    safetyClearance=(axleHeight/2.0);

  float safetyClearanceBody = RASM_TO_METERS(bodyClearance);
  if(safetyClearanceBody>(axleHeight/2.0))
    safetyClearanceBody=(axleHeight/2.0);

  float danger[4] = {
    /* how safe is the roll? */
    1.0 - (rollAngle/(dangerousRoll-safeRoll)),
    /* how safe is the pitch? */
    1.0 - (pitchAngle/(dangerousPitch-safePitch)),
    /* how safe is the axle clearance? */
    safetyClearance/(axleHeight/2.0),
    /* how safe is the body clearance? */
    safetyClearanceBody/(axleHeight/2.0)
  };

  /* how dangerous is it overall?  1 = safe, 0 = dangerous */
  float percent = danger[0] * danger[1] * danger[2] * danger[3];

  if(debugOutput){
    fprintf(debugOutput, "overall: %0.3f (1 = good, 0 = bad)", percent);
    fprintf(debugOutput, " roll %0.3f", danger[0]);
    fprintf(debugOutput, " pitch %0.3f", danger[1]);
    fprintf(debugOutput, " axle clearance %0.3f", danger[2]);
    fprintf(debugOutput, " body clearance %0.3f\n", danger[3]);
  }

  assert(percent>=0.0 && percent<=1.0);

  return FREE*(percent) + OBST*(1.0 - percent);
}



float TMAP::tmap_path::evaluate(TMAP::tmap_astar &model,
			       TMAP::obstacleMAP &obstacles,
			       float &costOfArc, float &costAfterArc, 
			       bool arcOnly, bool extraWide,
			       FILE *debugOutput){
  assert(pointsPerPath>0);

  TMAP::astarNode *vals = new TMAP::astarNode[numPoints()];

  const unsigned int numSteps = numPoints()/pointsPerPath;

  if(debugOutput)fprintf(debugOutput, "Evaluating this arc\n");

  /* for each step on the arc, evaluate the cell cost
   * note that numSteps is essentially the vehicle length,
   * so the cell cost is a function of the front and rear wheel positions
   */
  assert(2 == pointsPerPath);
  int a_tri=-1, b_tri=-1, tailA_tri=-1, tailB_tri=-1;
  for(unsigned int i=numStepsToSkip;i<numSteps;i++){
    if(debugOutput)
      fprintf(debugOutput, "Evaluating step %d of %d\n", i, numSteps);
    unsigned int ind = i*pointsPerPath;
    unsigned int tail = (i-numStepsToSkip)*pointsPerPath;
    if(!forwardPath){
      /* traveling backwards */
      unsigned int swap = ind;
      ind = tail;
      tail = swap;
    }
    vals[ind].cellcost = evaluate(model, 
				  ind, a_tri, ind+1, b_tri,
				  tail, tailA_tri, tail+1, tailB_tri,
				  obstacles, 
				  extraWide, debugOutput);
    if(vals[ind].cellcost < 0)vals[ind].cellcost = OBST;
    vals[ind + 1].cellcost = vals[ind].cellcost;/* copy the left wheel's cost to the right */
  }

  float acc = 0;/* accumulated arc cost */
  bool obstacle=false; /* flag indicating we hit an obstacle */
  unsigned int normalSegments=0; /* number of arc segments that evaluated properly */
  unsigned int lastGoodStep=0;
  for(unsigned int i=numStepsToSkip;!obstacle && i<numSteps;i++){
    /* check for obstacles */
    for(unsigned int j=0;j<pointsPerPath;j++)
      if(vals[i*pointsPerPath + j].cellcost >= OBST)
	obstacle=true;
    if(obstacle){
      if(debugOutput)
	fprintf(debugOutput, "Detected an obstacle on step %d\n", i);
      break;
    }

    /* check for gaps */
    bool gap=false;
    for(unsigned int j=0;j<pointsPerPath;j++)
      if(vals[i*pointsPerPath + j].cellcost < FREE)
	gap=true;
    if(gap){
      if(debugOutput)
	fprintf(debugOutput, "Detected a gap on step %d\n", i);
      continue;
    }

    /* normal arc, process it */
    float stepCost=0;
    for(unsigned int j=0;j<pointsPerPath;j++)
      stepCost += vals[i*pointsPerPath + j].cellcost;
    acc += stepCost;
    normalSegments++;
    lastGoodStep = i;
    if(debugOutput)
      fprintf(debugOutput, "On step %d, after %d segments, cost %f, have accumulated %f\n", 
	     i, normalSegments, stepCost, acc);
  }
  delete [] vals;

  if(normalSegments<=0)
    acc=0;
  else
    acc/=(float)(pointsPerPath * normalSegments);
  costOfArc = acc;

  if(arcOnly){
    costAfterArc = -1;
    if(obstacle)return -1;
    return 0;
  }

  /* Evaluate from the end to the goal */

  assert(2==pointsPerPath);
  int indA = lastGoodStep*2+0;
  int indB = lastGoodStep*2+1;

  if(!forwardPath){
    /* traveling backwards */
    indA = (lastGoodStep-numStepsToSkip)*2+0;
    indB = (lastGoodStep-numStepsToSkip)*2+1;
  }

  float pathA = -1, pathB = -1, toCenterA = -1, toCenterB = -1;
  int triA = model.getContainingTriangle(vertices[indA], toCenterA);
  if(triA < 0){
    obstacle = true;
  }else{
    pathA = model.readpathcost(triA, obstacles);
    if(pathA<0)
      obstacle = true;
  }
  int triB = model.getContainingTriangle(vertices[indB], toCenterB);
  if(triB < 0){
    obstacle = true;
  }else{
    pathB = model.readpathcost(triB, obstacles);
    if(pathB<0)
      obstacle = true;
  }
  if(debugOutput){
    model.writeEvaluationToFile(triA, vertices[indA], debugOutput);
    model.writeEvaluationToFile(triB, vertices[indB], debugOutput);
  }
  costAfterArc = (toCenterA + toCenterB + pathA+pathB)/2.0;

  if(obstacle)return -1;

  /*
  printf("Arc evaluation:");
  for(unsigned int i=0;i<numSteps;i++)
    printf(" %0.0f", vals[i*pointsPerArc].cellcost);
  printf(" %0.1f+2.0*%0.1f (avg of last %d)\n", lastPathCost, acc, normalSegments);
  */

  if(debugOutput)fprintf(debugOutput, "Evaluated arc %f and path %f\n", costOfArc, costAfterArc);
  return (costAfterArc + arcToPathFactor*costOfArc);
}



float TMAP::tmap_path::evaluateNear(TMAP::tmap_astar &model,
				   TMAP::obstacleMAP &obstacles, 
				   bool extraWide,
				   FILE *savedEvaluation){

  printf("[%s:%d %f] calling tmap_path::evaluateNear()\n",
	 __FILE__, __LINE__, now());

  assert(2 == pointsPerPath);

  TMAP::astarNode *vals = new TMAP::astarNode[numPoints()];

  const unsigned int numSteps = numPoints()/pointsPerPath;

  /* for each step on the arc, evaluate the cell cost
     note that numSteps is essentially the vehicle length,
     so the cell cost is a function of the front and rear wheel positions
   */
  int a_tri=-1, b_tri=-1, tailA_tri=-1, tailB_tri=-1;
  for(unsigned int i=numStepsToSkip;i<numSteps;i++){
    unsigned int costind = i*pointsPerPath;
    unsigned int ind = i*pointsPerPath;
    unsigned int tail = (i-numStepsToSkip)*pointsPerPath;
    if(!forwardPath){
      /* traveling backwards */
      unsigned int swap = ind;
      ind = tail;
      tail = swap;
    }
    vals[costind].cellcost = evaluate(model, 
				      ind, a_tri, ind+1, b_tri,
				      tail, tailA_tri, tail+1, tailB_tri,
				      obstacles, 
				      extraWide, savedEvaluation);
    if(savedEvaluation){
      fprintf(savedEvaluation, "[arcTmap] Step %d of %d, index %d evaluated to %f\n",
	      i, numSteps, costind, vals[costind].cellcost);
    }
    if(vals[costind].cellcost < 0)vals[costind].cellcost = OBST;
    vals[costind + 1].cellcost = vals[costind].cellcost;/* copy the left wheel's cost to the right */
  }

  float acc = 0;/* accumulated arc cost */
  bool obstacle=false; /* flag indicating we hit an obstacle */
  unsigned int normalSegments=0; /* number of arc segments that evaluated properly */
  unsigned int lastGoodStep=0;
  for(unsigned int i=numStepsToSkip;!obstacle && i<numSteps;i++){
    /* check for obstacles */
    for(unsigned int j=0;j<pointsPerPath;j++)
      if(vals[i*pointsPerPath + j].cellcost >= OBST)
	obstacle=true;
    if(obstacle){
      break;
    }

    /* check for gaps */
    bool gap=false;
    for(unsigned int j=0;j<pointsPerPath;j++)
      if(vals[i*pointsPerPath + j].cellcost < FREE)
	gap=true;
    if(gap){
      printf("Warning, gap on step %d of %d, index %d (%f %f)\n",
	     i, numSteps, i*pointsPerPath + 0,
	     vals[i*pointsPerPath + 0].cellcost,
	     vals[i*pointsPerPath + 1].cellcost);
      continue;
    }

    /* normal arc, process it */
    float stepCost=0;
    for(unsigned int j=0;j<pointsPerPath;j++)
      stepCost += vals[i*pointsPerPath + j].cellcost;
    acc += stepCost;
    normalSegments++;
    lastGoodStep = i;
  }
  delete [] vals;

  if(obstacle)return -1;

  if(normalSegments<=0){
    printf("No normal segments when evaluating %d steps of an arc segment?!\n", 
	   numSteps-numStepsToSkip);
    acc=0;
    assert(0);
    return -1;
  }
  return acc/(float)(pointsPerPath * normalSegments);
}

/* given point p in triangle tri
 * if moving directly to a neighbor is better,
 * then adjust the path and to center costs appropriately
 * if either tri or the existing pathCost is invalid, do nothing
 * returns tri if no change was made
 * if an improvement is made, returns the index of the neighboring triangle
 */
static int adjustFirstStep(const RASM::point3d &p, int tri,
			   TMAP::tmap_astar &model,
			   const TMAP::obstacleMAP &obstacles,
			   float &pathCost, float &toCenterCost){
  if((tri < 0) || (pathCost < 0.0))return tri;
  RASM::triangle t = model.getTriangle(tri);

  /* get the cost of the path and the cost of moving to the start
   * (for each valid neighbor)
   */
  float neighborPath[3] = {-1.0, -1.0, -1.0};
  float toCenter[3] = {-1.0, -1.0, -1.0};
  for(int i=0;i<3;i++){
    int neighborTri = t.neighbors[i];
    if(neighborTri<0)continue;
    neighborPath[i] = model.readpathcost(neighborTri, obstacles);
    if(neighborPath[i] < 0)continue;
    toCenter[i] = model.costToTriangleCenter(p, (unsigned int)neighborTri);
  }

  /* make improvements if possible */
  int ret = tri;
  for(int i=0;i<3;i++){
    if(neighborPath[i] < 0 || toCenter[i] < 0)continue;
    if(pathCost + toCenterCost <= neighborPath[i] + toCenter[i])continue;
    pathCost = neighborPath[i];
    toCenterCost = toCenter[i];
    ret = t.neighbors[i];
  }
  return ret;
}

float TMAP::tmap_path::evaluateFar(TMAP::tmap_astar &model,
				  const obstacleMAP &obstacles,
				  bool extraWide, FILE *savedEvaluation)const{
  assert(2==pointsPerPath);

  const unsigned int numSteps = numPoints()/pointsPerPath;
  unsigned int lastGoodStep=numSteps-1;

  /* get the wheel locations (vertices[indA] and vertices[indB]) */
  int indA = lastGoodStep*2+0;
  int indB = lastGoodStep*2+1;

  if(!forwardPath){
    /* traveling backwards */
    indA = (lastGoodStep-numStepsToSkip)*2+0;
    indB = (lastGoodStep-numStepsToSkip)*2+1;
  }

  float pathA = -1, pathB = -1, toCenterA = -1, toCenterB = -1;
  /* get the cost from the wheel to the center of its triangle */
  int triA = model.getContainingTriangle(vertices[indA], toCenterA);
  int triB = model.getContainingTriangle(vertices[indB], toCenterB);
  /* get the cost from that triangle to the goal */
  if(triA >= 0)pathA = model.readpathcost(triA, obstacles);
  if(triB >= 0)pathB = model.readpathcost(triB, obstacles);

  /* go straight to the neighbor if that's cheaper */
  triA = adjustFirstStep(vertices[indA], triA,
			 model, obstacles,
			 pathA, toCenterA);
  triB = adjustFirstStep(vertices[indB], triB,
			 model, obstacles,
			 pathB, toCenterB);

  /* save those paths to a file */
  if(savedEvaluation){
    model.writeEvaluationToFile(triA, vertices[indA], savedEvaluation);
    model.writeEvaluationToFile(triB, vertices[indB], savedEvaluation);
  }

  /* one side unreachable */
  if((pathA<0) || (pathB<0))return -1;

  /* average the costs */

//  printf("toCenterA: %f toCenterB: %f pathA: %f pathB: %f /2 = %f\n",
//		  toCenterA, toCenterB, pathA,pathB, (toCenterA + toCenterB + pathA+pathB)/2.0);
  return (toCenterA + toCenterB + pathA+pathB)/2.0;
}

/* fill in the vehicle position and yaw at the end of the arc */
void TMAP::tmap_path::getFinalPose(RASM::point3d &p, float &yaw) const{
  /* get the position of the front and rear pivots */
  assert(2 == pointsPerPath);
  assert(numStepsToSkip>=2);
  assert(numPoints()>=2);
  unsigned int frontInd = numPoints()-2;
  unsigned int rearInd  = numPoints()-(2*numStepsToSkip);

  if(!forwardPath){
    unsigned int swap = frontInd;
    frontInd = rearInd;
    rearInd = swap;
  }

  RASM::point3d frontPivot = vertices[frontInd] + vertices[frontInd+1];
  RASM::point3d rearPivot  = vertices[rearInd]  + vertices[rearInd+1];
  frontPivot /= 2;
  rearPivot  /= 2;

  /* report the final position and yaw as a function of the pivots */
  p = frontPivot + rearPivot;
  p /= 2;
  yaw = atan2(RASM_TO_METERS(frontPivot.Y() - rearPivot.Y()),
	      RASM_TO_METERS(frontPivot.X() - rearPivot.X())) - M_PI/2.0;
}


/* gets the distance and bearing of point p relative to the center c */
static void distBearing(const RASM::point2d &p, const RASM::point2d &c,
			double &bearing, double &dist){
  const double dx = RASM_TO_METERS(p.X() - c.X());
  const double dy = RASM_TO_METERS(p.Y() - c.Y());
  bearing = atan2(dy, dx);
  dist = sqrt(dx*dx + dy*dy);
}

/* given two points and radii,
 * find the center point (where the radius is 0)
 * from the center get the distance to each point
 * also get the bearing to the two points and the half angle spanning the radii
 */
static void getCenterBearingAndDist(const RASM::point2d &p0, double r0,
				    const RASM::point2d &p1, double r1,
				    RASM::point2d &c,
				    double &d0, double &d1,
				    double &bearing, double &halfAngle){

  assert((r1>r0) && (r0>0.0));

  /* r0= 0       -> c=p0
   * r0=0.5*r1   -> c=p0 - (p1-p0)/1
   * r0=0.25*r1  -> c=p0 - (p1-p0)/3
   * r0=0.125*r1 -> c=p0 - (p1-p0)/7
   */
  const double p = r1/r0;
  assert(p>1.0);

  c = p0 - ((p1-p0) * (1.0/(p-1.0)));

  distBearing(p0, c, bearing, d0);
  distBearing(p1, c, bearing, d1);

  assert(d1>d0);

  halfAngle = atan(r1/d1);
}

/* returns 1 if this arc intersects a circle or ray */
bool TMAP::tmap_path::intersectCircle(const RASM::point2d &p,
				     double radius)const{

  assert(radius > 0.0);

  const double radSq = radius*radius;
  for(unsigned int i=0;i<numPoints();i++){
    if(distSq2d(p, vertices[i]) < radSq){
      if(verboseIntersection){
	printf("tmap_arc::intersectCircle... intersects on point %d\n", i);
      }

      return true;
    }
  }

  if(verboseIntersection){
    printf("tmap_arc::intersectCircle... no intersection\n");
  }

  return false;
}

bool TMAP::tmap_path::intersectRay(const RASM::point2d &p0, double r0,
				  const RASM::point2d &p1, double r1)const{

  assert((r1 > r0) && (r0>0.0));

  RASM::point2d c;
  double d0, d1, bearing, halfAngle;
  getCenterBearingAndDist(p0, r0, p1, r1, c, d0, d1, bearing, halfAngle);
  assert(d1>d0);

  if(verboseIntersection){
    printf("tmap_arc::intersectRay  dists %0.2f,%0.2f bearing %0.1f halfAngle %0.1f\n",
	   d0, d1, bearing*180.0/M_PI, halfAngle*180.0/M_PI);
  }


  /* check if any point falls inside the ray */
  for(unsigned int i=0;i<numPoints();i++){
    double thisBearing, thisDist;
    distBearing(vertices[i], c, thisBearing, thisDist);

    if(verboseIntersection){
      printf("tmap_arc::intersectRay  point %d dist %0.2f bearing %0.1f\n",
	     i, thisDist, thisBearing*180.0/M_PI);
    }


    if((thisDist<d0) || (thisDist>d1))continue;
    if(fabs(thisBearing - bearing) > halfAngle)continue;

    if(verboseIntersection){
      printf("tmap_arc::intersectRay  intersects on point %d\n", i);
    }

    return true;
  }

  if(verboseIntersection){
    printf("tmap_arc::intersectRay  no intersection, checking endpoints\n");
  }

  /* check endpoints */
  return (intersectCircle(p0, r0) || intersectCircle(p1, r1));
}

TMAP::tmap_path::tmap_path()
 : tmap_moveable()
{ 
}

