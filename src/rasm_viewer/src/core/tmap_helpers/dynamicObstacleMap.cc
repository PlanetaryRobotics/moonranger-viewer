#include <dynamicObstacleMap.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <string.h>
#include <rasm_common_types.h>

/* when we stop seeing an obstacle,
 * how long to wait before throwing it out (in seconds)
 */
const double dynamicObstacleDecay = 3.0;

/* max speed of an obstacle (m/s)
 * can observe velocities up to twice this
 */
const double maxObstacleVelocity = 2.0;

/* max size of an obstacle (m)
 * can observe sizes up to twice this
 */
const double maxObstacleRadius = 0.5;

/* grow the radius as a speed that this percent of the velocity */
const double radiusExpansionFactor = 0.25;

/* grow the radius atleast this fast (m/s) */
const double minRadiusExpansionSpeed = 0.1;

/*** end of constants ***/

TMAP::dynamicObstacleMAP::dynamicObstacleMAP()
  :data(NULL), numObstacles(0),
   newData(NULL), numObservations(0),
   curTime(0.0){}

TMAP::dynamicObstacleMAP::~dynamicObstacleMAP(){
  clearObstaclesAndData();
}

/* following observations will have this time */
void TMAP::dynamicObstacleMAP::startObservations(double t){
  assert(!newData && (0 == numObservations));
  curTime = t;
}

/* observed an obstacle at this point, with this size and this many points */
void TMAP::dynamicObstacleMAP::addObservation(const RASM::point3d &p, 
					      const RASM_UNITS r,
					      unsigned int N){

  /* enforce a limit */
  if(RASM_TO_METERS(r) > 2.0*maxObstacleRadius)return;

  newData = (struct observation *)realloc(newData,
					  sizeof(TMAP::dynamicObstacleMAP::observation)*(numObservations+1));
  newData[numObservations].p = p;
  newData[numObservations].r = r;
  newData[numObservations].N = N;
  newData[numObservations].t = curTime;
  ++numObservations;
}

void TMAP::dynamicObstacleMAP::appendNewObstacle(const TMAP::dynamicObstacleMAP::observation *o){
  data = (TMAP::dynamicObstacleMAP::dynamicObstacle *)realloc(data, sizeof(TMAP::dynamicObstacleMAP::dynamicObstacle)*(numObstacles+1));
  memcpy(&(data[numObstacles].cur), o, sizeof(struct observation));
  data[numObstacles].prev.N = 0;
  ++numObstacles;
}

/* an estimate of how well this dynamic obstacle matches an observation */
static float getMatchScore(const TMAP::dynamicObstacleMAP::dynamicObstacle &A,
			   const TMAP::dynamicObstacleMAP::observation &B){

  /* get the change in position, time and size */
  const double dist = dist2d(B.p, A.cur.p);
  const double dt = B.t - A.cur.t;
  const double dr = fabs(RASM_TO_METERS(B.r - A.cur.r));

  assert(dt > 0.0);

  const double maxV = 2.0*maxObstacleVelocity;/* max observed velocity (m/s) */
  const double maxR = maxV*radiusExpansionFactor;/* max change in size (m) */

  if(dist/dt > maxV)return -1.0;
  if(dr      > maxR)return -1.0;

  /* if the obstacle was only seen once, just use proximity */
  if(0 == A.prev.N)return dist+dr;

  /* if the obstacle was tracked, add a linear motion estimate */
  const double prevDist = dist2d(B.p, A.prev.p);
  const double prevDT = B.t - A.prev.t;
  const double dv = fabs(dist/dt - prevDist/prevDT);

  return dist+dr+dv;
}

/* given an observation, give each obstacle a score,
 * then fill in a ranked list of obstacles
 */
static void fillRanking(float *score, int *rank,
			unsigned int N,
			const TMAP::dynamicObstacleMAP::dynamicObstacle *dyn,
			const TMAP::dynamicObstacleMAP::observation &obs){

  /* fill in the scores */
  for(unsigned int i=0;i<N;i++)
    score[i] = getMatchScore(dyn[i], obs);

  /* create a ranking by sorting the scores */
  for(unsigned int i=0;i<N;i++)
    rank[i] = (score[i]<0.0)?-1:((int)i);

  for(unsigned int i=0;i<N;i++){
    for(unsigned int j=i+1;j<N;j++){
      if(rank[j] < 0)continue;

      if((rank[i] < 0) || (score[ rank[j] ] < score[ rank[i] ])){
	/* swap i and j */
	unsigned int tmp = rank[i];
	rank[i] = rank[j];
	rank[j] = tmp;
      }
    }
    /* check if the current (and therefore rest) obstacle was not valid */
    if(-1 == rank[i])break;
  }
}



/* scans for unmatched observations and matches them with obstacles
 * the obstacles can be either unmatched or matched
 * returns true if any change was made
 */
bool TMAP::dynamicObstacleMAP::greedyMatch(const float *matchScore,
					   const int *matchRank,
					   int *obstacleNumber,
					   int *observationNumber){
  bool madeChange=0;
  for(unsigned int i=0;i<numObservations;i++){
    /* check if this observation is already matched */
    if(-1 != obstacleNumber[i])continue;

    /* go through the rank list until a useable match is found */
    for(unsigned int j=0;j<numObstacles;j++){
      /* the next best obstacle is o */
      const int o = matchRank[i*numObstacles + j];

      /* no more possibilities */
      if(-1 == o)break;

      assert((o < (int)numObstacles) && (o>=0));

      /* obstacle o is associated with observation k */
      const int k = observationNumber[o];

      /* if obstacle o is unused, claim it */
      if(-1 == k){
	observationNumber[o] = i;
	obstacleNumber[i] = o;
	madeChange = true;
	break;
      }

      assert((k < (int)numObservations) && (k>=0));

      /* obstacle o is already matched to observation k
       * there is still a chance this observation might be paired up
       */

      if((j+1 == numObstacles) || (-1 == matchRank[i*numObstacles + j + 1])){
	/* this is the last possibility for observation i
	 * allow observation i to claim obstacle o if:
	 *  - it has a better match
	 *  - observation k has another unused possibility
	 */

	/* check if observation i is a better match than observation k */
	bool shouldSwap = (matchScore[i*numObstacles + o] < 
			   matchScore[k*numObstacles + o] );

	/* check if observation k can be moved */
	for(unsigned int j2=0; !shouldSwap && (j2 < numObstacles); j2++){
	  const int o2 = matchRank[k*numObstacles + j2];
	  if(-1 == o2)break;
	  assert((o2 < (int)numObstacles) && (o2>=0));
	  if(-1 == observationNumber[o2])shouldSwap = true;
	}

	if(shouldSwap){
	  obstacleNumber[k] = -1;
	  obstacleNumber[i] = o;
	  observationNumber[o] = i;
	  madeChange = true;
	}

	break;	
      }
    }
  }

  return madeChange;
}


/* This function attempts to match obstacles to observations
 * there must be atleast one of each
 *
 * How well observation i and obstacle h match is stored as:
 *  matchScore[i*numObstacles + j]
 * A negative score indicates this observation can not be for this obstacle
 *
 * Each observation has a ranked list of obstacles.
 * The jth best match for observation i is stored as:
 *  matchRank[i*numObstacles + j]
 * A negative obstacle number means there are no more potential matches
 *
 * Output:
 * obstacleNumber[j] is the index of the obstacle
 * A negative index indicates this observation is a new obstacle
 * No obstacle will be matched more than once (some may be unmatched)
 */
void TMAP::dynamicObstacleMAP::matchObservations(const float *matchScore,
						 const int *matchRank,
						 int *obstacleNumber){

  int *observationNumber = (int *)malloc(numObstacles*sizeof(int));
  assert(observationNumber);

  /* initially mark each observation and obstacle as unmatched */
  for(unsigned int i=0;i<numObservations;i++)obstacleNumber[i] = -1;
  for(unsigned int i=0;i<numObstacles;i++)observationNumber[i] = -1;

  for(unsigned int i=0;i<numObservations;i++)
    if(!greedyMatch(matchScore, matchRank,
		    obstacleNumber, observationNumber))
      break;

  free(observationNumber);
}


/* finishes adding observations and attempts to match up with previous */
void TMAP::dynamicObstacleMAP::endObservations(){
  /* check if there's anything to do */
  if(0 == numObservations)return;

  /* check if we blindly accept everything */
  if(0 == numObstacles){
    for(unsigned int i=0;i<numObservations;i++)
      appendNewObstacle(&(newData[i]));

    /* cleanup temporary data */
    numObservations = 0;
    free(newData);
    newData = NULL;

    return;
  }

  /* match observations in newData[] with existing in data[].cur */

  /* rate how well each observation matches each existing obstacle
   * and create an ordered ranking of obstacles for each observation
   */
  float *matchScore = (float *)malloc(numObstacles*numObservations*sizeof(float));
  int *matchRank = (int *)malloc(numObstacles*numObservations*sizeof(int));
  assert(matchScore && matchRank);

  for(unsigned int i=0;i<numObservations;i++)
    fillRanking(&(matchScore[i*numObstacles]),
		&(matchRank[i*numObstacles]),
		numObstacles,
		data,
		newData[i]);
			  
  /* find the best correlation for each existing obstacle */
  int *obstacleNumber = (int *)malloc(numObservations*sizeof(int));
  assert(obstacleNumber);
  matchObservations(matchScore, matchRank, obstacleNumber);

  /* update the obstacles,
   * by adding new ones or updating the observations of existing ones
   */
  for(unsigned int i=0;i<numObservations;i++){
    const int o = obstacleNumber[i];
    if(-1 == o){
      /* new obstacle */
      appendNewObstacle(&(newData[i]));
    }else{
      assert((o < (int)numObstacles) && (o>=0));

      /* matches an existing obstacle, update the current observation */
      memcpy(&(data[o].prev), &(data[o].cur), sizeof(struct observation));
      memcpy(&(data[o].cur),  &(newData[i]),  sizeof(struct observation));
    }
  }

  /* cleanup temporary data */
  free(matchScore);
  free(matchRank);
  free(obstacleNumber);
  numObservations = 0;
  free(newData);
  newData = NULL;
}

/* checks if the line between these points is free of moving obstacles
 * for some time interval (in seconds)
 */
bool TMAP::dynamicObstacleMAP::crossesObstacle(const RASM::point3d &a,
					       const RASM::point3d &b,
					       double t0, double t1) const{
  assert(0);
}

/* uses linear interpolation to construct an obesrvation at time t
 */
static void predictObservation(TMAP::dynamicObstacleMAP::observation &o,
			       double t,
			       const TMAP::dynamicObstacleMAP::observation &a,
			       const TMAP::dynamicObstacleMAP::observation &b){
  /* make sure the observations are ordered correctly and vary significantly */
  assert(a.t > b.t+0.1);

  /* p=1 -> output a
   * p=0 -> output b
   */
  const double p = (t-b.t)/(a.t-b.t);
  assert(p>=1.0);/* should not be in the past */

  /* interpolate/extrapolate the x/y position */
  for(int i=0;i<2;i++)
    o.p.coord3d[i] = (RASM_UNITS)((  p  )*((double)a.p.coord3d[i]) +
				  (1.0-p)*((double)b.p.coord3d[i]) );
  o.p.coord3d[2] = a.p.coord3d[2];/* copy the altitude */

  /* get a velocity */
  double v = dist2d(a.p, b.p)/(a.t-b.t);
  if(v >= maxObstacleVelocity)v=maxObstacleVelocity;/* cap the velocity */

  /* get the epansion velocity */
  double rv = v*radiusExpansionFactor;
  if(rv<minRadiusExpansionSpeed)rv = minRadiusExpansionSpeed;

  /* expand the radius based on the expansion velocity */
  o.r = a.r + METERS_TO_RASM((t-a.t) * rv);
  o.N = 0;
  o.t = t;

  /* cap the radius */
  if(o.r>METERS_TO_RASM(2.0*maxObstacleRadius))
    o.r = METERS_TO_RASM(2.0*maxObstacleRadius);
}

/* converts all the dynamic obstacles into static ones
 * predict motion to time t
 */
void TMAP::dynamicObstacleMAP::exportStatic(TMAP::obstacleMAP &staticObstacles,
					    double t)const{
  for(unsigned int i=0;i<numObstacles;i++){

    assert(data[i].cur.N>0);
    assert(t > data[i].cur.t);

    /* if tracking the obstacle do some sort of interpolation */
    if(data[i].prev.N>0){
      TMAP::dynamicObstacleMAP::observation o;
      predictObservation(o, t, data[i].cur, data[i].prev);

      printf("Exporting tracked obstacle %d with radii: %0.2f %0.2f\n",
	     i, RASM_TO_METERS(data[i].cur.r), RASM_TO_METERS(o.r));

      printf("Obstacle observated at %0.2f,%0.2f (radius=%0.2f) -> %0.2f,%0.2f (radius=%0.2f)\n",
	     RASM_TO_METERS(data[i].prev.p.X()),
	     RASM_TO_METERS(data[i].prev.p.Y()),
	     RASM_TO_METERS(data[i].prev.r),
	     RASM_TO_METERS(data[i].cur.p.X()),
	     RASM_TO_METERS(data[i].cur.p.Y()),
	     RASM_TO_METERS(data[i].cur.r));
      printf("Obstacle predicted to %0.2f,%0.2f (radius=%0.2f) -> %0.2f,%0.2f (radius=%0.2f)\n",
	     RASM_TO_METERS(data[i].cur.p.X()),
	     RASM_TO_METERS(data[i].cur.p.Y()),
	     RASM_TO_METERS(data[i].cur.r),
	     RASM_TO_METERS(o.p.X()),
	     RASM_TO_METERS(o.p.Y()),
	     RASM_TO_METERS(o.r));

      /* use observations data[i].cur and o to create a static representation
       * eg, some line segments along the direction of travel
       */

      /* get the motion vector and a unit vector perpendicular to it */
      const RASM::point3d vec = o.p - data[i].cur.p;
      const double d = sqrt(RASM_TO_METERS(vec.X())*RASM_TO_METERS(vec.X())+
			    RASM_TO_METERS(vec.Y())*RASM_TO_METERS(vec.Y()));
      const RASM::point3d para(METERS_TO_RASM(RASM_TO_METERS(vec.X())/d),
			       METERS_TO_RASM(RASM_TO_METERS(vec.Y())/d),
			       0);
      const RASM::point3d perp(-para.Y(), para.X(), 0);

      printf("Parallel: %0.2f,%0.2f  Perpendicular: %0.2f,%0.2f\n",
	     RASM_TO_METERS(para.X()),
	     RASM_TO_METERS(para.Y()),
	     RASM_TO_METERS(perp.X()),
	     RASM_TO_METERS(perp.Y()));

      /* radius of each circle, perpendicular and parallel to motion vector */
      const RASM::point3d perp0 = perp * RASM_TO_METERS(data[i].cur.r);
      const RASM::point3d para0 = para * RASM_TO_METERS(data[i].cur.r);
      const RASM::point3d perp1 = perp * RASM_TO_METERS(o.r);
      const RASM::point3d para1 = para * RASM_TO_METERS(o.r);

      printf("Extend to %0.2f,%0.2f and  %0.2f,%0.2f\n",
	     RASM_TO_METERS((data[i].cur.p + para0).X()),
	     RASM_TO_METERS((data[i].cur.p + para0).Y()),
	     RASM_TO_METERS((o.p           - para1).X()),
	     RASM_TO_METERS((o.p           - para1).Y()));

      /* insert 2 line segments parallel to the travel */
      staticObstacles.insert(data[i].cur.p + perp0, o.p + perp1, 1);
      staticObstacles.insert(data[i].cur.p - perp0, o.p - perp1, 1);
      /* insert a cap at the current observation data[i].cur */
      staticObstacles.insert(data[i].cur.p + perp0, data[i].cur.p - para0, 1);
      staticObstacles.insert(data[i].cur.p - perp0, data[i].cur.p - para0, 1);
      /* insert a cap at the predicted observation o */
      staticObstacles.insert(o.p + perp1, o.p + para1, 1);
      staticObstacles.insert(o.p - perp1, o.p + para1, 1);


    }else{

      /* not tracking, create some sort of worse case circle */
      const double r = RASM_TO_METERS(data[i].cur.r) +
	(t - data[i].cur.t)*minRadiusExpansionSpeed;

      printf("Exporting untracked obstacle %d with radius: %0.2fm = %0.2f + %0.2f*%0.2f*%0.2f\n",
	     i, r,
	     RASM_TO_METERS(data[i].cur.r), (t - data[i].cur.t), 
	     maxObstacleVelocity, radiusExpansionFactor);

      const unsigned int ringN = 8;
      for(unsigned int j=0;j<ringN;j++){
	const double theta0 = 2.0*M_PI*((double)j)/((double)ringN);
	const double theta1 = 2.0*M_PI*((double)((j+1)%ringN))/((double)ringN);

	const RASM::point3d a(METERS_TO_RASM(r*sin(theta0)),
			      METERS_TO_RASM(r*cos(theta0)),
			      0);
	const RASM::point3d b(METERS_TO_RASM(r*sin(theta1)),
			      METERS_TO_RASM(r*cos(theta1)),
			      0);

	staticObstacles.insert(data[i].cur.p+a, data[i].cur.p+b, 1);
      }
    }

  }
}


void TMAP::dynamicObstacleMAP::writeToFile(const char *filename)const{
  FILE *f = fopen(filename, "w");
  assert(f);

  printf("Saving %d obstacles to %s\n", numObstacles, filename);fflush(NULL);

  for(unsigned int i=0;i<numObstacles;i++){
    assert(data);

    printf("Saving obstacle %d, cur %d, radius %0.2fm\n",
	   i, data[i].cur.N, RASM_TO_METERS(data[i].cur.r));
    fflush(NULL);

    unsigned int ringN = data[i].cur.N*2;
    for(unsigned int j=0;j<ringN;j++){
      const double theta = 2.0*M_PI*((double)j)/((double)ringN);
      fprintf(f, "%0.3f %0.3f %0.3f\n", 
	      RASM_TO_METERS(data[i].cur.p.X()) + 
	      RASM_TO_METERS(data[i].cur.r) * sin(theta),
	      RASM_TO_METERS(data[i].cur.p.Y()) + 
	      RASM_TO_METERS(data[i].cur.r) * cos(theta),
	      RASM_TO_METERS(data[i].cur.p.Z()));
      fprintf(f, "%0.3f %0.3f %0.3f\n", 
	      RASM_TO_METERS(data[i].cur.p.X()) + 
	      RASM_TO_METERS(data[i].cur.r) * 0.5 * sin(theta),
	      RASM_TO_METERS(data[i].cur.p.Y()) + 
	      RASM_TO_METERS(data[i].cur.r) * 0.5 * cos(theta),
	      RASM_TO_METERS(data[i].cur.p.Z()));
    }

    if(data[i].prev.N > 0){
      printf("Saving obstacle %d, prev %d, radius %0.2fm\n",
	     i, data[i].prev.N, RASM_TO_METERS(data[i].prev.r));
      fflush(NULL);
    }

    ringN = data[i].prev.N*2;
    for(unsigned int j=0;j<ringN;j++){
      const double theta = 2.0*M_PI*((double)j)/((double)ringN);
      fprintf(f, "%0.3f %0.3f %0.3f\n", 
	      RASM_TO_METERS(data[i].prev.p.X()) + 
	      RASM_TO_METERS(data[i].prev.r) * sin(theta),
	      RASM_TO_METERS(data[i].prev.p.Y()) +
	      RASM_TO_METERS(data[i].prev.r) * cos(theta),
	      RASM_TO_METERS(data[i].prev.p.Z()));
    }

  }
  fclose(f);
}

void TMAP::dynamicObstacleMAP::readFromFile(const char *filename){
  assert(0);
}

void TMAP::dynamicObstacleMAP::prune(double t){
  unsigned int i=0;
  while(i<numObstacles){
    /* check the time of obstacle i */
    if(t < data[i].cur.t + dynamicObstacleDecay){
      i++;
      continue;
    }

    /* obstacle i is stale, overwrite it with the last obstacle */
    --numObstacles;
    memcpy(&(data[i]), &(data[numObstacles]), sizeof(TMAP::dynamicObstacleMAP::dynamicObstacle));
  }
}

/* checks if this arc intersects any obstacle to time t
 * returns true if there is an intersection
 */
extern int verboseIntersection;
bool TMAP::dynamicObstacleMAP::checkArc(const TMAP::multiPath &arc,
					double t)const{
  for(unsigned int i=0;i<numObstacles;i++){

    assert(data[i].cur.N>0);
    assert(t > data[i].cur.t);

    /* if tracking the obstacle do some sort of interpolation */
    if(data[i].prev.N>0){

      if(verboseIntersection){
	printf("Checking tracked obstacle %d of %d\n", i, numObstacles);
      }

      struct observation o;
      predictObservation(o, t, data[i].cur, data[i].prev);
      assert(o.r > data[i].cur.r);
      if(arc.intersectRay(data[i].cur.p, RASM_TO_METERS(data[i].cur.r),
			  o.p, RASM_TO_METERS(o.r)))
	return true;

    }else{

      if(verboseIntersection){
	printf("Checking untracked obstacle %d of %d\n", i, numObstacles);
      }

      /* not tracking, create some sort of worse case circle */
      const double r = RASM_TO_METERS(data[i].cur.r) +
	(t - data[i].cur.t)*minRadiusExpansionSpeed;

      if(arc.intersectCircle(data[i].cur.p, r))
	return true;

    }
  }

  if(verboseIntersection){
    printf("Checked all obstacles, no intersection\n");
  }

  /* no intersection */
  return false;
}

void TMAP::dynamicObstacleMAP::clearObstaclesAndData(){
  if(data){
    free(data);
    data = NULL;
  }
  numObstacles = 0;

  if(newData){
    free(newData);
    newData = NULL;
  }
  numObservations = 0;
}

void TMAP::dynamicObstacleMAP::insert(const TMAP::dynamicObstacleMAP::observation *cur, const TMAP::dynamicObstacleMAP::observation *prev){
  numObstacles++;
  data = (TMAP::dynamicObstacleMAP::dynamicObstacle *)realloc(data, numObstacles*sizeof(TMAP::dynamicObstacleMAP::dynamicObstacle));
  memcpy(&data[numObstacles-1].cur, cur, 
	 sizeof(TMAP::dynamicObstacleMAP::observation));
  memcpy(&data[numObstacles-1].prev, prev, 
	 sizeof(TMAP::dynamicObstacleMAP::observation));
}


/* clears all data and makes a copy of the map */
void TMAP::dynamicObstacleMAP::clone(const dynamicObstacleMAP &copy){
  clearObstaclesAndData();
  for(unsigned int i=0;i<copy.getNumObstacles();i++)
    insert(&copy.getObstacle(i).cur, &copy.getObstacle(i).prev);
}
