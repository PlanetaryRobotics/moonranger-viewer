/*
 * Subclass of tmap_base to create the shape of an arc and evaluate it
 *
 * Dominic Jonak
 */
/*!
 * 12/07/2020:
 * As far as I've been able to tell, MoonRanger's navigation doesn't
 * explicitly use the tmap_arc class; it only uses an arc/kinematic
 * model interface for path evaluation.
 *
 * Note, too, that multiArc is not used.
 */
#ifndef TMAP_ARC_H
#define TMAP_ARC_H

#include <rasm_common_types.h>
#include <tmap_moveable.h>
#include <tmap_astar.h>
#include <obstacleMap.h>
#include <path_model.h>

namespace TMAP {
  const unsigned int pointsPerPath = 2; /* ie, tracks */

  extern unsigned int numAxleChecks; /* check this many points between the wheels for clearance */
  extern float wheelDist; /* half the body width, ie distance from wheel to body pivot */
  extern float axleHeight; /* roughly the wheel radius */
  extern float roverLength; /* distance from front wheels to back wheels */

  /* don't allow roll/pitch over this amount (in radians) */
  extern float dangerousRoll;
  extern float dangerousPitch;

  /* roll/pitch under this amount are treated as flat (in radians) */
  extern float safeRoll;
  extern float safePitch;

  /* arc evaluation = arc cost * arcToPathFactor + path cost */
  extern float arcToPathFactor;

  /* whether or not intersection tests should be verbose (for debugging) */
  extern bool verboseIntersection;

  class tmap_path : public virtual tmap_moveable{
    unsigned int numStepsToSkip;
    bool forwardPath;

    /* given 2 points vertices[a] and vertices[b] representing the front wheels
     * find the cost of the vehicle sitting there
     * the _tri parameters are triangle indices near those points
     * returns <0 if it is an obstacle
     * otherwise returns a value between FREE and OBST
     */
    float evaluate(const tmap_astar &model, 
		   unsigned int a, int &a_tri,
		   unsigned int b, int &b_tri,
		   unsigned int tailA, int &tailA_tri,
		   unsigned int tailB, int &tailB_tri,
		   obstacleMAP &obstacles, 
		   bool extraWide, 
		   FILE *debugOutput);

    RASM_UNITS getMinClearance(const tmap_astar &model, 
			       const RASM::point3d &A,
			       const RASM::point3d &B,
			       unsigned int triangleIndex,
			       bool extraWide)const;

  public:

    bool inline isForward()const{return forwardPath;}
    tmap_path();

    void createPath(PathModel *path_model, 
		    unsigned int path_index, 
		    float arcLength, 
		    float arcResolution,
		    bool forward, 
		    float max_heading_change, 
		    float headingResolution);

    /* fills in costOfArc and costAfterArc
     * it first computes costOfArc by adjusting the arc to conform to the
     * world model and evaluting the result
     * if obstacles are detected, they are inserted into the obstacle map
     *
     * if arcOnly is set, then costAfterArc is not computed
     * if an obstacle was detected, -1 is returned, otherwise returns 0
     *
     * otherwise it runs astar (on the world model and respecting the obstacles)
     * to compute costAfterArc
     * then returns a combined score which is -1 for obstacles/unreachable goal
     * and >0 for valid arcs
     */
    float evaluate(tmap_astar &model, obstacleMAP &obstacles,
		   float &costOfPath, float &costAfterPath, 
		   bool pathOnly, bool extraWide,
		   FILE *debugOutput=NULL);

    /* returns the near evaluation of the arc or -1 for obstacles */
    float evaluateNear(tmap_astar &model, obstacleMAP &obstacles, 
		       bool extraWide, FILE *savedEvaluation=NULL);

    /* returns the far evaluation of the arc or -1 for unreachable goal */
    float evaluateFar(tmap_astar &model, const obstacleMAP &obstacles, 
		      bool extraWide, FILE *savedEvaluation=NULL)const;

    /* fill in the vehicle position and yaw at the end of the arc */
    void getFinalPose(RASM::point3d &p, float &yaw) const;
    
    /* returns true if this arc intersects a circle or ray */
    bool intersectCircle(const RASM::point2d &p, double radius)const;
    bool intersectRay(const RASM::point2d &p0, double r0,
		      const RASM::point2d &p1, double r1)const;

  }; /* end of class tmap_arc */

} /* end of namespace TMAP */

#endif /* end of TMAP_ARC_H */
