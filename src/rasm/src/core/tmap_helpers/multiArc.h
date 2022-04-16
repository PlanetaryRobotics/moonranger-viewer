/*
 * a multi arc is like an arc, but with branching ends
 * used to combine arcs into compound actions
 *
 * Dominic Jonak
 */

/*!
 * 12/07/2020:
 * This class is not presently used by MoonRanger's navigation.
 * Its functionality, however, may be of future interest given
 * the fact that MoonRanger is presently evaluating a single arc
 * at a time.
 */
#ifndef MULTI_ARC_H
#define MULTI_ARC_H

#include <rasm_common_types.h>
#include <path_model.h>
#include <stdlib.h>
#include <stdio.h>
#include <tmap_arc.h>
#include <tmap_base.h>

namespace TMAP {

  class multiPath{
  private:

    /* a bonus given to forward children of reverse paths
     * this modifies the effective length
     */
    static const float adjustment_stopBackingUp;

    /* a penalty given to curving branches
     * this modifies the effective length
     */
    static const float adjustment_curvedBranches;

    /* after applying adjustments,
     * ensure each path is this long
     */
    static const float minAdjustedPathLength;

    /* when adding a child path,
     * make it sparser by increasing the spacing/resolution this much
     */
    static const float adjustment_childResolutionFactor;

    /* collection of child paths */
    struct pathList{multiPath *child;struct pathList *next;};
    struct pathList *head;

    /* this arc segment and the generation parameters */
    tmap_path *initialPath;
    float rad, len, res;
    bool forw;

    /* evaluation information */
    float costOfPath, costAfterPath;/* 0 means unevaluated */
    multiPath *bestChild;

  public:

    multiPath():head(NULL), initialPath(NULL), bestChild(NULL){}
    ~multiPath();

    void createMultiPath( PathModel * path_model, 
			  unsigned int path_index, 
			  float arcLength, 
			  float arcResolution,
			  bool forward, 
			  float max_heading_change,
			  float headingResolution);

    /* adds a direct child multiPath */
    void addBranch(PathModel *path_model, 
		   unsigned int path_index, 
		   bool forward, 
		   float max_heading_change, 
		   float headingResolution);

    /* adds child multiArcs to any children that match the filter...
     * given radius R, example radiusLists:
     *         add R as a direct child
     * 5       pick the 5m child and add R as a direct child of it
     * 0       add R as a direct child of all children
     * 1000,5  pick the 1000m child, then its 5m child and insert there
     * 1000,0  pick the 1000m child, then each of its children and add it there
     * 0,1000  pick any child and go to its 1000m child and insert there
     */
    void addBranch(PathModel *path_model, 
		   unsigned int path_index,
		   bool forward,
		   const float *radiusList, 
		   unsigned int radiusListLen, 
		   float max_heading_change, 
		   float headingResolution);

    void evaluateNear(tmap_astar &model, obstacleMAP &obstacles, 
		      bool extraWide, FILE *savedEvaluation);
    void evaluateFar(tmap_astar &model, const obstacleMAP &obstacles, 
		     bool extraWide, FILE *savedEvaluation, bool savePaths);
    float getEvaluation(float &_costOfArc, float &_costAfterArc);

    void appendToTmap(tmap_base &tmap) const;
    void writeToFile(const char *name=NULL) const;
    void writeToFileTrack(const char *prefix=NULL) const;
    void writeToFile(FILE *f, unsigned int &numPriorPoints) const;
    void translate(const RASM::point3d &p);
    void translateAndRotate(const RASM::point3d &p,
			    double roll, double pitch, double yaw);

    /* returns 1 if this multi arc intersects a circle or ray
     * either parent or all children must intersect
     */
    bool intersectCircle(const RASM::point2d &p, double radius)const;
    bool intersectRay(const RASM::point2d &p0, double r0,
		      const RASM::point2d &p1, double r1)const;

    void markIntersection(){costOfPath = -1.0;}

    /* accessors (that apply to the parent arc, not any children) */
    bool isForward()const{return forw;}
    float getRadius()const{return rad;}

  };/* end of class multiArc declaration */

} /* end of namespace TMAP */

#endif
