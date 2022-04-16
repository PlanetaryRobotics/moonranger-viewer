/*
 * Subclass of tmap_base to compute cost/distance between points
 *
 * Dominic Jonak
 */
#ifndef ASTAR_TMAP_H
#define ASTAR_TMAP_H

#include <stdio.h>
#include <tmap_searchable.h>
#include <obstacleMap.h>

namespace TMAP {

#define OBST	255	/* obstacle cost */
#define FREE	5	/* free space cost */

  extern bool NO_PATH_SMOOTHING;
  extern bool USE_HEURISTIC;

  /* triangle cost in astar search, based on z component of unit normal */
  extern float astarZmin;
  extern float astarZmax;

  /* altitude cost based on vertical distance from a target */
  extern float minAltitude;
  extern float maxAltitude;

  /* look at the total altitude change of this triangle
   * if it's a tiny change (ie, a tiny triangle)
   * then ignore the slope component or atleast scale it down
   */
  extern float minPeakTrough;
  extern float maxPeakTrough;

  /* this class defines the interface to a heap class used during the search */
  class openListInterface{
  public:
    virtual ~openListInterface(){}
    virtual void print() const{}
    virtual void insert(unsigned int ind, double cost, int neighbor, 
			float heuristicCost)=0;
    virtual void removeFirst(int &ind, float &cost, int &neighbor)=0;
    virtual float minCost() const=0;
    virtual unsigned int getsize() const =0;
    virtual void reset(){}
    virtual int getTriangle(unsigned int i)const =0;
    virtual void setHeuristic(unsigned int i, float h)=0;
    virtual void fixHeap()=0;
  };

  /* this class stores book keeping information used during the search */
  struct astarNode{
    int triangleIndex; /* index of the triangle that defines this node */
    int neighborToGoal; /* index of neighbor which is closest to goal
			   -1 means unknown */

    float cellcost; /* cost to traverse 1 meter in this triangle,
		       <0 means unknown */

    /* total cost to the goal,
     * 0 means this is a goal,
     * <0 means unknown
     */
    float pathcost;

    /* minimum cost to the target triangle
     * <0 means unknown
     */
    float heuristicCost;

    /* when entering from triangle i and triangle i is not the goal;
     * i != neighborToGoal
     * i == neighborFromGoal[0] or i == neighborFromGoal[1]
     * then don't go to the center of this traingle, 
     * instead go to the corresponding transitionPoint
     * also the total cost of the path from that transitionPoint to the goal
     * is  transitionPathCost[i]
     *
     * if triangle i is the goal (ie, -1==neighborToGoal)
     * then neighborFromGoal, transitionPoint and transitionPathCost
     * are all invalid
     */
    int neighborFromGoal[2];
    RASM::point3d transitionPoint[2];
    float transitionPathCost[2];

    astarNode():triangleIndex(-1), neighborToGoal(-1), cellcost(-1), pathcost(-1){}
    astarNode(int ind, float cost):triangleIndex(ind), neighborToGoal(-1), cellcost(-1), pathcost(cost){}
  };/* end of class astarNode */

  class tmap_astar: public virtual tmap_searchable{
  private:
    RASM::point2d goalPoint;
    int goalIndex;
    openListInterface *openList;
    astarNode *astarNodes;

    /* if astar runs out of triangles, that means the map is disconnected
     * call this function to jump to another portion of the map
     * returns -1 if all triangles have been evaluated already
     * otherwise returns the index of the closested one
     */
    int findRemainingTriangleClosestToGoal();

    /* initializes state for goalx,goaly which is off the map */
    void addExteriorGoal();

    /* minimum cost from triangle index, 
     * to either the goal or a specific triangle
     */
    float minCostToGoal(int ind);
    float minCostToTriangle(int ind, int targetTriangle);

    float estimatedCostThroughNeighbor(unsigned int triangleIndex, 
				       unsigned int neighbor);

    void fillInTransitionPoints(unsigned int triangleIndex);

    /* these are just for the brute-force edge transition search */
    void transferToEdges()const;
    void writeEdgeEvaluation(const RASM::point3d &p, int triangle,
			     const char *file)const;
    void writeEdgeEvaluation(const char *file)const;

  public:

    float getCost(int triangleIndex)const;

    /* searches for the pathcost of triangleIndex
     * returns 1 if it was found, 0 otherwise (eg a timeout)
     * maxSteps limits how many nodes are expanded
     * set maxSteps to -1 for keep expanding until we're out of nodes
     * set triangleIndex to -1 to keep expanding
     */
    int astar(int triangleIndex, int maxSteps, const obstacleMAP &obstacles);

    /* just calls astar with no limits, 
     * same as calling astar on each node
     */
    void evaluatePaths();

    void astarReset();

    tmap_astar();
    ~tmap_astar();

    float costToTriangleCenter(const RASM::point2d &p, unsigned int tri) const;
    int getContainingTriangle(const RASM::point2d &p, float &costToCenter) const;

    void writeEvaluationToFile(const char *filename);
    void writeEvaluationToFile(unsigned int triangleIndex,
			       const RASM::point2d &start,
			       FILE *f);
    void astarPrint() const;
    void setGoal(const RASM::point2d &goal);
    float readpathcost(int triangleIndex, const obstacleMAP &obstacles);
    float inline readcellcost(int triangleIndex)const
      {return astarNodes[triangleIndex].cellcost;}
    int inline readnextcell(int triangleIndex) const{
      return astarNodes[triangleIndex].neighborToGoal;
    }
  }; /* end of class tmap_astar */

} /* end of namespace TMAP */

#endif /* end of ASTAR_TMAP_H */
