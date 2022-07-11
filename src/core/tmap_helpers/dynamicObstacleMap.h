#ifndef DYNAMIC_OBSTACLE_MAP_H
#define DYNAMIC_OBSTACLE_MAP_H

#include <rasm_common_types.h>
#include <obstacleMap.h>
#include <multiArc.h>

namespace TMAP {

  class dynamicObstacleMAP{
  public:
    struct observation{
      RASM::point3d p;
      RASM_UNITS r;
      unsigned int N;
      double t;
    };

    struct dynamicObstacle{
      struct observation cur, prev;
    };

  private:
    struct dynamicObstacle *data;
    unsigned int numObstacles;

    struct observation *newData;
    unsigned int numObservations;
    double curTime;

    /* add a new observation, does a deep copy of the observation/obstacle */
    void appendNewObstacle(const struct observation *o);

    /* scans for unmatched observations and matches them with obstacles
     * the obstacles can be either unmatched or matched
     * returns true if any change was made
     */
    bool greedyMatch(const float *matchScore,
		     const int *matchRank,
		     int *obstacleNumber,
		     int *observationNumber);

    /* This function attempts to match obstacles to observations
     * there must be atleast one of each
     */
    void matchObservations(const float *matchScore,
			   const int *matchRank,
			   int *obstacleNumber);

  public:
    dynamicObstacleMAP();
    ~dynamicObstacleMAP();
    
    /* following observations will have this time */
    void startObservations(double t);
    /* observed obstacle at this point, with this size and this many points */
    void addObservation(const RASM::point3d &a, const RASM_UNITS dist,
			unsigned int N);
    /* finishes adding observations and attempts to match up with previous */
    void endObservations();

    /* checks if the line between these points is free of moving obstacles
     * for some time interval (in seconds)
     */
    bool crossesObstacle(const RASM::point3d &a, const RASM::point3d &b,
			 double t0, double t1) const;

    /* converts all the dynamic obstacles into static ones
     * predict motion to time t
     */
    void exportStatic(obstacleMAP &staticObstacles, double t)const;

    /* checks if this arc intersects any obstacle to time t
     * returns true if there is an intersection
     */
    bool checkArc(const multiPath &arc, double t)const;

    void writeToFile(const char *filename)const;
    void readFromFile(const char *filename);
    void prune(double t);

    /* removes all obstacles and data
     */
    void clearObstaclesAndData();

    /* forces a new obstacle defined by these two observations
     */
    void insert(const observation *cur, const observation *prev);

    /* returns the count of dynamic obstacles
     */
    unsigned int getNumObstacles()const{return numObstacles;}

    /* accessor to peek at the ith obstacle
     */
    const dynamicObstacle &getObstacle(unsigned int ind)const{return data[ind];}

    /* clears all data and makes a copy of the map */
    void clone(const dynamicObstacleMAP &copy);

  }; /* end of class dynamicObstacleMap declaration */

} /* end of namespace TMAP */

#endif
