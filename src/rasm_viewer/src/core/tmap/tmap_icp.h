/*
 * Subclass of tmap_base to apply ICP between two tmaps
 *
 * Dominic Jonak
 */

/*!
 * 12/4/2020 note:
 *
 * If unfamiliar with iterative closest point (ICP), it is worth reading about it in order to better understand the proceeding functionality:
 *
 * https://en.wikipedia.org/wiki/Iterative_closest_point
 */

#ifndef TMAP_ICP_H
#define TMAP_ICP_H

#include <tmap_moveable.h>
#include <tmap_searchable.h>

/* allow ICP to rotate? */
#define ICP_ROTATION 1

/* allow ICP to translate? */
#define ICP_TRANSLATION 1

/* how to pick the closest point, should it be a point or from the surface? */
#define ICP_USE_SURFACE 0

/* if ICP_USE_SURFACE, should it be quick and dirty, or optimal? */
#define ICP_USE_FAST_SURFACE 0

namespace TMAP {

  class tmap_icp :
  public virtual tmap_moveable,
    public virtual tmap_associated {
  public:
    static int getTransformation(unsigned int numPoints,
				 const RASM::point3d *local,
				 const RASM::point3d *world,
				 float transform[4][4],
				 bool shouldFindMean,
				 bool shouldRotate,
				 bool shouldTranslate,
				 bool shouldTranslateXY);

  private:

    /* given a world model, place each point in this model into one of the arrays
     * exteriorPoints - these are outside the other model (or far from targetPoint)
     * strayPoints    - these are far from the other model
     * interiorPoints - these are properly associated with the other model
     *                  (their associated points are stored in closestPoints)
     */
    void associatePoints(const tmap_searchable &world,
			 const RASM::point3d &targetPoint, double maxDistSq,
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
			 
			 /* distance threshold for stray points */
			 double distThreshSq, double verticalDist 
			 ) const;

    /* like above, but optimized:
     * starting with an arbitrary vertex, grows outward (breadth first)
     * and uses centerSearch() to improve from the previous match
     * this avoids a relatively costly tree lookup
     *
     * note that the results should be identical only when both
     * useSurface=1 and vertex3d=0,
     * otherwise the results may be slightly different as the associated point
     * need not be in the containing triangle
     *
     * the performance improvement is modest on small models
     * but very noticeable on large models
     *
     * Here's some timing numbers I got:
     *  # triangles   above    batch
     *     1,000    0.00250s 0.00212s
     *    70,000    0.380s   0.122s
     * (70,000 triangle models took about 1/3s each, down to about 1/10s)
     */
    void associatePointsBatch(const tmap_searchable &world,
			      const RASM::point3d &targetPoint, double maxDistSq,
			      unsigned int *interiorPoints,
			      RASM::point3d *closestPoints, /* points in world, may not be vertices */
			      unsigned int *strayPoints,
			      unsigned int *exteriorPoints,
			      unsigned int &numInteriorPoints,
			      unsigned int &numStrayPoints,
			      unsigned int &numExteriorPoints,
			      bool shouldFindMean,
			      bool useSurface, 
			      bool vertex3d, 
			      double distThreshSq, double verticalDist 
			      ) const;

    /* does one step of icp 
       targetPoint and maxDistSq 
       - if maxDistSq is positive, then only consider points within the
         square distance of the given point
       shouldFindMean
       - consider rotations about the mean point rather than the origin
       - unclear if this changes the results significantly
       shouldRotate
       - compute the rotation component
       shouldTranslate
       - compute the translation component
       shouldTranslateXY
       - if translating, are XY motions allowed or only altitude?
       useSurface
       - match points against the surface of the world
       - otherwise match against vertices
       vertex3d
       - if matching against vertices in the world, 
         should the nearest neighbor be found in XY only or in 3d
       transform
       - this matrix is filled in with the transformation to apply to
         this data to make it match the world

       return codes:
	 0 - ok
	 1 - not enough points
	 2 - excessive rotation
	 3 - flipped Z
	 4 - excessive motion
    */
    int icpOneStep(const tmap_searchable &world,
		   const RASM::point3d &targetPoint, double maxDistSq,
		   bool shouldFindMean, bool shouldRotate, 
		   bool shouldTranslate, bool shouldTranslateXY,
		   bool useSurface, bool vertex3d,
		   float transform[4][4]) const;

  public:

    tmap_icp():tmap_moveable(),tmap_associated(){}
    tmap_icp(const char *filename):tmap_moveable(filename),tmap_associated(){}

    /* for an arbitrary pair of local and world maps,
     * shouldFindMean should be set to 1, 
     * this will rotate the local map about it's center of mass
     *
     * if BOTH maps are aligned to the origin, you can set it to 0,
     * this will rotate the local map about the origin
     * regardless of where the center of mass is (or moves to)
     *
     * 0 - ok
     * 1 - not enough points
     * 2 - excessive rotation
     * 3 - excessive motion
     */
    int icp(const tmap_searchable &world, unsigned int maxIterations,
	    const RASM::point3d &targetPoint, double maxDist,
	    bool shouldFindMean=true,
	    bool shouldRotate=ICP_ROTATION,
	    bool shouldTranslate=ICP_TRANSLATION,
	    bool shouldTranslateXY=true,
	    const char *fileBase=NULL,
	    bool useSurface=ICP_USE_SURFACE);

    /* like above except
       does not change the model
       //does not translate
       uses one iteration
    */
    int getTransform(const tmap_searchable &world,
		     float transform[4][4],
		     bool shouldFindMean=true) const;

    void getInteriorAndStrayPoints(const tmap_searchable &world,
				   unsigned int *interiorPoints,
				   unsigned int *strayPoints,
				   unsigned int &numInteriorPoints,
				   unsigned int &numStrayPoints,

				   /* distance threshold for stray points */
				   double nearDist, double verticalDist,

				   bool shouldFindMean=true)const;
  }; /* end of class tmap_icp */

} /* end of namespace TMAP */

#endif /* end of TMAP_ICP_H */
