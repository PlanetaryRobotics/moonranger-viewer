/*
 * Subclass of tmap_base to find closest points
 *
 * Dominic Jonak
 */
#ifndef SEARCHABLE_TMAP_H
#define SEARCHABLE_TMAP_H

#include <tmap_associated.h>
#include <tmapQuadTree.h>
#include <tmapOctTree.h>

namespace TMAP {

  class tmap_searchable : public virtual tmap_associated{
  protected:
    quadTree *treeOfVertices, *treeOfCenters;
    octTree *octOfVertices, *octOfCenters;

    unsigned int findClosestVertex2d(const RASM::point2d &target)const;
    unsigned int findClosestCenter2d(const RASM::point2d &target)const;
    unsigned int findClosestVertex3d(const RASM::point3d &target)const;
    unsigned int findClosestCenter3d(const RASM::point3d &target)const;

  public:


    tmap_searchable()
      :tmap_associated(),
      treeOfVertices(NULL), treeOfCenters(NULL),
      octOfVertices(NULL), octOfCenters(NULL){}
    tmap_searchable(const char *filename)
      :tmap_associated(filename),
      treeOfVertices(NULL), treeOfCenters(NULL),
      octOfVertices(NULL), octOfCenters(NULL){}

    virtual ~tmap_searchable();

    inline bool haveTreeOfVertices()const {return (treeOfVertices!=NULL);}
    inline bool haveTreeOfCenters()const {return (treeOfCenters!=NULL);}

    /* returns the index of the closest point */
    unsigned int findClosestVertex(const RASM::point2d &target)const;
    unsigned int findClosestVertex(const RASM::point3d &target)const;
    unsigned int findClosestCenter(const RASM::point2d &target)const;
    unsigned int findClosestCenter(const RASM::point3d &target)const;

    /* finds a point on the surface, either by projecting flatly (fast)
     * or searching in 3D (slow)
     * returns -1 if the target is not on the mesh
     */
    int findSurfaceAt(const RASM::point2d &target, RASM::point3d &result)const;
    RASM::point3d findClosestSurface(const RASM::point3d &target)const;

    /* like above, except specify the index of a nearby triangle to start at
     * set it to -1 to not start anywhere
     * triangleIndex is filled in with the containing triangle
     */
    int findSurfaceAt(const RASM::point2d &target, RASM::point3d &result,
		      int &traingleIndex)const;

    /* returns the index of the triangle that contains this point
     * returns -1 if no triangle was found
     */
    int findTriangle2d(const RASM::point2d &target, int verbose=0)const;

    /* like above, except that you supply the index of a triangle
     * this triangle should be close and may contain the target
     */
    int centerSearch(const RASM::point2d &target, unsigned int index, int verbose=0) const;

    /* builds a tree */
    void buildTreeVertices();
    void buildTreeCenters();

    /* inserts the last n vertices into the quad tree */
    void updateTreeVertices(unsigned int n);

    /* returns the index of the triangle that contains the ray 
     * through this point in this (unit) direction 
     * returns -1 if no triangle was found
     */
    int findTriangle3d(const RASM::point3d &point, 
		       const RASM::point3d &direction)const{return -1;}
  }; /* end of class tmap_searchable */
} /* end of namespace TMAP */

#endif /* end of SEARCHABLE_TMAP_H */
