/*
 * Subclass of tmap_base that adds centers and associations.
 * This is used to efficiently find triangle neighbors
 *
 * Dominic Jonak
 */
#ifndef TMAP_ASSOCIATED_H
#define TMAP_ASSOCIATED_H

#include <assert.h>
#include <tmap_base.h>

namespace TMAP {

  class tmap_associated : public virtual tmap_base {
  protected:

    /* centers of triangles */
    RASM::point3d *centers;

    /* for each vertex, a list of triangle indices associated with that vertex
     * (the first element is a count)
     *
     * there should be at least one association,
     * so triangleAssociations[i][0] will be > 0
     * and triangleAssociations[i][1...n] will be triangle indices
     */
    unsigned int **triangleAssociations;
    unsigned int allocatedAssociationSize;

    /* finds one of the two triangles that uses p1 and p2
     * ignoreMe is the one that you don't want
     * returns -1 if there is no second triangle
     *
     * relies on triangleAssociations, so call fillInAssociations() first
     */
    int findTriangleWithVertices(unsigned int p1, unsigned int p2,
				 unsigned int ignoreMe) const;

  public:
    tmap_associated();

    /* an smf file */
    tmap_associated(const char *filename);

    virtual ~tmap_associated();

    void print() const;
    void printAssociations() const;
    void printStats() const;

    void clearPointsAndTriangles();

    /* creates new vertices for the triangle centers */
    void fillInCenters();
    void removeCenters();

    /* fills in the association lookup table
     * this must be done AFTER fillInCenters()
     */
    void fillInAssociations();
    void removeAssociations();

    /* fills in the neighbor portion of each triangle,
     * this must be done AFTER fillInAssociations
     */
    void fillInNeighbors();

    inline const RASM::point3d &getCenter(unsigned int ind) const{return centers[ind];}

    virtual void errorCheck() const;

    void errorCheckAssociationSize()const{assert((numPoints()==allocatedAssociationSize) || (!triangleAssociations && (0==allocatedAssociationSize)));}

  }; /* end of class tmap_associated */
} /* end of namespace TMAP */

#endif /* end of TMAP_ASSOCIATED_H */
