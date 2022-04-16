#ifndef TMAP_CLEANABLE_H
#define TMAP_CLEANABLE_H

#include <stdlib.h>
#include <tmap_associated.h>

namespace TMAP {

  class tmap_cleanable : public virtual tmap_associated{
  private:
    /* face mask, mark faces for removal here */
    bool *fMask;

    /* cached mean and std edge lengths (<0 indicates not computed) */
    double edgeMean, edgeStd;

    double computeMeanEdgeLength()const;
    double computeEdgeLengthStd()const;

  public:
    /* statistics about the edge lengths */
    void invalidateEdgeLengths(){edgeMean = edgeStd = -1.0;}

    double getMeanEdgeLength(){
      if(edgeMean<0.0)edgeMean = computeMeanEdgeLength();
      return edgeMean;
    }

    double getEdgeLengthStd(){
      if(edgeMean<0.0){
	edgeMean = computeMeanEdgeLength();
	edgeStd = computeEdgeLengthStd();
      }else if(edgeStd<0.0){
	edgeStd = computeEdgeLengthStd();
      }
      return edgeStd;
    }

  public:

    tmap_cleanable()
      :tmap_associated(),fMask(NULL),edgeMean(-1.0),edgeStd(-1.0){}
    tmap_cleanable(const char *filename)
      :tmap_associated(filename),fMask(NULL),edgeMean(-1.0),edgeStd(-1.0){}
    virtual ~tmap_cleanable(){
      if(fMask){free(fMask);fMask=NULL;}
    }

    /* the deviation filters use the standard deviation of edge lengths
     * 'long' selects edges more than threshold times the deviation
     * 'points' does a single step expansion to grow the area
     *          ie. mark both endpoints instead of a single edge
     */
    enum cleanableTMAPfilters { noop=0,
				deviationsLong, deviationsLongPoints,
				deviationsShort, deviationsShortPoints,
				missingNeighbor,
				minAngle,
				unusedPoints,
				borderVertices
    };

    /* mask data according this filter
     *
     * returns the number of triangles that were masked
     */
    unsigned int maskBadData(cleanableTMAPfilters filter, double threshold);
    unsigned int maskBadData(cleanableTMAPfilters filter);

    /* keep data within distance of center
     *
     * returns the number of triangles that were masked
     */
    unsigned int maskFarData(RASM_UNITS distance, const RASM::point2d &center);

    /* start is the vehicle position, end is the goal location
     * data within minDist is kept, data beyond maxDist is removed
     * data between min and maxDist is checked to see if it is near the line
     * allowableAngle describes how close it needs to be to the line
     *
     * returns the number of triangles that were masked
     */
    unsigned int maskDataLine(const RASM::point2d &start, const RASM::point2d &end,
			      RASM_UNITS minDist, RASM_UNITS maxDist,
			      double allowableAngle);
    void removeBadData();
  }; /* end of class tmap_cleanable */

}/* end of namespace TMAP */

#endif /* end of TMAP_CLEANABLE_H */
