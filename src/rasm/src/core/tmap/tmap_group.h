/*
 * Subclass of tmap_base that finds clusters of points.
 * Uses the tmap as a simple graph to identify groups.
 * Relies on the tmap_associated subclass for efficient traversal.
 *
 * Dominic Jonak
 */

#ifndef TMAP_GROUP_H
#define TMAP_GROUP_H

#include <tmap_associated.h>

namespace TMAP {
  class tmap_group : public virtual tmap_associated{

  public:

    tmap_group():tmap_associated(){}
    tmap_group(const char *filename):tmap_associated(filename){}

    /* given an array of point indices
     * categorize them into groups and return the number of groups
     * the groups are 0 indexed;
     *   if all points are placed into the same group,
     *     then the groups array will be zero filled and 1 is returned
     *   if all points are placed into different gorups,
     *     then the groups array will be 0,1..(numInd-1) and numInd is returned
     */
    unsigned int markPointGroups(unsigned int numInd,
				 const unsigned int *ind,
				 unsigned int *groups)const;
  };/* end of tmap_group */

}/* end of namespace TMAP */

#endif /* end of TMAP_GROUP_H */
