/*
 * Subclass of tmap_base to combine two tmaps
 *
 * Dominic Jonak
 */
#ifndef TMAP_PASTEABLE_H
#define TMAP_PASTEABLE_H

#include <tmap_searchable.h>
#include <tmap_triangulate.h>

namespace TMAP {

  class tmap_pasteable : public virtual tmap_triangulate{

  public:

    tmap_pasteable():tmap_triangulate(){}
    tmap_pasteable(const char *filename):tmap_triangulate(filename){}

    /* puts the new map over this one,
     * remove any points in this model that are inside or
     * within a threshold of the new one
     * deletes data from the regions where the new map is
     */
    void paste(const tmap_searchable &newmap,
	       const float tooClose = 0.1);

  }; /* end of class tmap_pasteable */

} /* end of namespace TMAP */

#endif
