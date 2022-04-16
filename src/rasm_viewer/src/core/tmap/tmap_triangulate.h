/*
 * Subclass of tmap_base to triangulate a point cloud
 *
 * Dominic Jonak
 */
#ifndef TMAP_TRIANGULATE_H
#define TMAP_TRIANGULATE_H

#ifndef VERBOSE
#define VERBOSE 0
#endif

#include <tmap_associated.h>

namespace TMAP {

  class tmap_triangulate : public virtual tmap_base {

  public:

    tmap_triangulate():tmap_base(){}
    tmap_triangulate(const char *filename):tmap_base(filename){}
    
    /* triangulates the vertices */
    void triangulateMinimal();

    /* triangulates the vertices and fills in the faces */
    void triangulate();
  };

}

#endif
