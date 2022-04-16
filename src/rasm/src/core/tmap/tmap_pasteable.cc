#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <tmap_pasteable.h>

/* puts the new map over this one,
   remove any points in this model that are inside or
   within a threshold of the new one
   deletes data from the regions where the new map is
*/
void TMAP::tmap_pasteable::paste(const tmap_searchable &newmap, 
				 const float tooClose){
  errorCheck();
  newmap.errorCheck();

  unsigned int newcap = numPoints()+newmap.numPoints();
  unsigned int newsize = 0;
  RASM::point3d *merged = (RASM::point3d *)malloc(newcap*sizeof(RASM::point3d));

  printf("paste() The old model has %d vertices and %d faces\n", numPoints(), numTriangles());
  printf("paste() The new model has %d vertices and %d faces\n", newmap.numPoints(), newmap.numTriangles());

  for(unsigned int i=0;i<numPoints();i++){
    /* skip vertices that fall inside a triangle in the new model */
    //    if(-1 != newmap.findTriangle2d(vertices[i], 0))continue;

    /*!
     * 12/2/2020:
     * When we originally began incorporating RASM into the MoonRanger autonomy,
     * the following check was performed with 2D points; changing the check to
     * 3D points seemed to improve mesh-generation when ICP was incorporated.
     */
    /* skip vertices that fall really close to an existing vertex in the new model */
    //int closestVertex = newmap.findClosestVertex(RASM::point2d(vertices[i]));
    int closestVertex = newmap.findClosestVertex(RASM::point3d(vertices[i]));
    assert(closestVertex>=0);
    //if(dist2d(vertices[i], newmap.getPointRef(closestVertex)) < tooClose)continue;
    if(dist3d(vertices[i], newmap.getPointRef(closestVertex)) < tooClose)continue;

    /* add any points that pass that gauntlet */
    merged[newsize++] = vertices[i];
  }

  printf("paste() Finished scanning through old model, keeping %d points\n", newsize);

  for(unsigned int i=0;i<newmap.numPoints();i++)
    merged[newsize++] = newmap.getPointRef(i);

  printf("paste() Added all the corners from the new model, now have %d points\n", newsize);
  assert(newsize>0);

  if(vertices){free(vertices);vertices=NULL;}
  vertices = merged;
  //  sizeVertices = newsize;
  //  capacityVertices = newcap;
  resizeVertices(newsize, newcap);

  if(faces){free(faces);faces=NULL;}
  //  sizeFaces = capacityFaces = 0;
  resizeTriangles(0, 0);

  triangulate();
  errorCheck();
}
