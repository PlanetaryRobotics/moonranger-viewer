#include "plotClouds_triangulate.h"
#include "triangle.h"


triangles *createTriangles(const points *p){
  triangles *ret = new triangles();
  struct triangulateio in, out;
  SECONDS start;

  in.numberofpoints = p->size();
  in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
  for(int i=0;i<in.numberofpoints;i++){
    in.pointlist[i*2 + 0] = p->get(i).x;
    in.pointlist[i*2 + 1] = p->get(i).y;
  }

  in.numberofpointattributes = 0;
  in.pointattributelist = NULL;
  in.pointmarkerlist = NULL;
  in.numberofsegments = 0;
  in.numberofholes = 0;
  in.numberofregions = 0;
  in.regionlist = NULL;

  //printf("Input point set:\n\n");
  //report(&in, 1, 0, 0, 0, 0, 0);

  out.pointlist = (REAL *) NULL;            /* Not needed if -N switch used. */
  /* Not needed if -N switch used or number of point attributes is zero: */
  out.pointattributelist = (REAL *) NULL;
  out.pointmarkerlist = (int *) NULL; /* Not needed if -N or -B switch used. */
  out.trianglelist = (int *) NULL;          /* Not needed if -E switch used. */
  /* Not needed if -E switch used or number of triangle attributes is zero: */
  out.triangleattributelist = (REAL *) NULL;
  //out.neighborlist = (int *) NULL;         /* Needed only if -n switch used. */
  /* Needed only if segments are output (-p or -c) and -P not used: */
  //out.segmentlist = (int *) NULL;
  /* Needed only if segments are output (-p or -c) and -P and -B not used: */
  //out.segmentmarkerlist = (int *) NULL;
  //out.edgelist = (int *) NULL;             /* Needed only if -e switch used. */
  //out.edgemarkerlist = (int *) NULL;   /* Needed if -e used and -B not used. */

  printf("Calling triangulate on %d points\n", in.numberofpoints);
  start = getCurTime();
  triangulate("zQX", &in, &out, (struct triangulateio *) NULL);
  printf("Time: %g sec\n", ((getCurTime())-start));

  assert(3 == out.numberofcorners);
  for(int i=0;i<out.numberoftriangles;i++){
    int a = out.trianglelist[i * 3 + 0];
    int b = out.trianglelist[i * 3 + 1];
    int c = out.trianglelist[i * 3 + 2];
    if(p->get(a).normSq()>0 && p->get(b).normSq()>0 && p->get(c).normSq()>0){
      ret->add(triangle(p->get(a), p->get(b), p->get(c)));
      ret->getRef( ret->size()-1 ).setInd(a, b, c);
    }
  }

  printf("Number of triangles: %d\n", out.numberoftriangles);

  /* clean up all the triangulation stuff */
  free(in.pointlist);
  free(in.pointattributelist);
  free(in.pointmarkerlist);
  free(in.regionlist);
  free(out.pointlist);
  free(out.pointattributelist);
  free(out.pointmarkerlist);
  free(out.trianglelist);
  free(out.triangleattributelist);
//  free(out.neighborlist);
//  free(out.segmentlist);
//  free(out.segmentmarkerlist);
//  free(out.edgelist);
//  free(out.edgemarkerlist);

  printf("%d points -> %d triangles\n", p->size(), ret->size());

  return ret;
}
