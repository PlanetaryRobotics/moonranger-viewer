#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <tmap_triangulate.h>
#include <triangle.h>
#include <lineHelpers.h>


//#ifndef VERBOSE
//#define VERBOSE 0
//#endif

/* triangulates the vertices */
void TMAP::tmap_triangulate::triangulateMinimal(){

  /*
   * TEST: does this work? or should we do something else?
   */
  if(numPoints() < 3) return;

  struct triangulateio in, out;
  memset(&in,  0, sizeof(struct triangulateio));
  memset(&out, 0, sizeof(struct triangulateio));

  in.numberofpoints = numPoints();
  in.pointlist = (REAL *) alloca(in.numberofpoints * 2 * sizeof(REAL));
  for(int i=0;i<in.numberofpoints;i++){
    in.pointlist[i*2 + 0] = RASM_TO_METERS(vertices[i].X());
    in.pointlist[i*2 + 1] = RASM_TO_METERS(vertices[i].Y());
  }

  /*!
   * Here is where we have the option to modify the arguments to triangle.cc.
   * Note that 'X' is used to suppress Adaptive Exact Arithmetic, which was
   * what caused the deadlocking error on the TX2i. Suppressing it, though,
   * causes the planner to restart because of assertion errors. So, we need
   * it enabled so long as we use triangle.cc.
   */
  //printf("Triangulating %d of %d points\n", in.numberofpoints, numPoints());
  char args[4/*5*/] = {'z', 'Q', 'n', /*'X',*/ '\0'};
  ::triangulate(args, &in, &out, (struct triangulateio *) NULL);
  assert(3 == out.numberofcorners);

  //printf("Triangulated %d points, got %d faces\n", in.numberofpoints, out.numberoftriangles);
  //  capacityFaces = out.numberoftriangles;
  //  sizeFaces = 0;
  resizeTriangles(0, out.numberoftriangles);
  faces = (RASM::triangle *)realloc(faces, capacityTriangles() * sizeof(RASM::triangle));

  for(int i=0;i<out.numberoftriangles;i++){
    for(int j=0;j<3;j++)
      faces[i].points[j] = out.trianglelist[i * 3 + j];
  }

  for(int i=0;i<out.numberoftriangles;i++){
    bool isBad=false;
    for(int j=0;j<3;j++)
      if(faces[i].points[j]<0 || faces[i].points[j]>=numPoints())
	isBad=true;

    if(!isBad){
      if(faces[i].points[0] == faces[i].points[1] ||
	 faces[i].points[1] == faces[i].points[2] ||
	 faces[i].points[2] == faces[i].points[0]){

	printf("Error, duplicated points on triangle %d of %d\n", i, out.numberoftriangles);

	for(int j=0;j<3;j++){
	  int p = faces[i].points[j];
	  printf("  vertex %d ", p);
	  vertices[p].print();
	  printf("\n");
	}
	printf("Neighbors: %d %d %d\n", 
	       out.neighborlist[i*3+0],
	       out.neighborlist[i*3+1],
	       out.neighborlist[i*3+2]);

	assert(0);

	isBad = true;
      }
    }

    /* all the points are valid, do a colinear test */
    if(!isBad){
      isBad = isColinear(vertices, faces[i]);

      if(isBad){
#if VERBOSE
	printf("Colinear test failed in triangulation on face %d of %d\n", i, out.numberoftriangles);
	for(int j=0;j<3;j++){
	  unsigned int p = faces[i].points[j];
	  printf("  vertex %d ", p);
	  vertices[p].print();
	  printf("\n");
	}
	printf("Neighbors: ");
	for(int j=0;j<3;j++){
	  int n = out.neighborlist[i*3+j];
	  if(n<0)
	    printf(" %d", n);
	  else
	    printf(" %d %s", n, isColinear(vertices, faces[n])?"Y":"N");
	}
	printf("\n");
#endif
      }

      bool isRecoverable=false;
      if(isBad){
	/* check if this is recoverable:
	   we can recover if triangle i is on the edge of the map,
	   meaning at least 1 neighbor is missing

	   we can also recover if one of the neighbors is also colinear

	   in that case, update the existing neighbor(s) and remove this triangle
	 */
	for(int j=0;j<3;j++){
	  int n = out.neighborlist[i*3+j];
	  if(n < 0){
	    isRecoverable = true;
#if VERBOSE
	    printf("Recoverable because triangle %d has no neighbor in slot %d\n", i, j);
#endif
	  }
	}
#if VERBOSE
	if(!isRecoverable)
	  printf("Not recoverable because triangle %d has 3 neighbors\n", i);
#endif
	if(!isRecoverable){
	  for(int j=0;j<3;j++){
	    int n = out.neighborlist[i*3+j];
	    if(isColinear(vertices, faces[n])){
	      isRecoverable = true;
#if VERBOSE
	      printf("Recoverable because triangle %d has neighbor %d which is colinear\n", i, n);
#endif
	    }
	  }

	  if(!isRecoverable)
	    printf("Not recoverable because none of triangle %d neighbors are colinear\n", i);
	}
	if(isRecoverable){
#if VERBOSE
	  printf("Starting to recover\n");
#endif
	  /* update each neighbor */
	  for(int j=0;j<3;j++){
	    int k = out.neighborlist[i*3+j];
	    if(k<0)continue;
	    /* neighbor j is triangle k, 
	     * update k to remove all traces of colinear triangle i
	     */
	    for(int l=0;l<3;l++){
	      if(i == out.neighborlist[k*3+l]){
		faces[k].neighbors[l] = out.neighborlist[k*3+l] = -2;
		//printf("Updated triangle %d, neighbor %d was %d, now set to -2\n", k, l, i);
	      }
	    }
	  }
	}

	/* make sure there are no more references to triangle i */
	if(isRecoverable){
#if VERBOSE
	  printf("Checking recovery\n");
#endif

	  for(int j=0;j<out.numberoftriangles;j++){
	    for(int k=0;k<3;k++){
	      if(i == out.neighborlist[j * 3 + k]){
		isRecoverable = false;
		printf("Error, Triangle %d has this as neighbor %d: not recoverable!\n", j, k);
		printf("Note that we only expected neighbors %d %d %d\n",
		       out.neighborlist[i*3+0], out.neighborlist[i*3+1], out.neighborlist[i*3+2]);
	      }
	    }
	  }
	  if(!isRecoverable)
	    printf("Recovery failed\n");
	}
#if VERBOSE
	if(isRecoverable)
	  printf("Successfully recovered triangulation by removing colinear triangle %d!\n", i);
#endif
      }

      if(isBad && isRecoverable){
#if VERBOSE
	printf("This was a bad triangle, but we recovered\n");
#endif
      }else if(isBad && !isRecoverable){

	for(int j=0;j<out.numberoftriangles;j++){
	  for(int k=0;k<3;k++){
	    if(i == out.neighborlist[j * 3 + k]){
	      printf("Triangle %d has this as neighbor %d\n", j, k);
	      for(int l=0;l<3;l++){
		int p = faces[j].points[l];
		printf("  vertex %d ", p);vertices[p].print();printf("\n");
	      }
	    }
	  }
	}

	for(int j=0;j<3;j++){
	  unsigned int p = faces[i].points[j];
	  printf("Triangles that touch point %d ", p);vertices[p].print();printf("\n");
	  for(int k=0;k<out.numberoftriangles;k++){
	    if(p == faces[k].points[0] || p == faces[k].points[1] || p == faces[k].points[2]){
	      printf("Triangle %d:\n", k);
	      for(int l=0;l<3;l++){
		int p = faces[k].points[l];
		printf("  vertex %d ", p);vertices[p].print();printf("\n");
	      }
	      printf("  neighbors:");
	      for(int l=0;l<3;l++)
		printf(" %d", out.neighborlist[k * 3 + l]);
	      printf("\n");
	    }
	  }
	}


	assert(0);
      }
    }

    if(isBad){
#if VERBOSE
      printf("Skipping bad triangle (%d of %d)\n", i, out.numberoftriangles);
      printf("With neighbors: %d %d %d\n",
	     out.neighborlist[i*3+0], out.neighborlist[i*3+1], out.neighborlist[i*3+2]);
#endif
      out.numberoftriangles--;
      for(int j=0;j<3;j++)
	faces[i].points[j] = out.trianglelist[i * 3 + j] = out.trianglelist[out.numberoftriangles * 3 + j];
      for(int j=0;j<3;j++)
	faces[i].neighbors[j] = out.neighborlist[i * 3 + j] = out.neighborlist[out.numberoftriangles * 3 + j];

#if VERBOSE
      printf("Moved triangle %d with points %d %d %d and neighbors %d %d %d to index %d\n",
	     out.numberoftriangles,
	     faces[i].points[0], faces[i].points[1], faces[i].points[2],
	     faces[i].neighbors[0], faces[i].neighbors[1], faces[i].neighbors[2],
	     i);
#endif
      /* we just moved triangle index out.numberoftriangles to index i
	 update the neighbors to reflect the new index
       */
      for(int j=0;j<3;j++){
	int n = out.neighborlist[i*3 + j];
	if(n<0)continue;
	/* neighbor j is triangle n, look at triangle n and update it's neighbors */
	for(int k=0;k<3;k++){
	  if(out.numberoftriangles == out.neighborlist[n*3 + k]){
	    faces[n].neighbors[k] = out.neighborlist[n*3 + k] = i;
	    /*
	    printf("Updated triangle %d, neighbor was %d has been moved to %d\n",
		   n, out.numberoftriangles, i);
	    */
	  }
	}
      }

      i--;
    }else{
      for(int j=0;j<3;j++){
	faces[i].neighbors[j] = out.neighborlist[i * 3 + j];
	if(faces[i].neighbors[j]<0)
	  faces[i].neighbors[j]=-2;
      }
    }
  }
  //  sizeFaces = out.numberoftriangles;
  resizeTriangles(out.numberoftriangles);

  /* mark all the triangle vertices */
  int *used = (int *)alloca(numPoints()*sizeof(int));
  memset(used, 0, numPoints()*sizeof(int));
  for(unsigned int i=0;i<numTriangles();i++)
    for(int j=0;j<3;j++)
      used[faces[i].points[j]] = 1;

  /* assign new index numbers */
  unsigned int numUsed=0;
  for(unsigned int i=0;i<numPoints();i++){
    if(used[i]){
      used[i] = (int)numUsed;
      numUsed++;
    }else{
      //printf("Error, triangulation did not use point %d: ", i);vertices[i].print();printf("\n");
      used[i]=-1;
    }
  }

  if(numUsed!=numPoints()){
#if VERBOSE
    printf("Readjusting point indices to keep only %d of %d points\n", numUsed, numPoints());
#endif
    //assert(0);
    for(unsigned int i=0;i<numPoints();i++)
      if(used[i]>=0)
	vertices[used[i]] = vertices[i];
    for(unsigned int i=0;i<numTriangles();i++)
      for(int j=0;j<3;j++)
	faces[i].points[j] = used[faces[i].points[j]];
    //sizeVertices = numUsed;
    resizeVertices(numUsed);
  }

  /* check the triangle neighbors */
  bool err=false;
  for(unsigned int i=0;i<numTriangles();i++){
    for(int j=0;j<3;j++){
      int n = faces[i].neighbors[j];
      if(-2 == n)continue;
      if((n < 0) || (n >= (int)numTriangles())){
	printf("Triangulation error with neighbors, triangle %d of %d set neighbor to %d\n",
	       i, numTriangles(), n);
	err = true;

	for(unsigned int k=0;k<numTriangles();k++){
	  for(int l=0;l<3;l++){
	    if((int)i == faces[k].neighbors[l]){
	      printf("Triangle %d, neighbor %d is %d\n", k, l, i);
	    }
	  }
	}

      }
    }
  }
  assert(!err);

  /* clean up */
  free(in.pointattributelist);
  free(in.pointmarkerlist);
  free(in.regionlist);
  free(out.pointlist);
  free(out.pointattributelist);
  free(out.pointmarkerlist);
  free(out.trianglelist);
  free(out.triangleattributelist);
  free(out.neighborlist);
}

/* triangulates the vertices and fills in the faces */
void TMAP::tmap_triangulate::triangulate(){
  triangulateMinimal();
  errorCheck();
}
