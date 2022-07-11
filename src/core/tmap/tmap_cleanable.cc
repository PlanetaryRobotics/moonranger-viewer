#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <tmap_cleanable.h>
#include <planeHelpers.h>

#define DEBUG 0

double TMAP::tmap_cleanable::computeMeanEdgeLength()const{
  if(numTriangles()<=0)return 0.0;

  double acc=0.0;
  for(unsigned int i=0;i<numTriangles();i++)
    acc += TMAP::trianglePerimeter(vertices[ faces[i].points[0] ],
				   vertices[ faces[i].points[1] ],
				   vertices[ faces[i].points[2] ]);
  return acc/(double)(3*numTriangles());
}

double TMAP::tmap_cleanable::computeEdgeLengthStd()const{
  if(numTriangles()<=0)return 0.0;

  assert(edgeMean>=0.0);
  double acc=0;

  double lengths[3]={0.0, 0.0, 0.0};
  for(unsigned int i=0;i<numTriangles();i++){
    TMAP::triangleLengths(vertices[ faces[i].points[0] ],
			  vertices[ faces[i].points[1] ],
			  vertices[ faces[i].points[2] ],
			  lengths);
    for(int j=0;j<3;j++){
      lengths[j] -= edgeMean;
      acc += lengths[j]*lengths[j];
    }
  }

  return sqrt(acc/((double)(3*numTriangles())));
}


static bool shouldPruneEdge(TMAP::tmap_cleanable::cleanableTMAPfilters filter,
			    double threshold,
			    double length,
			    double edgeMean, double edgeStd){
  switch(filter){
  case TMAP::tmap_cleanable::deviationsLong:
  case TMAP::tmap_cleanable::deviationsLongPoints:
    return (fabs(length-edgeMean) > edgeStd*threshold);

  case TMAP::tmap_cleanable::deviationsShort:
  case TMAP::tmap_cleanable::deviationsShortPoints:
    return (fabs(length-edgeMean) < edgeStd*threshold);

  default:
    /* called with invalid filter parameter */
    assert(0);
    abort();
  }

  return false;
}

static bool shouldPrune(TMAP::tmap_cleanable::cleanableTMAPfilters filter, 
			double threshold,
			double lengths[3],
			double edgeMean,
			double edgeStd){
  switch(filter){
  case TMAP::tmap_cleanable::deviationsLong:
  case TMAP::tmap_cleanable::deviationsLongPoints:
    for(int i=0;i<3;i++)
      if(fabs(lengths[i]-edgeMean) > edgeStd*threshold)
	return true;
    return false;

  case TMAP::tmap_cleanable::deviationsShort:
  case TMAP::tmap_cleanable::deviationsShortPoints:
    for(int i=0;i<3;i++)
      if(fabs(lengths[i]-edgeMean) < edgeStd*threshold)
	return true;
    return false;

  default:
    /* called with invalid filter parameter */
    assert(0);
    abort();
  }

  return false;
}

/* triangle ind is an exterior, 
 * this function fills in a and b with the points of the exterior edge
 */
static void pickTwoExteriorPoints(const RASM::triangle *faces, int ind, 
				  unsigned int &a, unsigned int &b){

  /* find the candidate point that is shared by two neighbors */
  unsigned int candidates[3]={0,0,0};
  for(int neighbor=0;neighbor<3;neighbor++){
    int neighborIndex = faces[ind].neighbors[neighbor];
    if(neighborIndex<0)continue;/* this is the exterior */

    for(int point=0;point<3;point++){
      unsigned int pointIndex = faces[neighborIndex].points[point];
      for(int i=0;i<3;i++)
	if(pointIndex == faces[ind].points[i])
	  candidates[i]++;
    }
  }
  /* two of the candidates should be set to 1, and the other to 2 
   * the point set to 2 is safe
   */
  if(2 == candidates[0]){
    a = faces[ind].points[1];
    b = faces[ind].points[2];
  }else if(2 == candidates[1]){
    a = faces[ind].points[0];
    b = faces[ind].points[2];
  }else{
    assert(2 == candidates[2]);
    a = faces[ind].points[0];
    b = faces[ind].points[1];
  }

}

static unsigned int inline numMissingNeighbors(const RASM::triangle &t){
  unsigned int ret=0;
  for(unsigned int i=0;i<3;i++)if(t.neighbors[i]<0)ret++;
  return ret;
}

/* given two point indices, set the mask for any triangles that use either */
#if 0
static void markTrianglesThatUse(const RASM::triangle *faces,
				 unsigned int _sizeFaces, 
				 unsigned int index1, unsigned int index2,
				 bool *fMask){

  /* this is the SLOW version, do not use */
  assert(0);
  abort();

  for(unsigned int i=0;i<_sizeFaces;i++)
    for(unsigned int j=0;j<3;j++)
      if((index1 == faces[i].points[j]) || (index2 == faces[i].points[j]))
	fMask[i] = true;
}
#endif

/* given a point indices, set the mask for any triangles that use it
 * use the associations lookup to do this efficiently
 * returns the number of faces masked
 */
static unsigned int markTrianglesThatUse(const RASM::triangle *faces,
					 unsigned int _sizeFaces, 
					 unsigned int vindex,
					 bool *fMask,
					 unsigned int **triangleAssociations){

  assert(triangleAssociations);
  assert(triangleAssociations[vindex]);
  assert(triangleAssociations[vindex][0]>0);

  unsigned int numMasked=0;

  for(unsigned int j=0;j<triangleAssociations[vindex][0];j++){
    unsigned int tIndex = triangleAssociations[vindex][j+1];
    if(!fMask[tIndex]){
#if DEBUG
      printf("Flagging face %d of %d\n", tIndex, _sizeFaces);
#endif
      fMask[tIndex]=true;
      numMasked++;
    }
  }

  return numMasked;
}

unsigned int TMAP::tmap_cleanable::maskBadData(TMAP::tmap_cleanable::cleanableTMAPfilters filter,
					       double threshold){
  if(!fMask){
    fMask = (bool *)calloc(numTriangles(), sizeof(bool));
  }

  unsigned int numMasked=0;

  switch(filter){
  case noop:
    break;

  case deviationsLong:
  case deviationsShort:

    /* compute mean and std of edge lengths if needed */
    getMeanEdgeLength();
    getEdgeLengthStd();

    for(unsigned int i=0;i<numTriangles();i++){
      double l[3];
      TMAP::triangleLengths(vertices[faces[i].points[0]],
			    vertices[faces[i].points[1]],
			    vertices[faces[i].points[2]], l);
      if(!fMask[i] && shouldPrune(filter, threshold, l, edgeMean, edgeStd)){
	fMask[i]=true;
	numMasked++;
      }
    }
    break;

  case deviationsShortPoints:
  case deviationsLongPoints:
    /* compute mean and std of edge lengths if needed */
    getMeanEdgeLength();
    getEdgeLengthStd();

    for(unsigned int i=0;i<numTriangles();i++){
      double l[3];
      TMAP::triangleLengths(vertices[faces[i].points[0]],
			    vertices[faces[i].points[1]],
			    vertices[faces[i].points[2]], l);
      for(unsigned int k=0;k<3;k++){
	if(shouldPruneEdge(filter, threshold, l[k], edgeMean, edgeStd)){
	  numMasked += markTrianglesThatUse(faces, numTriangles(),
					    faces[i].points[k],
					    fMask,  triangleAssociations);
	  numMasked += markTrianglesThatUse(faces, numTriangles(),
					    faces[i].points[(k+1)%3],
					    fMask, triangleAssociations);
	}
      }
    }
    break;

  case missingNeighbor:

    for(unsigned int i=0;i<numTriangles();i++){
      if((-1 == faces[i].neighbors[0]) || 
	 (-1 == faces[i].neighbors[1]) || 
	 (-1 == faces[i].neighbors[2]) ){
	fillInCenters();
	fillInAssociations();
	fillInNeighbors();
      }

      assert((-1 != faces[i].neighbors[0]) &&
	     (-1 != faces[i].neighbors[1]) &&
	     (-1 != faces[i].neighbors[2]) );

      if(!fMask[i] &&
	 ((-2 == faces[i].neighbors[0]) ||
	  (-2 == faces[i].neighbors[1]) ||
	  (-2 == faces[i].neighbors[2]) )  ){
	fMask[i]=true;
	numMasked++;
      }
    }
    break;

  case unusedPoints:
    /* this is a no-op, we just needed to initialize the masks */
    break;

  case minAngle:
    for(unsigned int i=0;i<numTriangles();i++){
      if(fMask[i])continue;/* already masked, don't compute anything */

      /* the normal vector must point atleast threshold from level */
      RASM::point3d a = vertices[faces[i].points[0]]; 
      RASM::point3d b = vertices[faces[i].points[1]]; 
      RASM::point3d c = vertices[faces[i].points[2]]; 
      b-=a; 
      c-=a;
      a = b*c; /* 'a' is the direction of the normal vector */

      double z = RASM_TO_METERS(a.Z());
      double length = dist3d(RASM::point3d(), a);
      z /= length; /* 'z' is the z component of the normal */

      /* check if it matches the test */
      double m = sin(threshold);
      if(fabs(z) < fabs(m)){
	fMask[i]=true;
	numMasked++;
      }
    }
    break;

  case borderVertices:
    for(unsigned int i=0;i<numTriangles();i++){
      switch(numMissingNeighbors(faces[i])){
      case 0:/* missing 0 neighbors */
	break;
      case 1:/* missing 1 neighbor */
	do{
	  unsigned int a, b;
	  pickTwoExteriorPoints(faces, i, a, b);

	  numMasked += markTrianglesThatUse(faces, numTriangles(), a,
					    fMask,  triangleAssociations);
	  numMasked += markTrianglesThatUse(faces, numTriangles(), b,
					    fMask, triangleAssociations);

	}while(0);
	break;
      case 2:/* missing 2 neighbors... */
	if(!fMask[i]){
	  fMask[i] = true;
	  numMasked++;
	}
	break;
      case 3:/* missing 3 neighbors, floating triangle?!? */
	if(!fMask[i]){
	  fMask[i] = true;
	  numMasked++;
	}
	break;
      default:/* should be 0-3 neighbors */
	assert(0);
      }

    }
    break;

  default:
    printf("Have not implimented filter %d\n", filter);
    assert(0);    
  }

  //printf("Filter %d, threshold %g masked %d of %d faces\n", filter, threshold, numMasked, numTriangles());
  return numMasked;
}

unsigned int TMAP::tmap_cleanable::maskBadData(TMAP::tmap_cleanable::cleanableTMAPfilters filter){
  switch(filter){
  case deviationsLong:
  case deviationsShort:
  case deviationsLongPoints:
  case deviationsShortPoints:
  case minAngle:
    printf("Specify a threshold!\n");
    assert(0);
    abort();
  default:
    return maskBadData(filter, 0);
  }
}

unsigned int TMAP::tmap_cleanable::maskFarData(RASM_UNITS distance,
					       const RASM::point2d &center){
  assert(numPoints()>0);
  if(!triangleAssociations)fillInAssociations();
  assert(triangleAssociations);

  maskBadData(noop);

  unsigned int numMasked = 0;

  const double dist = RASM_TO_METERS(distance);
  const double distSq = dist*dist;

#if DEBUG
  printf("Masking points more than %f meters from (%f,%f)\n",
	 RASM_TO_METERS(distance),
	 RASM_TO_METERS(center.X()),
	 RASM_TO_METERS(center.Y()));
#endif

  for(unsigned int i=0;i<numPoints();i++){
    if(distSq2d(center, vertices[i])<distSq)continue;
#if DEBUG
    printf("Vertex %d of %d is far\n", i, numPoints());
#endif
    /* remove all faces that use this point */
    assert(triangleAssociations[i]);
    assert(triangleAssociations[i][0]>0);
    for(unsigned int j=0;j<triangleAssociations[i][0];j++){
      unsigned int tIndex = triangleAssociations[i][j+1];
      if(!fMask[tIndex]){
#if DEBUG
	printf("Flagging face %d of %d\n", tIndex, numTriangles());
#endif
	fMask[tIndex]=1;
	numMasked++;
      }
    }
  }

  return numMasked;
}

void TMAP::tmap_cleanable::removeBadData(){
  if(!fMask)return;
#if DEBUG
  for(unsigned int i=0;i<numTriangles();i++){
    for(int j=0;j<3;j++){
      assert(faces[i].points[j]>=0);
      assert(faces[i].points[j]<numPoints());
    }
  }
#endif

  bool needAssociations = (NULL != triangleAssociations);
  if(needAssociations)
    removeAssociations();

  /* initially mask all the points */

  /* -2 means unused
     -1 means used, but not sure what the final index is
     >=0 means used and contains the new index
   */
  int *vMap = (int *)alloca(numPoints()*sizeof(int));
  for(unsigned int i=0;i<numPoints();i++)
    vMap[i]=-2;

  /* unmask the ones we want to keep */
  for(unsigned int i=0;i<numTriangles();i++){
    if(!fMask[i]){
      vMap[faces[i].points[0]] = -1;
      vMap[faces[i].points[1]] = -1;
      vMap[faces[i].points[2]] = -1;
    }
  }

  /* remove unused points from the array */
  unsigned int goodVertices=0;

  /* find the first one to remove */
  while((goodVertices<numPoints()) && (-1 == vMap[goodVertices])){
    vMap[goodVertices]=goodVertices;
    goodVertices++;
  }

  /* scan through the remaining ones
   * and shuffle the data to put the good ones in front
   */
  for(unsigned int i=goodVertices;i<numPoints();i++){
    if(-1==vMap[i]){
      /* keep point i, it's new index is goodVertices */
      vertices[goodVertices] = vertices[i];
      vMap[i] = goodVertices++;
    }
  }
  /* clear out remaining points */

#if DEBUG
  printf("Reduced from %d to %d points\n", numPoints(), goodVertices);
#endif
  //  sizeVertices=goodVertices;
  resizeVertices(goodVertices);

  /* remove unused faces from the array */
  unsigned int goodFaces=0;
  /* find the first bad face */
  while(goodFaces<numTriangles() && !fMask[goodFaces])goodFaces++;
  for(unsigned int i=goodFaces;i<numTriangles();i++){
    /* keep face i, its new index is goodFaces */
    if(!fMask[i])
      faces[goodFaces++] = faces[i];
  }
  //printf("Reduced from %d to %d faces\n", numTriangles(), goodFaces);
  //  sizeFaces=goodFaces;
  resizeTriangles(goodFaces);

  /* shuffle around the vertex indices */
  for(unsigned int i=0;i<numTriangles();i++){
#if DEBUG
    for(int j=0;j<3;j++)
      if(vMap[faces[i].points[j]]<0){
	printf("Error remapping corner %d of triangle %d, old point %d, new point %d??\n",
	       j, i, faces[i].points[j], vMap[faces[i].points[j]]);
	assert(0);
      }
#endif

    for(int j=0;j<3;j++)
      faces[i].points[j] = vMap[faces[i].points[j]];
  }

  free(fMask);
  fMask = NULL;
  if(centers){
    free(centers);
    centers = NULL;
    fillInCenters();
  }
  if(needAssociations)
    fillInAssociations();
  removeNeighbors();
  fillInNeighbors();
  invalidateEdgeLengths();
  errorCheck();
}

/* returns true if target is near the line between start and end */
bool checkLine(RASM::point2d target,
	       const RASM::point2d &start,
	       RASM::point2d end,
	       double allowableAngle){
  target -= start;
  end -= start;

  /* get the heading to the target and goal */
  double theta_target = atan2(RASM_TO_METERS(target.Y()), 
			      RASM_TO_METERS(target.X()));
  double theta_end = atan2(RASM_TO_METERS(end.Y()), 
			   RASM_TO_METERS(end.X()));
  
  /* get the different between the headings */
  double theta_rel = theta_target - theta_end;
  if(theta_rel < -M_PI)theta_rel+=(2.0*M_PI);
  if(theta_rel >  M_PI)theta_rel-=(2.0*M_PI);
  theta_rel = fabs(theta_rel);

  /* large heading difference, must be behind us */
  return (theta_rel<allowableAngle);
}

unsigned int TMAP::tmap_cleanable::maskDataLine(const RASM::point2d &start,
						const RASM::point2d &end,
						RASM_UNITS minDist,
						RASM_UNITS maxDist,
						double allowableAngle){
  assert(numPoints()>0);
  if(!triangleAssociations)fillInAssociations();
  assert(triangleAssociations);

  maskBadData(noop);

  unsigned int numMasked = 0;

  const double maxDist_d = RASM_TO_METERS(maxDist);
  const double minDist_d = RASM_TO_METERS(minDist);
  const double maxDistSq = maxDist_d*maxDist_d;
  const double minDistSq = minDist_d*minDist_d;

#if DEBUG
  printf("Masking on line between (%f,%f) and (%f,%f) with distance %f,%f\n",
	 RASM_TO_METERS(start.X()),
	 RASM_TO_METERS(start.Y()),
	 RASM_TO_METERS(end.X()),
	 RASM_TO_METERS(end.Y()),
	 RASM_TO_METERS(minDist),
	 RASM_TO_METERS(maxDist));
#endif

  for(unsigned int i=0;i<numPoints();i++){
    if(distSq2d(start, vertices[i])<minDistSq)continue;
    if(distSq2d(start, vertices[i])<maxDistSq &&
       checkLine(vertices[i], start, end, allowableAngle))continue;
#if DEBUG
    printf("Vertex %d of %d is far\n", i, numPoints());
#endif
    /* remove all faces that use this point */
    assert(triangleAssociations[i]);
    assert(triangleAssociations[i][0]>0);
    for(unsigned int j=0;j<triangleAssociations[i][0];j++){
      unsigned int tIndex = triangleAssociations[i][j+1];
      if(!fMask[tIndex]){
#if DEBUG
	printf("Flagging face %d of %d\n", tIndex, numTriangles());
#endif
	fMask[tIndex]=1;
	numMasked++;
      }
    }
  }
  return numMasked;
}

