#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <tmap_associated.h>


TMAP::tmap_associated::tmap_associated()
  :tmap_base(), centers(NULL), triangleAssociations(NULL),
   allocatedAssociationSize(0)
{}

TMAP::tmap_associated::tmap_associated(const char *filename)
  :tmap_base(filename), centers(NULL), triangleAssociations(NULL),
   allocatedAssociationSize(0)
{}


int TMAP::tmap_associated::findTriangleWithVertices(unsigned int p1,
						    unsigned int p2, 
						    unsigned int ignoreMe) const{
  assert(triangleAssociations);
  assert(allocatedAssociationSize == numPoints());
  unsigned int nA = triangleAssociations[p1][0];
  unsigned int nB = triangleAssociations[p2][0];
  assert(nA>0 && nB>0);

  unsigned int *trianglesA=&(triangleAssociations[p1][1]);
  unsigned int *trianglesB=&(triangleAssociations[p2][1]);

  for(unsigned int i=0;i<nA;i++){
    if(trianglesA[i] == ignoreMe)continue;
    for(unsigned int j=0;j<nB;j++){
      if(trianglesB[j] == ignoreMe)continue;
      if(trianglesA[i] == trianglesB[j])
	return trianglesA[i];
    }
  }

  return -2;
}

/* fills in the association lookup table */
void TMAP::tmap_associated::fillInAssociations(){ 
  if(triangleAssociations){
    assert(allocatedAssociationSize == numPoints());
    return;
  }

  /* initialize the lookup table */
  allocatedAssociationSize = numPoints();
  triangleAssociations = (unsigned int **)malloc(numPoints()*sizeof(unsigned int *));
  assert(triangleAssociations);
  for(unsigned int i=0;i<numPoints();i++){
    triangleAssociations[i] = (unsigned int *)malloc(sizeof(unsigned int));
    assert(triangleAssociations[i]);
    triangleAssociations[i][0] = 0;
  }

  /* fill in the associations for each triangle */
  for(unsigned int triangleIndex=0;triangleIndex<numTriangles();triangleIndex++){
    for(int j=0;j<3;j++){
      unsigned int pointIndex = faces[triangleIndex].points[j];
      assert(pointIndex < numPoints());
      unsigned int n = ++triangleAssociations[pointIndex][0];
      triangleAssociations[pointIndex] = (unsigned int *)realloc(triangleAssociations[pointIndex], 
								 (1+triangleAssociations[pointIndex][0])*sizeof(unsigned int));
      triangleAssociations[pointIndex][n] = triangleIndex;
    }
  }
}

void TMAP::tmap_associated::removeAssociations(){
  if(!triangleAssociations){
    assert(0 == allocatedAssociationSize);
    return;
  }

  assert(allocatedAssociationSize == numPoints());
  for(unsigned int i=0;i<numPoints();i++){
    assert(triangleAssociations[i]);
    free(triangleAssociations[i]);
  }
  free(triangleAssociations);
  triangleAssociations=NULL;
  allocatedAssociationSize = 0;
}

void TMAP::tmap_associated::removeCenters(){
  if(centers){
    free(centers);
    centers = NULL;
  }
}
void TMAP::tmap_associated::fillInCenters(){
  if(centers)free(centers);
  centers = (RASM::point3d *)malloc(numTriangles() * sizeof(RASM::point3d));
  for(unsigned int i=0;i<numTriangles();i++){
    for(int j=0;j<3;j++){
      if(faces[i].points[j]>=numPoints()){
	printf("Error, face %d, corner %d is vertex %d, should be <%d\n", 
	       i, j, faces[i].points[j], numPoints());
	assert(0);
      }
    }

    /* compute the center of this face */
    centers[i] = (vertices[faces[i].points[0]] + vertices[faces[i].points[1]] + vertices[faces[i].points[2]]);
    centers[i] /= (RASM_UNITS)3;

#if 0
    for(unsigned int j=0;j<numPoints();j++){
      if(distSq2d(centers[i], vertices[j]) < RASM_TO_METERS(RASM_EPSILON)*RASM_TO_METERS(RASM_EPSILON)){
	printf("Error, center of triangle %d close to point %d:\n", i, j);
	printf("center %d ", i);centers[i].print();printf("\n");
	printf("point %d ", j);vertices[j].print();printf("\n");
	printf("Center of triangle %d with points %d,%d,%d\n", i, faces[i].points[0], faces[i].points[1], faces[i].points[2]);
	for(int k=0;k<3;k++){
	  printf("  point %d ", faces[i].points[k]);vertices[faces[i].points[k]].print();printf("\n");
	}
	assert(0);
      }
    }
#endif

  }
}

void TMAP::tmap_associated::fillInNeighbors(){
  if(!triangleAssociations)fillInAssociations();
  errorCheck();
  for(unsigned int i=0;i < numTriangles();i++){
    //printf("Triangle %d has points %d %d %d\n", i, faces[i].points[0], faces[i].points[1], faces[i].points[2]);
    for(int j=0;j<3;j++){
      //if(faces[i].neighbors[j]>=0)continue;
      
      faces[i].neighbors[j] = findTriangleWithVertices(faces[i].points[j], 
						       faces[i].points[(j+1)%3],
						       i);
      /*
      printf(" added neighbor %d", faces[i].neighbors[j]);
      if(faces[i].neighbors[j]>=0)
	printf(" (that triangle has points %d %d %d)", 
	       faces[faces[i].neighbors[j]].points[0],
	       faces[faces[i].neighbors[j]].points[1],
	       faces[faces[i].neighbors[j]].points[2]);
      printf("\n");
      */
    }
  }

  errorCheck();
}


void TMAP::tmap_associated::printAssociations() const{
  if(!triangleAssociations){
    printf("Can't print associations, not computed yet\n");
    return;
  }

  assert(allocatedAssociationSize == numPoints());
  for(unsigned int i=0;i<numPoints();i++){
    unsigned int n = triangleAssociations[i][0];
    printf("Point %d has %d references:", i, n);
    for(unsigned int j=0;j<n;j++)printf(" %d", triangleAssociations[i][j+1]);
    printf("\n");
  }
}

void TMAP::tmap_associated::printStats() const{
  printf("%d of %d vertices < %d of %d faces\n", 
	 numPoints(), capacityPoints(),
	 numTriangles(), capacityTriangles());

  if(triangleAssociations)
    printf("triangleAssociations defined\n");
  else
    printf("triangleAssociations NOT defined\n");

  if(centers)
    printf("triangle centers defined\n");
  else
    printf("triangle centers NOT defined\n");
}

void TMAP::tmap_associated::clearPointsAndTriangles(){
  if(0 == numPoints())return;

  removeCenters();
  removeAssociations();
  tmap_base::clearPointsAndTriangles();
}

void TMAP::tmap_associated::print() const{
  printStats();
  tmap_base::print();
}

TMAP::tmap_associated::~tmap_associated(){
  if(triangleAssociations)removeAssociations();

  if(centers){
    free(centers);
    centers = NULL;
  }
}

void TMAP::tmap_associated::errorCheck() const{
#if TMAP_ERROR_CHECK
  tmap_base::errorCheck();

  int error=0;

  if(triangleAssociations){
    assert(allocatedAssociationSize == numPoints());
    for(unsigned int i=0;i<numPoints();i++){
      unsigned int N = triangleAssociations[i][0];
      for(unsigned int j=0;j<N;j++){
	if(triangleAssociations[i][j+1] >= numTriangles()){
	  printf("Vertex %d, association %d of %d, refers to triangle %d, should be 0 to %d\n",
		 i, j, N, triangleAssociations[i][j+1], numTriangles());
	  error=1;
	}
      }
    }
  }

  if(error){
    printf("This tmap has errors (see above)\n");
    printf("There are %d vertices and %d faces\n", numPoints(), numTriangles());
    assert(0);
  }
#endif
}

