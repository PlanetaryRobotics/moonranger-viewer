#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <tmap_base.h>

#define TMAP_ERROR_CHECK 0

TMAP::tmap_base::tmap_base()
  :sizeVertices(0), capacityVertices(0), 
   sizeFaces(0),    capacityFaces(0),
   vertices(NULL),  faces(NULL)
   
{}

TMAP::tmap_base::tmap_base(const char *filename)
  :sizeVertices(0), capacityVertices(0),
   sizeFaces(0),    capacityFaces(0),
   vertices(NULL),  faces(NULL)
{
  readFromFile(filename);
}

void TMAP::tmap_base::readFromFile(const char *filename){
  //printf("Loading %s\n", filename);
  FILE *f = fopen(filename, "r");
  assert(f);

  sizeVertices = sizeFaces = 0;
  capacityVertices = capacityFaces = 0;
  if(vertices){
    free(vertices);
    vertices=NULL;
  }
  if(faces){
    free(faces);
    faces=NULL;
  }

  /* read each line,
   * add the raw data into either the vertices or faces array
   */
  char buf[64];
  while(fgets(buf, sizeof(buf), f)){
    float x,y,z;
    if('v' == buf[0]){
      /* this line should be a vertex/point */
      if((3==sscanf(buf+2, "%f %f %f", &x, &y, &z)) &&
	 std::isfinite(x) && std::isfinite(y) && std::isfinite(z)){
	addPoint(RASM::point3d(METERS_TO_RASM(x),
			       METERS_TO_RASM(y),
			       METERS_TO_RASM(z)));
      }

    }else if('f' == buf[0]){
      /* this line should be a face/triangle */
      int a, b, c;
      if((3==sscanf(buf+2, "%d %d %d", &a, &b, &c)) && (a>0 && b>0 && c>0)){
	unsigned int tri[3];
	tri[0] = (unsigned int)a-1;
	tri[1] = (unsigned int)b-1;
	tri[2] = (unsigned int)c-1;
	int neighbors[3] = {-1,-1,-1};
	addTriangle(tri, neighbors);
      }

      /* unknown start of line, see if it's 3 numbers */
    }else if((3 == sscanf(buf, "%f %f %f", &x, &y, &z)) && 
	     std::isfinite(x) && std::isfinite(y) && std::isfinite(z)){
      addPoint(RASM::point3d(METERS_TO_RASM(x),
			     METERS_TO_RASM(y),
			     METERS_TO_RASM(z)));

    }else{
      /* unknown line, skip it */
    }
  }
  fclose(f);

  /* done reading the raw data,
   * do some error checking
   */
  for(unsigned int i=0;i<sizeFaces;i++){
    for(int j=0;j<3;j++){
      int v = faces[i].points[j];
      if((v<0) || (v>=(int)sizeVertices)){
	printf("Error, triangle %d, point %d is %d, should be 0 to %d\n", i, j, v, sizeVertices);
	assert(0);
      }
    }
  }
}

#ifndef FAST_WRITE_TO_FILE
#define FAST_WRITE_TO_FILE 1
#endif
#if FAST_WRITE_TO_FILE
#define VBUF_ADD(c) do{*buf = (c);buf++;len++;}while(0)
static inline int vbufPosInt3(char *buf, int v){
  assert(v>=0 && v<1000);
  buf[0] = '0' + (v/100);
  buf[1] = '0' + ((v/10)%10);
  buf[2] = '0' + (v%10);
  return 3;
}

static inline int vbufPosInt(char *buf, int v){
  int len=0;
  char rev[32];
  int N=0;

  /* create the reverse string */
  while(v>0){
    rev[N] = '0' + (v%10);
    N++;
    v/=10;
  }

  /* copy it to buf backwards (warning: --N vs len++) */
  while(N>0)
    buf[len++] = rev[--N];

  return len;
}

static inline int vbufDouble(char *buf, double v){
  int l, len=0;
  int iv, iv3;
  const double epsilon = 0.0001;/* below printing resolution */
  if(v<0.0){
    VBUF_ADD('-');
    v -= epsilon;
    iv = (int)(-v);
    iv3 = ((int)(v*-1000.0))%1000;
  }else{
    v += epsilon;
    iv = (int)(v);
    iv3 = ((int)(v*1000.0))%1000;
  }

  if(0 == iv){
    VBUF_ADD('0');
  }else{
    l = vbufPosInt(buf, iv);len+=l;buf+=l;
  }

  VBUF_ADD('.');
  l = vbufPosInt3(buf, iv3);len+=l;buf+=l;
  return len;
}

static inline void vbufXYZ(FILE *f, double x, double y, double z){
  char buffer[256];
  char *buf = buffer;
  int l, len = 0;
  VBUF_ADD('v');

  VBUF_ADD(' ');
  l = vbufDouble(buf, x);buf+=l;len+=l;
  VBUF_ADD(' ');
  l = vbufDouble(buf, y);buf+=l;len+=l;
  VBUF_ADD(' ');
  l = vbufDouble(buf, z);buf+=l;len+=l;

  VBUF_ADD('\n');
  fwrite(buffer, 1, len, f);
}
		      
static inline void vbufABC(FILE *f, int a, int b, int c){
  char buffer[256];
  char *buf = buffer;
  int l, len = 0;
  VBUF_ADD('f');

  VBUF_ADD(' ');
  l = vbufPosInt(buf, a);buf+=l;len+=l;
  VBUF_ADD(' ');
  l = vbufPosInt(buf, b);buf+=l;len+=l;
  VBUF_ADD(' ');
  l = vbufPosInt(buf, c);buf+=l;len+=l;

  VBUF_ADD('\n');
  fwrite(buffer, 1, len, f);
}
#undef VBUF_ADD
#endif/* end of FAST_WRITE_TO_FILE */

void TMAP::tmap_base::writeToFile(const char* filename) const
{
  RASM::pose pose; // should cause 0's to be written in pose comment
  writeToFile(filename, pose); 
}

void TMAP::tmap_base::writeToFile(const char *filename,
				  const RASM::pose& pose) const
{
  FILE *f = fopen(filename, "w");
  if(!f){
    printf("Error, unable to write to \"%s\"\n", filename);
    return;
  }

  RASM::point3d position = pose.getPosition();
  fprintf(f, "# x %f m y %f m z %f m roll %f rad pitch %f rad yaw %f rad timestamp %f\n",
	  RASM_TO_METERS(position.X()),
	  RASM_TO_METERS(position.Y()),
	  RASM_TO_METERS(position.Z()),
	  pose.getOrientation().roll,
	  pose.getOrientation().pitch,
	  pose.getOrientation().yaw,
	  pose.getTime());

  for(unsigned int i=0;i<sizeVertices;i++){
#if FAST_WRITE_TO_FILE
    vbufXYZ(f,
	    RASM_TO_METERS(vertices[i].coord3d[0]), 
	    RASM_TO_METERS(vertices[i].coord3d[1]), 
	    RASM_TO_METERS(vertices[i].coord3d[2]));
#else
    fprintf(f, "v %0.3f %0.3f %0.3f\n", 
	    RASM_TO_METERS(vertices[i].coord3d[0]), 
	    RASM_TO_METERS(vertices[i].coord3d[1]), 
	    RASM_TO_METERS(vertices[i].coord3d[2]));
#endif
  }
  for(unsigned int i=0;i<sizeFaces;i++){
#if FAST_WRITE_TO_FILE
    vbufABC(f,
	    faces[i].points[0]+1,
	    faces[i].points[1]+1,
	    faces[i].points[2]+1);
#else
    fprintf(f, "f %d %d %d\n",
	    faces[i].points[0]+1,
	    faces[i].points[1]+1,
	    faces[i].points[2]+1);
#endif
  }
  fclose(f);
}

void TMAP::tmap_base::addPoint(const RASM::point3d &point){
  if(sizeVertices>= capacityVertices){
    capacityVertices = capacityVertices*2 + 4;
    vertices = (RASM::point3d *)realloc(vertices, capacityVertices * sizeof(RASM::point3d));
  }
  vertices[sizeVertices++] = point;
}

void TMAP::tmap_base::addTriangle(const unsigned int tri[3],
				  const int neighbors[3]){
  if(sizeFaces>= capacityFaces){
    capacityFaces = sizeFaces*2 + 4;
    faces = (RASM::triangle *)realloc(faces, capacityFaces * sizeof(RASM::triangle));
  }

  for(int i=0;i<3;i++)
    faces[sizeFaces].points[i] = tri[i];
  for(int i=0;i<3;i++)
    faces[sizeFaces].neighbors[i]=neighbors[i];

  sizeFaces++;
}

void TMAP::tmap_base::addInterestPoint(const RASM::point3d &point){
  interestPoints.push_back(point);
}

void TMAP::tmap_base::removePoint(unsigned int ind){
  assert(ind < sizeVertices);
  sizeVertices--;
  /* swap with the last point */
  if(ind <= sizeVertices)
    vertices[ind] = vertices[sizeVertices];
}

void TMAP::tmap_base::removeInterestPoint(unsigned int ind){
  assert(ind < interestPoints.size());
  interestPoints.erase(interestPoints.begin() + ind);
}

void TMAP::tmap_base::replacePoint(unsigned int ind,
				   const RASM::point3d &point){
  assert(ind<sizeVertices);
  vertices[ind] = point;
}

void TMAP::tmap_base::replaceTriangle(unsigned int ind,
				      const unsigned int tri[3],
				      const int neighbors[3]){
  assert(ind<sizeFaces);
  for(int i=0;i<3;i++)
    faces[ind].points[i] = tri[i];
  for(int i=0;i<3;i++)
    faces[ind].neighbors[i]=neighbors[i];
}

void TMAP::tmap_base::replaceInterestPoint(unsigned int ind,
				   const RASM::point3d &point){
  assert(ind<interestPoints.size());
  interestPoints[ind] = point;
}

void TMAP::tmap_base::removeNeighbors(){
  for(unsigned int i=0;i < sizeFaces;i++)
    for(int j=0;j<3;j++)
      faces[i].neighbors[j] = -1;
}


int TMAP::tmap_base::checkNeighbors() const{
  int ret=0;

  /* check that all the neighbors make sense */
  for(unsigned int i=0;i<numTriangles();i++){
    const RASM::triangle t = getTriangle(i);
    for(int j=0;j<3;j++){
      if(t.neighbors[j] == -2)continue;
      if((t.neighbors[j] < 0                   ) || 
	 (t.neighbors[j] >= (int)numTriangles()) ){
	printf("Error, triangle %d has invalid neighbor %d: %d (should be 0 to %d or -2)\n",
	       i, j, t.neighbors[j], numTriangles());
	ret = -1;
      }
    }
  }

  /* check that neighbors refer back */
  for(unsigned int i=0;i<numTriangles();i++){
    RASM::triangle t = getTriangle(i);
    for(int j=0;j<3;j++){
      if(t.neighbors[j] < 0)continue;
      const RASM::triangle n = getTriangle(t.neighbors[j]);
      if((n.neighbors[0] != (int)i) && 
	 (n.neighbors[1] != (int)i) && 
	 (n.neighbors[2] != (int)i) ){
	printf("Error, triangle %d, neighbor %d is %d, but that has neighbors %d %d %d\n",
	       i, j, t.neighbors[j],
	       n.neighbors[0], n.neighbors[1], n.neighbors[2]);
	ret = -1;
      }
    }
  }

  return ret;
}



void TMAP::tmap_base::clearPointsAndTriangles(){
  /* don't actually release memory, leave the capacity alone */
  sizeVertices = sizeFaces = 0;
}

void TMAP::tmap_base::print() const{

  for(unsigned int i=0;i<sizeVertices;i++){
    printf("Point %d ", i);
    vertices[i].print();
    printf("\n");
  }

  for(unsigned int i=0;i<sizeFaces;i++){
    printf("Face %d Points %d,%d,%d with neighbors %d,%d,%d\n", i, 
	   faces[i].points[0], 
	   faces[i].points[1], 
	   faces[i].points[2],
	   faces[i].neighbors[0], 
	   faces[i].neighbors[1], 
	   faces[i].neighbors[2]);
  }
}

TMAP::tmap_base::~tmap_base(){
  if(capacityVertices>0){
    sizeVertices = capacityVertices = 0;
    free(vertices);
    vertices = NULL;
  }

  if(capacityFaces>0){
    sizeFaces = capacityFaces = 0;
    free(faces);
    faces = NULL;
  }
}

void TMAP::tmap_base::errorCheck() const{
#if TMAP_ERROR_CHECK
  int error=0;
  for(unsigned int i=0;i<sizeVertices;i++){
    for(unsigned int j=i+1;j<sizeVertices;j++){
      if(distSq2d(vertices[i], vertices[j]) < RASM_TO_METERS(RASM_EPSILON)*RASM_TO_METERS(RASM_EPSILON)){
	printf("Error, points %d and %d are close together:\n", i, j);
	printf(" point %d ", i);vertices[i].print();printf("\n");
	printf(" point %d ", j);vertices[j].print();printf("\n");
	error=1;
      }
    }
  }

  for(unsigned int i=0;i<sizeFaces;i++){
    for(int j=0;j<3;j++){
      if(faces[i].points[j]>=sizeVertices){
	printf("Triangle %d, point %d is %d, should be 0 to %d\n",
	       i, j, faces[i].points[j], sizeVertices);
	error=1;
      }
    }
  }

  for(unsigned int i=0;i<sizeFaces;i++){
    for(int j=0;j<3;j++){
      if((faces[i].neighbors[j] >= (int)sizeFaces) || 
	 (faces[i].neighbors[j] <- 2             ) ||
	 (faces[i].neighbors[j] == (int)i        ) ){
	printf("Triangle %d, neighbor %d is %d, should be -2 to %d (and not itself)\n",
	       i, j, faces[i].neighbors[j], sizeFaces);
	error=1;
      }
    }
  }

  if(-1 == checkNeighbors())
    error=1;

  /* look for unused points */
  int *used = new int[sizeVertices];
  for(unsigned int i=0;i<sizeVertices;i++)
    used[i] = 0;
  for(unsigned int i=0;i<sizeFaces;i++)
    for(int j=0;j<3;j++)
      used[faces[i].points[j]] = 1;
  for(unsigned int i=0;i<sizeVertices;i++){
    if(!used[i]){
      printf("unused point %d ", i);vertices[i].print();printf("\n");
      error=1;
    }
  }
  delete[] used;


  /* check all edges,
   * make sure there are at most two triangle that share it
   */
  for(unsigned int i=0; i<sizeFaces;i++){
    TMAP::edge edgeA(faces[i].points[0], faces[i].points[1], i);
    TMAP::edge edgeB(faces[i].points[1], faces[i].points[2], i);
    TMAP::edge edgeC(faces[i].points[2], faces[i].points[0], i);

    for(unsigned int j=i+1; j<sizeFaces;j++){
      if(edgeA.match(faces[j].points[0],faces[j].points[1],faces[j].points[2]))
	if(-1 == edgeA.setNeighbor(j))
	  error=1;
      if(edgeB.match(faces[j].points[0],faces[j].points[1],faces[j].points[2]))
	if(-1 == edgeB.setNeighbor(j))
	  error=1;
      if(edgeC.match(faces[j].points[0],faces[j].points[1],faces[j].points[2]))
	if(-1 == edgeC.setNeighbor(j))
	  error=1;
    }
  }

  if(error){
    printf("This tmap has errors (see above)\n");
    printf("There are %d vertices and %d faces\n", sizeVertices, sizeFaces);
    assert(0);
  }
#endif
}

