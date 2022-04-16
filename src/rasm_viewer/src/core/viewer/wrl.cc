#include "wrl.h"
//#include "helpers.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/time.h>

#if !DISABLE_GL
#include <GL/gl.h>
#endif

static int verbose = 0;

/*** start of declaration of various helper functions ***/

/* readTuple() family of functions
 * loads an array of some type of data from file,
 * updates the line number and sets N to the pmalloc()'d] array length
 * returns -1 on error
 */
static int readTuple1f(FILE *f, unsigned int &lineNum,
		       unsigned int &n, float **arr);
static int readTuple3f(FILE *f, unsigned int &lineNum,
		       unsigned int &n, wrl::tuple3f **arr);
static int readTuple1i(FILE *f, unsigned int &lineNum,
		       unsigned int &n, int **arr);
static int readTuple3i4(FILE *f, unsigned int &lineNum,
			unsigned int &n, wrl::tuple3i **arr);//drop every 4th

static int readMaterialBinding(FILE *f, unsigned int &lineNum);
static int loadVertices(FILE *f, unsigned int &lineNum,
			unsigned int &n, wrl::tuple3f **vertices);
static int loadFaces(FILE *f, unsigned int &lineNum,
		     unsigned int &n, wrl::tuple3i **faces, int **materials);

/* overloaded append() function combines two arrays into a single one,
 * pass the array lengths and mutable arrays that get free()'d
 */
static wrl::tuple3f *append(unsigned int na, unsigned int nb,
			    wrl::tuple3f *a, wrl::tuple3f *b);
static wrl::tuple3i *append(unsigned int na, unsigned int nb,
			    wrl::tuple3i *a, wrl::tuple3i *b);
static float *append(unsigned int na, unsigned int nb, float *a, float *b);
static int *append(unsigned int na, unsigned int nb, int *a, int *b);


static bool match(const char *a, const char *b);
static double curTime();

static void applyTransform(const float rpy[3][3],
			   float dx, float dy, float dz, 
			   float &x, float &y, float &z);

/*** end of declaration of various helper functions ***/

/*** start of tuple3f helper functions ***/
static wrl::tuple3f minus(const wrl::tuple3f &a, const wrl::tuple3f &b){
  wrl::tuple3f ret;
  ret.x = a.x - b.x;
  ret.y = a.y - b.y;
  ret.z = a.z - b.z;
  return ret;
}

static wrl::tuple3f cross(const wrl::tuple3f &a, const wrl::tuple3f &b){
  wrl::tuple3f ret;
  ret.x = a.y*b.z - a.z*b.y;
  ret.y = a.z*b.x - a.x*b.z;
  ret.z = a.x*b.y - a.y*b.x;
  return ret;
}

static wrl::tuple3f normalize(const wrl::tuple3f &a){
  wrl::tuple3f ret;
  float f = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
  ret.x = a.x/f;
  ret.y = a.y/f;
  ret.z = a.z/f;
  return ret;
}
/*** end of tuple3f helper functions ***/


/*** start of wrl::materialData method definitions ***/

wrl::materialData::materialData()
  :numMaterials(0),
   ambientColor(NULL),
   diffuseColor(NULL),
   emissiveColor(NULL),
   specularColor(NULL),
   numShine(0),
   shininess(NULL),
   transparency(NULL){}

wrl::materialData::~materialData(){
  if(ambientColor)free(ambientColor);
  if(diffuseColor)free(diffuseColor);
  if(emissiveColor)free(emissiveColor);
  if(specularColor)free(specularColor);
  if(shininess)free(shininess);
  if(transparency)free(transparency);
}

int wrl::materialData::readFromFile(FILE *f, unsigned int &lineNum){
  char buf[1024];
  unsigned int n;
  wrl::tuple3f *t;
  float *t1;

  //printf("materialData:: read line %d\n", lineNum);

  // read ambient colors
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "ambientColor ")){
    fprintf(stderr, "expected ambientColor, got %s", buf);
    return -1;
  }
  n=0;t=NULL;
  if(-1 == readTuple3f(f, lineNum, n, &t)){
    fprintf(stderr, "Error reading tuple for ambient color\n");
    return -1;
  }
  ambientColor = append(numMaterials, n, ambientColor, t);

  //printf("read %d ambient colors (appending to %d prev)\n", n, numMaterials);
  unsigned int num = n;

  //save the number of colors

  // read diffuse
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "diffuseColor ")){
    fprintf(stderr, "expected diffuseColor, got \"%s\"", buf);
    return -1;
  }
  n=0;t=NULL;
  if(-1 == readTuple3f(f, lineNum, n, &t)){
    fprintf(stderr, "Error reading tuple for diffuse color\n");
    return -1;
  }
  diffuseColor = append(numMaterials, n, diffuseColor, t);

  //check the number of diffuse colors
  if(num != n){
    fprintf(stderr, "Error, got %d ambient colors and %d diffuse ones\n",
	    num, n);
    return -1;
  }

  // read emissive
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "emissiveColor ")){
    fprintf(stderr, "expected emissiveColor, got %s", buf);
    return -1;
  }
  n=0;t=NULL;
  if(-1 == readTuple3f(f, lineNum, n, &t)){
    fprintf(stderr, "Error reading tuple for emissive color\n");
    return -1;
  }
  emissiveColor = append(numMaterials, n, emissiveColor, t);

  //check the number of emissive colors
  if(num != n){
    fprintf(stderr, "Error, got %d ambient colors and %d emissive ones\n",
	    numMaterials, n);
    return -1;
  }

  // read specular
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "specularColor ")){
    fprintf(stderr, "expected specularColor, got %s", buf);
    return -1;
  }
  n=0;t=NULL;
  if(-1 == readTuple3f(f, lineNum, n, &t)){
    fprintf(stderr, "Error reading tuple for specular color\n");
    return -1;
  }
  specularColor = append(numMaterials, n, specularColor, t);

  //check the number of specular colors
  if(num != n){
    fprintf(stderr, "Error, got %d ambient colors and %d specular ones\n",
	    numMaterials, n);
    return -1;
  }

  /*** done with colors ***/
  numMaterials += num;

  // read shininess
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "shininess ")){
    fprintf(stderr, "expected shininess, got %s", buf);
    return -1;
  }
  n=0;t1=NULL;
  if(-1 == readTuple1f(f, lineNum, n, &t1)){
    fprintf(stderr, "Error reading shininess values\n");
    return -1;
  }
  shininess = append(numShine, n, shininess, t1);
  //printf("read %d shinines values (appending to %d prev)\n", n, numShine);

  num = n;

  // read transparency
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "transparency ")){
    fprintf(stderr, "expected transparency, got %s", buf);
    return -1;
  }
  n=0;t1=NULL;
  if(-1 == readTuple1f(f, lineNum, n, &t1)){
    fprintf(stderr, "Error reading transparency values\n");
    return -1;
  }
  transparency = append(numShine, n, transparency, t1);

  //check the number of transparency values
  if(num != n){
    fprintf(stderr, "Error, got %d shinines values and %d transparency values\n",
	    num, n);
    return -1;
  }

  /*** done with shininess ***/
  numShine+=num;

  // done reading material data
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "}")){
    fprintf(stderr, "expected \'}\', got %s", buf);
    return -1;
  }

  return 0;
}


bool wrl::materialData::sameAs(int ind, int i)const{
  assert((ind>=0) && (ind<(int)numMaterials));
  assert((i>=0) && (i<(int)numMaterials));
  const float EP = 1e-5;

  if(fabs(ambientColor[ind].x - ambientColor[i].x)>EP)return false;
  if(fabs(ambientColor[ind].y - ambientColor[i].y)>EP)return false;
  if(fabs(ambientColor[ind].z - ambientColor[i].z)>EP)return false;

  if(fabs(diffuseColor[ind].x - diffuseColor[i].x)>EP)return false;
  if(fabs(diffuseColor[ind].y - diffuseColor[i].y)>EP)return false;
  if(fabs(diffuseColor[ind].z - diffuseColor[i].z)>EP)return false;

  if(fabs(emissiveColor[ind].x - emissiveColor[i].x)>EP)return false;
  if(fabs(emissiveColor[ind].y - emissiveColor[i].y)>EP)return false;
  if(fabs(emissiveColor[ind].z - emissiveColor[i].z)>EP)return false;

  if(fabs(specularColor[ind].x - specularColor[i].x)>EP)return false;
  if(fabs(specularColor[ind].y - specularColor[i].y)>EP)return false;
  if(fabs(specularColor[ind].z - specularColor[i].z)>EP)return false;

  if(fabs(shininess[ind] - shininess[i])>EP)return false;
  if(fabs(transparency[ind] - transparency[i])>EP)return false;

  return true;
}

#if DISABLE_GL
void wrl::materialData::glColor(int ind)const{
  fprintf(stderr, "materialData::glColor() error, gl has been disabled\n");
  abort();
}
#else
void wrl::materialData::glColor(int ind)const{
  assert((ind>=0) && (ind<(int)numMaterials));

  //float alpha = 1.0 - transparency[ind];
  float alpha = 1.0;

#if 1

  glColor4f(ambientColor[ind].x,
	    ambientColor[ind].y,
	    ambientColor[ind].z,
	    alpha);

#else

  printf("Color %d: %0.3f %0.3f %0.3f, %0.3f %0.3f %0.3f, %0.3f %0.3f %0.3f, %0.3f %0.3f %0.3f, %0.3f\n",
	 ind,
	 ambientColor[ind].x,
	 ambientColor[ind].y,
	 ambientColor[ind].z,
	 diffuseColor[ind].x,
	 diffuseColor[ind].y,
	 diffuseColor[ind].z,
	 emissiveColor[ind].x,
	 emissiveColor[ind].y,
	 emissiveColor[ind].z,
	 specularColor[ind].x,
	 specularColor[ind].y,
	 specularColor[ind].z,
	 alpha);
  

  GLfloat c[4];

  c[0] = ambientColor[ind].x;
  c[1] = ambientColor[ind].y;
  c[2] = ambientColor[ind].z;
  c[3] = alpha;
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

  c[0] = diffuseColor[ind].x;
  c[1] = diffuseColor[ind].y;
  c[2] = diffuseColor[ind].z;
  c[3] = alpha;
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

  c[0] = emissiveColor[ind].x;
  c[1] = emissiveColor[ind].y;
  c[2] = emissiveColor[ind].z;
  c[3] = alpha;
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

  c[0] = specularColor[ind].x;
  c[1] = specularColor[ind].y;
  c[2] = specularColor[ind].z;
  c[3] = alpha;
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

  c[0] = shininess[ind];
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, c);
#endif


}
#endif

unsigned int wrl::materialData::marshallSize()const{
  return (2*sizeof(int) + /* num materials and shine */
	  numMaterials * 4 * sizeof(wrl::tuple3f) +
	  numShine * 2 * sizeof(float));
}

void wrl::materialData::marshall(void *data)const{
  int *num = (int *)data;
  num[0] = numMaterials;
  num[1] = numShine;

  /* ambientColor start right after the numbers */
  wrl::tuple3f *af = (wrl::tuple3f *)(&(num[2]));
  memcpy(af, ambientColor, numMaterials*sizeof(wrl::tuple3f));

  /* diffuseColor start right after the ambientColor */
  wrl::tuple3f *df = (wrl::tuple3f *)(&(af[numMaterials]));
  memcpy(df, diffuseColor, numMaterials*sizeof(wrl::tuple3f));

  /* emissiveColor start right after the diffuseColor */
  wrl::tuple3f *ef = (wrl::tuple3f *)(&(df[numMaterials]));
  memcpy(ef, emissiveColor, numMaterials*sizeof(wrl::tuple3f));

  /* specularColor start right after the emissiveColor */
  wrl::tuple3f *sf = (wrl::tuple3f *)(&(ef[numMaterials]));
  memcpy(sf, specularColor, numMaterials*sizeof(wrl::tuple3f));

  /* shininess start right after the specularColor */
  float *shinef = (float *)(&(sf[numMaterials]));
  memcpy(shinef, shininess, numShine*sizeof(float));

  /* transparency start right after the shininess */
  float *transf = (float *)(&(shinef[numShine]));
  memcpy(transf, transparency, numShine*sizeof(float));
}

void wrl::materialData::unmarshall(const void *data){
  const unsigned int *num = (const unsigned int *)data;
  numMaterials = num[0];
  numShine = num[1];

  ambientColor  = (wrl::tuple3f *)malloc(numMaterials*sizeof(wrl::tuple3f));
  diffuseColor  = (wrl::tuple3f *)malloc(numMaterials*sizeof(wrl::tuple3f));
  emissiveColor = (wrl::tuple3f *)malloc(numMaterials*sizeof(wrl::tuple3f));
  specularColor = (wrl::tuple3f *)malloc(numMaterials*sizeof(wrl::tuple3f));

  shininess = (float *)malloc(numShine*sizeof(float));
  transparency = (float *)malloc(numShine*sizeof(float));

  /* ambientColor start right after the numbers */
  const wrl::tuple3f *af = (wrl::tuple3f *)(&(num[2]));
  memcpy(ambientColor, af, numMaterials*sizeof(wrl::tuple3f));

  /* diffuseColor start right after the ambientColor */
  const wrl::tuple3f *df = (wrl::tuple3f *)(&(af[numMaterials]));
  memcpy(diffuseColor, df, numMaterials*sizeof(wrl::tuple3f));

  /* emissiveColor start rig () ht after the diffuseColor */
  const wrl::tuple3f *ef = (wrl::tuple3f *)(&(df[numMaterials]));
  memcpy(emissiveColor, ef, numMaterials*sizeof(wrl::tuple3f));

  /* specularColor start right after the emissiveColor */
  const wrl::tuple3f *sf = (wrl::tuple3f *)(&(ef[numMaterials]));
  memcpy(specularColor, sf, numMaterials*sizeof(wrl::tuple3f));

  /* shininess start right after the specularColor */
  const float *shinef = (float *)(&(sf[numMaterials]));
  memcpy(shininess, shinef, numShine*sizeof(float));

  /* transparency start right after the shininess */
  const float *transf = (float *)(&(shinef[numShine]));
  memcpy(transparency, transf, numShine*sizeof(float));
}

/*** end of wrl::materialData method definitions ***/

/*** start of wrl method definitions ***/

wrl::wrl(unsigned int ind)
  :dataIndex(ind),
   visible(true),
   numVertices(0),
   numFaces(0),
   vertices(NULL),
   faces(NULL),
   materialIndex(NULL),
   materials(NULL),
   neighbors(NULL){}

wrl::~wrl(){
  if(materials)delete materials;
  if(vertices)free(vertices);
  if(faces)free(faces);
  if(materialIndex)free(materialIndex);
}



int wrl::load(const char *file, const char *binName){
  assert(!materials && !vertices && !faces && !neighbors && !materialIndex);
  assert(file);

  if(binName){
    if(-1 != loadBin(file, binName))return 0;
    printf("Could not load cached version, loading original\n");
  }

  double startTime = curTime();

  FILE *f = fopen(file, "r+");
  if(!f){
    fprintf(stderr, "Unable to load %s\n", file);
    return -1;
  }
  materials = new materialData();

  char buf[1024];
  unsigned int lineNum=0;
  unsigned int vertexOffset=0;
  unsigned int materialOffset=0;
  unsigned int numSeparators=0;

  while(!feof(f)){
    lineNum++;
    if(!fgets(buf, sizeof(buf), f)      || 
       (strlen(buf) >= (sizeof(buf)-1)) ){

      if((numFaces>0) && (numVertices>0)){
	printf("Done reading %s:%d (took %0.3fs)\n",
	       file, lineNum, curTime() - startTime);

	printf("Components %03d ", numSeparators);
	printf("color %05d ", materials->numMaterials);
	printf("shine %05d ", materials->numShine);
	printf("vertices %06d ", numVertices);
	printf("faces %06d ", numFaces);
	printf("\n");

	break;
      }

      fprintf(stderr, "Error reading %s:%d\n", file, lineNum);
      return -1;
    }
    lineNum++;

    /* skip comments and empty lines */
    if(('\0' == buf[0]) || ('\n' == buf[0]) || 
       ('#'  == buf[0]) || ('}'  == buf[0]) )
      continue;

    /* skip the header */
    if(match(buf, "Separator ")){
      numSeparators++;
      /*
      printf("Components %03d ", numSeparators);
      printf("color %05d ", materials->numMaterials);
      printf("shine %05d ", materials->numShine);
      printf("vertices %06d ", numVertices);
      printf("faces %06d ", numFaces);
      printf("\n");
      */
      continue;
    }

    /* determine what we're reading */
    if(match(buf, "MaterialBinding ")){
      if(-1 == readMaterialBinding(f, lineNum)){
	printf("Error reading material binding %s:%d\n", file, lineNum);
	return -1;
      }

      //printf("readMaterialBinding success\n");

    }else if(match(buf, "Material ")){
      materialOffset = materials->numMaterials;

      if(-1 == materials->readFromFile(f, lineNum)){
	printf("Error reading material %s:%d\n", file, lineNum);
	return -1;
      }

      //printf("read materials success have %d (prev %d)\n", materials->numMaterials, materialOffset);

    }else if(match(buf, "Coordinate3 ")){
      unsigned int n=0;
      wrl::tuple3f *t=NULL;

      vertexOffset = numVertices;

      if(-1 == loadVertices(f, lineNum, n, &t)){
	printf("Error reading vertices %s:%d\n", file, lineNum);
	return -1;
      }

      //printf("read vertices success (got %d, appending to %d prev)\n", n, numVertices);

      vertices = append(numVertices, n, vertices, t);
      numVertices += n;

    }else if(match(buf, "IndexedFaceSet ")){
      unsigned int n=0;
      wrl::tuple3i *fa=NULL;
      int *mi=NULL;

      if(-1 == loadFaces(f, lineNum, n, &fa, &mi)){
	printf("Error reading faces %s:%d\n", file, lineNum);
	return -1;
      }

      //printf("read faces success (got %d, appending to %d prev)\n", n, numFaces);

      //fix offsets
      for(unsigned int i=0;i<n;i++){
	fa[i].x += vertexOffset;
	fa[i].y += vertexOffset;
	fa[i].z += vertexOffset;
	mi[i] += materialOffset;
      }

      faces = append(numFaces, n, faces, fa);
      materialIndex = append(numFaces, n, materialIndex, mi);
      numFaces+=n;

    }else{
      printf("Unknown input line %s\n", buf);
    }

  }

  fclose(f);
  printf("Finished reading %d separators\n", numSeparators);

  assert(!neighbors);
  neighbors = (tuple3i *)malloc(numFaces*sizeof(tuple3i));
  for(unsigned int i=0;i<numFaces;i++)
    neighbors[i].x = neighbors[i].y = neighbors[i].z = -2;

  if(binName){
    printf("Saving cached version...\n");
    if(-1 == saveBin(file, binName))
      printf("Error saving cached version\n");
    else
      printf("Created cached version\n");
  }

  return 0;
}



void wrl::saveAsSMF(const char *file)const{
  FILE *f = fopen(file, "w");
  if(!f){
    fprintf(stderr, "Error, unable to open %s\n", file);
    return;
  }

  printf("Writing %d vertices and %d faces to %s\n",
	 numVertices, numFaces, file);

  for(unsigned int i=0;i<numVertices;i++)
    fprintf(f, "v %0.5f %0.5f %0.5f\n",
	    vertices[i].x, vertices[i].y, vertices[i].z);

  for(unsigned int i=0;i<numFaces;i++)
    fprintf(f, "f %d %d %d\n",
	    faces[i].x+1, faces[i].y+1, faces[i].z+1);

  fclose(f);
}


#if DISABLE_GL
void wrl::draw()const{
  fprintf(stderr, "wrl::draw() error: gl has been disabled\n");
  abort();
}
#else

void wrl::draw()const{
  if(!visible)return;

  int lastColor = -1;

  glPushMatrix();

  glBegin(GL_TRIANGLES);
  for(unsigned int i=0;i<numFaces;i++){
    wrl::tuple3f p1 = vertices[faces[i].x];
    wrl::tuple3f p2 = vertices[faces[i].y];
    wrl::tuple3f p3 = vertices[faces[i].z];

    wrl::tuple3f a = minus(p2, p1);
    wrl::tuple3f b = minus(p3, p1);
    wrl::tuple3f c = normalize(cross(b, a));

    glNormal3f(c.x, c.y, c.z);

    int matInd = materialIndex[i];
    if((-1 == lastColor) || !materials->sameAs(matInd, lastColor)){
      materials->glColor(matInd);
      lastColor = matInd;
    }

    glVertex3f(p1.x, p1.y, p1.z);
    glVertex3f(p2.x, p2.y, p2.z);
    glVertex3f(p3.x, p3.y, p3.z);

  }
  glEnd();
  glPopMatrix();
}



unsigned int wrl::marshallSize()const{
  return (2*sizeof(int) +/* num vertices and faces */
	  numVertices*sizeof(wrl::tuple3f) + /* per vertex */
	  numFaces*(sizeof(wrl::tuple3i) + /* vertex indices */
		    sizeof(wrl::tuple3i) + /* neighbor indices */
		    sizeof(int)) + /* material index */
	  materials->marshallSize());
}

void wrl::marshall(void *data)const{
  int *num = (int *)data;
  num[0] = numVertices;
  num[1] = numFaces;

  /* materials start right after the numbers */
  char *ptr = (char *)(&(num[2]));
  materials->marshall(ptr);
  ptr += materials->marshallSize();

  /* vertices start right after the materials */
  wrl::tuple3f *vf = (wrl::tuple3f *)ptr;
  memcpy(vf, vertices, numVertices*sizeof(wrl::tuple3f));

  /* faces start right after the vertices */
  wrl::tuple3i *ff = (wrl::tuple3i *)(&(vf[numVertices]));
  memcpy(ff, faces, numFaces*sizeof(wrl::tuple3i));

  /* neighbors start right after the faces */
  wrl::tuple3i *nf = (wrl::tuple3i *)(&(ff[numFaces]));
  assert(neighbors);
  memcpy(nf, neighbors, numFaces*sizeof(wrl::tuple3i));

  /* materialIndex start right after the neighbors */
  int *mif = (int *)(&(nf[numFaces]));
  assert(materialIndex);
  memcpy(mif, materialIndex, numFaces*sizeof(int));
}

void wrl::unmarshall(const void *data){
  const int *num = (int *)data;
  numVertices = num[0];
  numFaces = num[1];

  materials = new materialData();
  vertices = (wrl::tuple3f *)malloc(numVertices*sizeof(wrl::tuple3f));
  faces = (wrl::tuple3i *)malloc(numFaces*sizeof(wrl::tuple3i));
  neighbors = (wrl::tuple3i *)malloc(numFaces*sizeof(wrl::tuple3i));
  materialIndex = (int *)malloc(numFaces*sizeof(int));

  /* materials start right after the numbers */
  const char *ptr = (char *)(&(num[2]));
  materials->unmarshall(ptr);
  ptr += materials->marshallSize();

  /* vertices start right after the materials */
  const wrl::tuple3f *vf = (wrl::tuple3f *)(ptr);
  memcpy(vertices, vf, numVertices*sizeof(wrl::tuple3f));

  /* faces start right after the vertices */
  wrl::tuple3i *ff = (wrl::tuple3i *)(&(vf[numVertices]));
  memcpy(faces, ff, numFaces*sizeof(wrl::tuple3i));

  /* neighbors start right after the faces */
  wrl::tuple3i *nf = (wrl::tuple3i *)(&(ff[numFaces]));
  memcpy(neighbors, nf, numFaces*sizeof(wrl::tuple3i));

  /* materialIndex start right after the neighbors */
  int *mif = (int *)(&(nf[numFaces]));
  memcpy(materialIndex, mif, numFaces*sizeof(int));
}


int wrl::loadBin(const char *file, const char *binName){
  assert(file && binName);

  /* see if the file exists and how big it is */
  struct stat buf;
  if(-1 == stat(binName, &buf))return -1;

  /* make sure we can read it */
  int fd = open(binName,O_RDONLY);
  if(-1 == fd)return -1;

  /* mmap the file */
  void *map = mmap(NULL, buf.st_size, PROT_READ, MAP_SHARED, fd, 0);
  if(MAP_FAILED == map){close(fd);return -1;}

  unmarshall(map);

  printf("Read cached version of %s (%s) (%d points/%d triangles)\n",
	 file, binName, numVertices, numFaces);

  /* success, clean up */
  munmap(map, buf.st_size);
  close(fd);
  return 0;
}


int wrl::saveBin(const char *file, const char *binName)const{
  assert(file && binName);

  /* make sure we can write it */
  int fd = open(binName,O_RDWR|O_CREAT, S_IRWXU);
  if(-1 == fd){
    perror("Error opening file");
    return -1;
  }

  /* compute the file size */
  size_t size = marshallSize();

  if(-1 == lseek(fd, size-1, SEEK_SET)){
    perror("lseek(size) failed");
    return -1;
  }

  char tmp = '\0';
  if(1 != write(fd, &tmp, 1)){
    perror("write failed");
    return -1;
  }

  if(-1 == lseek(fd, 0, SEEK_SET)){
    perror("lseek(0) failed");
    return -1;
  }

  printf("Creating %s of %lu bytes\n", binName, size);

  /* mmap the file */
  void *map = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  if(MAP_FAILED == map){close(fd);return -1;}

  marshall(map);

  printf("Created cached version of %s (%s)\n", file, binName);

  /* success, clean up */
  munmap(map, size);
  close(fd);
  return 0;
}

void wrl::applyTransform(const float rpy[3][3], float x, float y, float z){
  for(unsigned int i=0;i<numVertices;i++)
    ::applyTransform(rpy, x, y, z,
		     vertices[i].x, vertices[i].y, vertices[i].z);
}

/*** end of wrl method definitions ***/

/*** start of definition of various helper functions ***/

static bool match(const char *a, const char *b){
  return !strncmp(a, b, strlen(b));
}

static double curTime(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (1e-6)*tv.tv_usec;
}



static int readMaterialBinding(FILE *f, unsigned int &lineNum){
  char buf[1024];
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;

  if(!match(buf, "value PER_FACE_INDEX")){
    fprintf(stderr, "expected PER_FACE_INDEX, got %s", buf);
    return -1;
  }

  do{
    if(!fgets(buf, sizeof(buf), f))return -1;
    lineNum++;
  }while('\n' == buf[0]);

  if(!match(buf, "}")){
    fprintf(stderr, "expected \'}\', got %s", buf);
    return -1;
  }

  return 0;
}

/* concatenates the two arrays and frees the originals */
static wrl::tuple3f *append(unsigned int na, unsigned int nb,
			    wrl::tuple3f *a, wrl::tuple3f *b){
  assert((na+nb)>0);
  wrl::tuple3f *ret= (wrl::tuple3f *)malloc((na+nb)*sizeof(wrl::tuple3f));
  assert(ret);

  if(a){
    memcpy(ret, a, na*sizeof(wrl::tuple3f));
    free(a);
  }

  if(b){
    memcpy(&(ret[na]), b, nb*sizeof(wrl::tuple3f));
    free(b);
  }

  return ret;
}

static wrl::tuple3i *append(unsigned int na, unsigned int nb,
			    wrl::tuple3i *a, wrl::tuple3i *b){
  assert((na+nb)>0);
  wrl::tuple3i *ret= (wrl::tuple3i *)malloc((na+nb)*sizeof(wrl::tuple3i));
  assert(ret);

  if(a){
    memcpy(ret, a, na*sizeof(wrl::tuple3i));
    free(a);
  }

  if(b){
    memcpy(&(ret[na]), b, nb*sizeof(wrl::tuple3i));
    free(b);
  }

  return ret;
}

static float *append(unsigned int na, unsigned int nb, float *a, float *b){
  assert((na+nb)>0);
  float *ret= (float *)malloc((na+nb)*sizeof(float));
  assert(ret);

  if(a){
    memcpy(ret, a, na*sizeof(float));
    free(a);
  }

  if(b){
    memcpy(&(ret[na]), b, nb*sizeof(float));
    free(b);
  }

  return ret;
}

static int *append(unsigned int na, unsigned int nb, int *a, int *b){
  assert((na+nb)>0);
  int *ret= (int *)malloc((na+nb)*sizeof(int));
  assert(ret);

  if(a){
    memcpy(ret, a, na*sizeof(int));
    free(a);
  }

  if(b){
    memcpy(&(ret[na]), b, nb*sizeof(int));
    free(b);
  }

  return ret;
}


static int loadVertices(FILE *f, unsigned int &lineNum,
			unsigned int &n, wrl::tuple3f **vertices){
  char buf[1024];

  //verbose=1;

  do{
    if(!fgets(buf, sizeof(buf), f))return -1;
    lineNum++;
  }while('\n' == buf[0]);

  if(!match(buf, "point ")){
    fprintf(stderr, "expected point, got %s", buf);
    return -1;
  }

  //read points
  if(-1 == readTuple3f(f, lineNum, n, vertices)){
    fprintf(stderr, "Error reading tuple for points\n");
    return -1;
  }

  verbose=0;

  //printf("Read %d points\n", *n);

  // done reading data
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "}")){
    fprintf(stderr, "expected \'}\', got %s", buf);
    return -1;
  }

  //these files have z forward, y up, x left
  //we want x right, y forward, z up
  for(unsigned int i=0; i<n; i++){
    float x = (*vertices)[i].x;
    float y = (*vertices)[i].y;
    float z = (*vertices)[i].z;
    (*vertices)[i].x = -x;
    (*vertices)[i].y =  z;
    (*vertices)[i].z =  y;
  }

  return 0;
}

/* returns -1 on error */
static int loadFaces(FILE *f, unsigned int &lineNum, unsigned int &n,
		     wrl::tuple3i **faces, int **materials){
  char buf[1024];


  do{
    if(!fgets(buf, sizeof(buf), f))return -1;
    lineNum++;
  }while('\n' == buf[0]);

  if(!match(buf, "coordIndex ")){
    fprintf(stderr, "expected coordIndex, got %s", buf);
    return -1;
  }

  //read faces
  if(-1 == readTuple3i4(f, lineNum, n, faces))
    return -1;

  //printf("Read %d faces\n", *n);

  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "materialIndex ")){
    fprintf(stderr, "expected point, got %s", buf);
    return -1;
  }

  //read materials
  unsigned int n2=0;
  if(-1 == readTuple1i(f, lineNum, n2, materials))
    return -1;
  //printf("Read %d material indices\n", n2);

  if(n2 != n){
    fprintf(stderr, "Error, read %d faces but %d materials\n", n, n2);
    return -1;
  }

  // done reading data
  if(!fgets(buf, sizeof(buf), f))return -1;
  lineNum++;
  if(!match(buf, "}")){
    fprintf(stderr, "expected \'}\', got %s", buf);
    return -1;
  }

  return 0;
}


/************ helper functions to read an array of numbers ******************/

/* reads a single value token
 * the value starts with whitespace, 
 * and is terminated by whitespace or a comma
 *
 * returns 1 if a ']' is reached
 * (in this case following whitespace is also consumed)
 * returns 0 on success and fills in the value
 * returns -1 on error (EOF, large value, bad data, etc)
 */
static int readToken(FILE *f, unsigned int &lineNum,
		     char *buf, unsigned int buflen){
  char c;
  buf[0]='\0';

  /* advance the file stream to a number */
  while(1){
    c = getc(f);
    if('\n' == c)lineNum++;
    if(feof(f) || EOF == c)return -1;
    if(isspace(c))continue;
    if(']' == c){
      while(1){
	c = getc(f);
	if(feof(f) || EOF == c)return -1;
	if(isspace(c))continue;
	ungetc(c, f);
	return 1;
      }
    }

    //make sure this is some sort of input
    if('-' == c || isdigit(c) || '.' == c){
      ungetc(c, f);
      break;
    }

    //unexpected character
    fprintf(stderr, "Before reading a number, got unexpected char %c\n", c);
    return -1;
  }

  /* now read a number */
  for(unsigned int i=0;i<buflen;i++){
    if(i==(buflen-1)){
      fprintf(stderr, "Error, large number (over %d chars)\n", buflen);
      return -1;
    }

    c = getc(f);
    if('\n' == c)lineNum++;
    if(feof(f) || EOF == c)return -1;

    //make sure this is some sort of input
    if(('-' == c && 0 == i) || isdigit(c) || '.' == c){
      buf[i] = c;
      continue;
    }

    //end of number
    buf[i] = '\0';

    //check the termination cause
    if(isspace(c) || ',' == c)break;

    if(']' == c){
      //input terminated by brace... defer processing the brace
      ungetc(c, f);
      break;
    }

    //unexpected character
    fprintf(stderr, "While reading a number, got unexpected char %c\n", c);
    return -1;
  }//done reading the number


  return 0;
}

/*** helper functions to read 1 float or int ***/
static int readF(FILE *f, unsigned int &lineNum, float *val){
  char buf[16];
  
  int ret = readToken(f, lineNum, buf, sizeof(buf));
  if(ret != 0)return ret;

  if(verbose)
    printf("readF(): line %d, token \"%s\"\n", lineNum, buf);
  //abort();

  //make sure the data is sensible
  if(!strcmp(buf, "-") || '\0' == buf[0]){
    fprintf(stderr, "Error, read no number\n");
    return -1;
  }

  //success
  *val = atof(buf);

  return 0;
}

static int readI(FILE *f, unsigned int &lineNum, int *val){
  char buf[16];
  
  int ret = readToken(f, lineNum, buf, sizeof(buf));
  if(ret != 0)return ret;

  //make sure the data is sensible
  if(!strcmp(buf, "-") || '\0' == buf[0]){
    fprintf(stderr, "Error, read no number\n");
    return -1;
  }

  if(index(buf, '.')){
    fprintf(stderr, "Error, read a float, expected an int\n");
    return -1;
  }

  //success
  *val = atoi(buf);

  return 0;
}

/*** helper functions to read a 3d float or int array ***/
static int readTuple3f(FILE *f, unsigned int &lineNum,
		       unsigned int &n, wrl::tuple3f **arr){
  assert(arr);
  assert(!(*arr));

  //printf("readTuple3f start at line %d\n", lineNum);

  /* read as 1d */
  float *arr2=NULL;
  if(-1 == readTuple1f(f, lineNum, n, &arr2)){
    if(arr2)free(arr2);
    return -1;
  }

  /* make sure there is a multiple of 3 */
  if(0 != n%3){
    if(arr2)free(arr2);
    fprintf(stderr, "Error reading 3 float tuple, got %d items\n", n);
    return -1;
  }

  /* convert from 1d to 3d array */
  int n2 = n;
  n = n2/3;
  *arr = (wrl::tuple3f *)malloc(n * (sizeof(wrl::tuple3f)));
  assert(*arr);
  for(unsigned int i=0; i<n; i++){
    (*arr)[i].x = arr2[i*3 + 0];
    (*arr)[i].y = arr2[i*3 + 1];
    (*arr)[i].z = arr2[i*3 + 2];
  }
  if(arr2)free(arr2);
  return 0;
}


static int readTuple3i4(FILE *f, unsigned int &lineNum,
			unsigned int &n, wrl::tuple3i **arr){
  assert(arr);
  assert(!(*arr));

  /* read as 1d */
  int *arr2=NULL;
  if(-1 == readTuple1i(f, lineNum, n, &arr2)){
    if(arr2)free(arr2);
    return -1;
  }

  /* make sure there is a multiple of 4 */
  if(0 != n%4){
    if(arr2)free(arr2);
    fprintf(stderr, "Error reading 3/4 int tuple, got %d items\n", n);
    return -1;
  }

  /* convert from 1d to 3d array */
  unsigned int n2 = n;
  n = n2/4;
  *arr = (wrl::tuple3i *)malloc(n * (sizeof(wrl::tuple3i)));
  for(unsigned int i=0; i<n; i++){
    (*arr)[i].x = arr2[i*4 + 0];
    (*arr)[i].y = arr2[i*4 + 1];
    (*arr)[i].z = arr2[i*4 + 2];
  }
  if(arr2)free(arr2);
  return 0;
}

/*** helper functions to load a 1d float or int array ***/
static int readTuple1f(FILE *f, unsigned int &lineNum,
		       unsigned int &n, float **arr){
  assert(arr);
  assert(!(*arr));

  n = 0;
  unsigned int num = 0;
  float *buf = NULL;

  //printf("readTuple1f start at line %d\n", lineNum);

  while(1){
    float val=0.0;

    switch(readF(f, lineNum, &val)){
    case -1:
      printf("Error reading value %d\n", n);
      return -1;
    case 1://reached end
      *arr = buf;
      n = num;
      return 0;
    }

    if(verbose)
      printf("Value %d = %0.5f\n", num, val);

    /* read the value, store it */
    num++;
    buf = (float *)realloc(buf, num*sizeof(float));
    assert(buf);
    buf[num-1] = val;
  }

  //never reaches here
  assert(0);
  return -1;
}

static int readTuple1i(FILE *f, unsigned int &lineNum,
		       unsigned int &n, int **arr){
  assert(arr);
  assert(!(*arr));

  n = 0;

  while(1){
    int val=0;

    switch(readI(f, lineNum, &val)){
    case -1:
      printf("Error reading value %d\n", n);
      return -1;
    case 1://reached end
      return 0;
    }

    /* read the value, store it */
    n++;
    *arr = (int *)realloc(*arr, n*sizeof(int));
    (*arr)[n-1] = val;
  }

  //never reaches here
  return -1;
}

static void applyTransform(const float rpy[3][3], 
			   float dx, float dy, float dz,
			   float &x, float &y, float &z){
  float x2 = x;
  float y2 = y;
  float z2 = z;

  x = rpy[0][0]*x2 + rpy[0][1]*y2 + rpy[0][2]*z2 + dx;
  y = rpy[1][0]*x2 + rpy[1][1]*y2 + rpy[1][2]*z2 + dy;
  z = rpy[2][0]*x2 + rpy[2][1]*y2 + rpy[2][2]*z2 + dz;
}


/*** end of definition of various helper functions ***/

#endif
