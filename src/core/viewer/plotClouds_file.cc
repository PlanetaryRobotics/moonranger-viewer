#include "plotClouds_file.h"
#include "plotClouds_globals.h"
#include "plotClouds_edge.h"
#include "plotClouds_triangulate.h"

#include "smfUtil.h"
#include "wrl.h"

#include <string.h>

time_t loadedFile::getModified() const{
  struct stat buf;
  if(-1 == stat(name, &buf))return 0;
  return buf.st_mtime;
}

loadedFile::loadedFile(const char *f){
  name=f;
  updateModified();
}

bool fileExists(const char *filename){
  struct stat buf;
  return (-1 != stat(filename, &buf));
}

/* helper function to convert between wrl::tuple3f and point/triangle */
static point tuple3fToPoint(const wrl::tuple3f &p){
  return point(p.x, p.y, p.z);
}
static triangle tuple3fsToTriangle(const wrl::tuple3f &a,
				   const wrl::tuple3f &b,
				   const wrl::tuple3f &c){
  return triangle( tuple3fToPoint(a), tuple3fToPoint(b), tuple3fToPoint(c) );
}

int readWRL(points *&p, triangles *&t, const char *file){
  /* load the wrl file */
  wrl *w = new wrl(clouds.size());
  char *binName = toBinName(file);
  if(-1 == w->load(file, binName)){
    free(binName);
    free(w);
    return -1;
  }
  free(binName);
  wrls.push_back(w);

  /* use the wrl model to create a regular mesh */
  p = new points();
  for(unsigned int i=0;i<w->getNumVertices();i++)
    p->add(tuple3fToPoint(w->getVertex(i)));

  t = new triangles();
  for(unsigned int i=0;i<w->getNumFaces();i++)
    t->add(tuple3fsToTriangle(w->getVertex( w->getFace(i).x ),
			      w->getVertex( w->getFace(i).y ),
			      w->getVertex( w->getFace(i).z )));

  return 0;
}

void loadFile(const char *file, int reload){
  if(!reload)fileList.push_back(new loadedFile(file));

  points *p=NULL;
  triangles *t=NULL;

  printf("Loading \"%s\" ", file);

  if(!strcasecmp(file, "live_stereo") ||
     !strcasecmp(file, "live_laser")  ){
    fprintf(stderr, "Error, refusing to load virtual file %s\n", file);
    abort();
  }

  if(strstr(file, ".smf")){
    printf("Reading SMF file\n");
    if(-1 == readSMF(p, t, file)){
      printf("Error reading SMF file \"%s\"\n", file);
      return;
    }

    /* use the smf model to create a shadow model */
    if(useShadows){
      shadowModel *s = new shadowModel(p->size(), t->size());
      for(unsigned int i=0;i<p->size();i++){
	s->v[i].x = p->get(i).x;
	s->v[i].y = p->get(i).y;
	s->v[i].z = p->get(i).z;
      }

      points *dummy=NULL;
      std::vector<faceIndices> faceList;
      FILE *f = fopen(file, "r");
      readSMF(dummy, faceList, f);
      fclose(f);
      delete dummy;

      for(unsigned int i=0;i<faceList.size();i++){
	s->f[i].points[0] = faceList[i].a;
	s->f[i].points[1] = faceList[i].b;
	s->f[i].points[2] = faceList[i].c;
	s->f[i].neighbors[0]=-1;
	s->f[i].neighbors[1]=-1;
	s->f[i].neighbors[2]=-1;
      }
      s->computeNormals(0);
      s->computeNeighbors();
      shadows.push_back(s);
    }

  }else if(strstr(file, ".wrl")){
    printf("Reading WRL file\n");
    if(-1 == readWRL(p, t, file)){
      printf("Error reading WRL file \"%s\"\n", file);
      return;
    }
  }else if(strstr(file, ".paths")){
    printf("Reading paths file\n");
    if(-1 == readPaths(file))
      printf("Error reading paths file \"%s\"\n", file);
    //printf("Success\n");
    return;
  }else{
    printf("Reading points and generating mesh\n");
    p = new points(file);
    printf("Got %d points\n", p->size());
    t = createTriangles(p);
  }

  if(minAngle>0){
    triangles *good = pruneVertical(t, minAngle);
    delete t;
    t = good;
  }

  if(maxDeviationsLong>0){
    triangles *good = pruneLongEdges(t, maxDeviationsLong);
    delete t;
    t = good;
  }

  if(maxDeviationsShort>0){
    triangles *good = pruneShortEdges(t, maxDeviationsShort);
    delete t;
    t = good;
  }

  clouds.push_back(p);
  triangleMeshes.push_back(t);
  if(!reload)isTracks.push_back(false);
}

void reloadFiles(){
  /* delete old data */
  freedata();

  /* reload all files */
  for(unsigned int i=0;i<fileList.size();i++){
    loadFile(fileList[i]->getName(), 1);
    fileList[i]->updateModified();
  }
}

bool needReload(){
  for(unsigned int i=0;i<fileList.size();i++){
    if(fileList[i]->needUpdate()){
      printf("File %d of %lu (%s) was modified\n", 
	     i, fileList.size(), fileList[i]->getName());
      return true;
    }
  }
  return false;
}

bool key_files(unsigned char key,int x, int y){
  switch(key){
  case 'r':
    printf("Reloading all files\n");
    reloadFiles();
    break;

  default:
    return false;
  }

  glutPostRedisplay();
  return true;
}
