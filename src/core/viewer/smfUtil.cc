#include "smfUtil.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>

int writeSMF(const points *p, const std::vector<faceIndices> &faceList, FILE *f){
  if(!f)return -1;
  fprintf(f, "begin\n");
  for(unsigned int i=0;i<p->size();i++)
    fprintf(f, "v %g %g %g\n", p->get(i).x, p->get(i).y, p->get(i).z);
  for(unsigned int i=0;i<faceList.size();i++)
    fprintf(f, "f %d %d %d\n", 1+faceList[i][0],
	    1+faceList[i][1], 1+faceList[i][2]);
  fprintf(f, "end\n");
  return 0;
}

int writeSMF(const points *p, const std::vector<faceIndices> &faceList, 
	     const char *file){
  FILE *f = fopen(file, "w");
  int ret = writeSMF(p, faceList, f);
  if(f)fclose(f);
  return ret;
}

int writeSMF(const points *p, const int *faceList, int numFaces, FILE *f){
  if(!f)return -1;
  fprintf(f, "begin\n");
  for(unsigned int i=0;i<p->size();i++)
    fprintf(f, "v %g %g %g\n", p->get(i).x, p->get(i).y, p->get(i).z);
  for(int i=0;i<numFaces;i++)
    fprintf(f, "f %d %d %d\n", 1+faceList[i * 3 + 0],
	    1+faceList[i * 3 + 1], 1+faceList[i * 3 + 2]);
  fprintf(f, "end\n"); 
  return 0;
}

int writeSMF(const points *p, const int *faceList, int numFaces,
	     const char *file){
  FILE *f = fopen(file, "w");
  int ret = writeSMF(p, faceList, numFaces, f);
  if(f)fclose(f);
  return ret;
}


int readSMF(points *&p, std::vector<faceIndices> &faceList, FILE *f){
  p = new points();

  METERS x, y, z;
  char c;
  char linebuf[64];
  while(!feof(f)){
    //printf("Reading line\n");
    if(!fgets(linebuf, sizeof(linebuf), f))continue;
    //printf("Got line \"%s\"\n", linebuf);
    if(4 != sscanf(linebuf, "%c %lg %lg %lg", &c, &x, &y, &z))continue;

    if('v' == c){
      p->add(point(x, y, z));
      //printf("Added point %d\n", p->size());

    }else if('f' == c){
      if(x<1 || y<1 || z<1 || x>p->size() || y>p->size() || z>p->size()){
	printf("warning: ignoring invalid triangle indices %g, %g, %g (should be 1 to %d)\n", 
	       x,y,z, p->size());
	//fclose(f);
	//return -1;
      }else{
	int addme[3]={(int)x-1, (int)y-1, (int)z-1};
	faceList.push_back(faceIndices(addme));
	//printf("Added triangle %d\n", faceList->size());
      }
    }
  }
  return 0;
}

int readSMF(points *&p, int *faceList, int &numFaces, FILE *f){
  faceList = NULL;
  numFaces=0;

  std::vector<faceIndices> vec;
  if(-1 == readSMF(p, vec, f))return -1;

  /* change the face list to an int list */
  faceList = (int *)new int[vec.size()*3];
  if(!faceList)return -1;

  numFaces = vec.size();
  for(int i=0;i<numFaces;i++)
    for(int j=0;j<3;j++)
      faceList[3*i+j] = vec[i][j];

  return 0;
}

int readSMF(points *&p, triangles *&t, FILE *f){
  std::vector<faceIndices> vec;
  if(-1 == readSMF(p, vec, f))return -1;

  /* change the face list to a triangle list */
  t = new triangles();
  for(unsigned int i=0;i<vec.size();i++){
    t->add(triangle(p->get(vec[i].a), p->get(vec[i].b), p->get(vec[i].c)));
    t->getRef(i).setInd(vec[i].a, vec[i].b, vec[i].c);
  }

  return 0;
}

static void marshall(void *data, const points *p, const triangles *t){
  int *num = (int *)data;
  num[0] = p->size();
  num[1] = t->size();

  /* points start right after the numbers */
  point *pf = (point *)(&(num[2]));

  /* write the points */
  for(unsigned int i=0;i<p->size();i++)
    pf[i] = p->get(i);

  /* triangles start right after the points */
  triangle *tf = (triangle *)(&(pf[p->size()]));

  /* write the triangles */
  for(unsigned int i=0;i<t->size();i++)
    tf[i] = t->get(i);
}

static void unmarshall(const void *data, points *&p, triangles *&t){
  const int *num = (int *)data;
  int numPoints = num[0];
  int numTriangles = num[1];

  /* points start right after the numbers */
  const point *pf = (point *)(&(num[2]));

  /* read the points */
  p = new points();
  for(int i=0;i<numPoints;i++)
    p->add(pf[i]);

  /* triangles start right after the points */
  const triangle *tf = (triangle *)(&(pf[numPoints]));

  /* read the triangles */
  t = new triangles();
  for(int i=0;i<numTriangles;i++)
    t->add(tf[i]);
}

int readSMFbin(points *&p, triangles *&t, const char *file){
  char *binName = toBinName(file);

  /* see if the file exists and how big it is */
  struct stat buf;
  if(-1 == stat(binName, &buf)){free(binName);return -1;}

  /* check the modification time, the file may have been updated */
  struct stat origbuf;
  if(-1 == stat(file, &origbuf)){
    perror("Error, Unable to stat file");
    free(binName);
    return -1;
  }
  if(origbuf.st_mtime > buf.st_mtime){
    printf("Warning, cached file is stale\n");
    free(binName);
    return -1;
  }

  /* make sure we can read it */
  int fd = open(binName,O_RDONLY);
  if(-1 == fd){free(binName);return -1;}

  /* mmap the file */
  void *map = mmap(NULL, buf.st_size, PROT_READ, MAP_SHARED, fd, 0);
  if(MAP_FAILED == map){free(binName);close(fd);return -1;}

  unmarshall(map, p, t);

  printf("Read cached version of %s (%s) (%d points/%d triangles)\n",
	 file, binName, p->size(), t->size());

  /* success, clean up */
  free(binName);
  munmap(map, buf.st_size);
  close(fd);
  return 0;
}

int writeSMFbin(const points *p, const triangles *t, const char *file){
  char *binName = toBinName(file);

  /* make sure we can write it */
  int fd = open(binName,O_RDWR|O_CREAT, S_IRWXU);
  if(-1 == fd){
    perror("Error openning file");
    free(binName);
    return -1;
  }

  /* compute the file size */
  size_t size = 2*sizeof(int) + p->size()*sizeof(point) + t->size()*sizeof(triangle);

  if(-1 == lseek(fd, size-1, SEEK_SET)){
    perror("lseek(size) failed");
    free(binName);
    return -1;
  }

  char tmp = '\0';
  if(1 != write(fd, &tmp, 1)){
    perror("write failed");
    free(binName);
    close(fd);
    return -1;
  }

  if(-1 == lseek(fd, 0, SEEK_SET)){
    perror("lseek(0) failed");
    free(binName);
    close(fd);
    return -1;
  }


  /* mmap the file */
  void *map = mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  if(MAP_FAILED == map){free(binName);close(fd);return -1;}

  marshall(map, p, t);

  printf("Created cached version of %s (%s)\n", file, binName);

  /* success, clean up */
  free(binName);
  munmap(map, size);
  close(fd);
  return 0;
}

int readSMF(points *&p, triangles *&t, const char *file){
  /* try the cache first */
  if(0 == readSMFbin(p, t, file))return 0;

  FILE *f = fopen(file, "r");
  if(!f)return -1;

  /* read all the vertices and faces */
  if(-1 == readSMF(p, t, f))return -1;
  fclose(f);

  printf("Read %d points and %d triangles\n", p->size(), t->size());

  /* write the cache */
  if(-1 == writeSMFbin(p, t, file))
    printf("Warning, unable to create cached version\n");

  return 0;
}
