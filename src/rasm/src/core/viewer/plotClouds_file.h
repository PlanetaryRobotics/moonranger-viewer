/*
 * Contains all the file i/o code
 * Handles reloading data if stat() indicates it was modified
 *
 * Dominic Jonak
 */
#ifndef PLOT_CLOUDS_FILE_H
#define PLOT_CLOUDS_FILE_H

#include "helpers.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

class loadedFile{
 private:
  //hidden default constructor
  loadedFile(){}

 protected:
  const char *name;
  time_t lastModified;
  time_t getModified() const;

 public:
  void updateModified(){
    lastModified = getModified();
  }
  bool needUpdate()const{
    /*
    printf("%s %ld>%ld && %ld>%ld\n", name,
	   getModified(), lastModified, time(NULL), getModified());
    */
    return ((getModified()>lastModified) && (time(NULL)>getModified()));
  }
  const char *getName()const{
    return name;
  }
  loadedFile(const char *f);
};

bool fileExists(const char *filename);
int readWRL(points *&p, triangles *&t, const char *file);
void loadFile(const char *file, int reload=0);
void reloadFiles();
bool needReload();

/* handles a keypress, calls glutPostRedisplay() and returns true
 * returns false if the key is unrelated
 */
bool key_files(unsigned char key,int x, int y);

#endif
