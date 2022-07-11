#ifndef SMF_UTIL_H
#define SMF_UTIL_H

#include "helpers.h"
#include <stdio.h>
#include <vector>

struct faceIndices{
  int a, b, c;
  faceIndices(int arr[3]):a(arr[0]), b(arr[1]), c(arr[2]){}
  faceIndices(int A, int B, int C):a(A), b(B), c(C){}
  int &operator[](int i)const{return ((int *)this)[i];}
};

int readSMF(points *&p, std::vector<faceIndices> &faceList, FILE *f);
int readSMF(points *&p, int *faceList, int &numFaces, FILE *f);
int readSMF(points *&p, triangles *&t, FILE *f);
int readSMF(points *&p, triangles *&t, const char *file);

int writeSMF(const points *p, const std::vector<faceIndices> &faceList, 
	     FILE *f);
int writeSMF(const points *p, const int *faceList, int numFaces,
	     FILE *f);
int writeSMF(const points *p, const std::vector<faceIndices> &faceList, 
	     const char *file);
int writeSMF(const points *p, const int *faceList, int numFaces,
	     const char *file);

#endif
