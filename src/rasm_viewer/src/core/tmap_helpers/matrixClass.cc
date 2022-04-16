#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <rasm_common_types.h>
#include <matrixClass.h>

matrix::matrix(unsigned int rows, unsigned int cols)
  :numRows(rows), numCols(cols),
   data(new double[numRows*numCols]),
   doChecks(1){
  for(unsigned int i=0;i<rows*cols;i++)
    data[i] = 0.0;
}

matrix::matrix(const matrix &mat):numRows(0), numCols(0), data(NULL), doChecks(1){
  copy(mat);
}
matrix::matrix(const matrix *mat):numRows(0), numCols(0), data(NULL), doChecks(1){
  copy(mat);
}

void matrix::copy(const matrix &copy){
  numRows = copy.numRows;
  numCols = copy.numCols;
  if(data)delete []data;
  data = new double[numRows*numCols];
  memcpy(data, copy.data, numRows*numCols * sizeof(double));
}

void matrix::copy(const matrix *copy){
  numRows = copy->numRows;
  numCols = copy->numCols;
  memcpy(data, copy->data, numRows*numCols * sizeof(double));
}

matrix::~matrix(){
  if(data)delete []data;
  data = NULL;
}

unsigned int matrix::getIndex(unsigned int row, unsigned int col) const{
  if(doChecks){
    consistencyCheck();
    if(row >= numRows){
      printf("matrix error, tried to get row %d, max rows %d\n", row, numRows);
      assert(0);
    }
    if(col >= numCols){
      printf("matrix error, tried to get col %d, max cols %d\n", col, numCols);
      assert(0);
    }
  }
  return (row*numCols + col);
}

double *matrix::get(unsigned int row) const{
  return &(data[getIndex(row, 0)]);
}
double &matrix::get(unsigned int row, unsigned int col) const{
  return data[getIndex(row, col)];
}
unsigned int matrix::rows() const{
  return numRows;
}
unsigned int matrix::cols() const{
  return numCols;
}

void matrix::printRow(unsigned int row, const char *name) const{
  double *rowdata = get(row);
  if(name && *name)printf("%s=", name);
  for(unsigned int c=0;c<numCols;c++)
    printf("%5.5f ", rowdata[c]);
  printf("\n");
}

void matrix::print(const char *name) const{
  for(unsigned int r=0;r<numRows;r++)
    printRow(r, name);
}

void matrix::consistencyCheck() const{
  for(unsigned int i=0;i<(numRows*numCols);i++){
    if(!std::isfinite(data[i])){
      printf("Consistency check failed!\n");

      for(unsigned int r=0;r<numRows;r++){
	for(unsigned int c=0;c<numCols;c++){
	  printf("%g ", data[r*numCols + c]);
	}
	printf("\n");
      }

      assert(0);
    }
  }
}

void static_svd(const matrix &Morig, matrix &U, matrix &S, matrix &V);
void matrix::svd(matrix &U, matrix &S, matrix &V) const{
  static_svd(*this, U, S, V);
}

matrix matrix::svdRotation() const{
  matrix U(numRows, numRows);
  matrix S(numRows, numCols);
  matrix V(numCols, numCols);
  svd(U, S, V);
  return V * (U.transpose());
}


#ifndef HELPER_DEFINES
#define HELPER_DEFINES
#define min(a, b) (((a)<(b))?(a):(b))
#define max(a, b) (((a)<(b))?(b):(a))
#define pythag(a, b) (sqrt((a)*(a) + (b)*(b)))
#define SIGN(a, b) ((b) >= 0. ? fabs(a) : -fabs(a))
#endif

/* compute A = B * C */
void matrixMult(matrix &A, const matrix &B, const matrix &C){
  if(B.cols() != C.rows()){
    printf("Can't multiply %dx%d matrix by %dx%d matrix\n",
	   B.rows(), B.cols(), C.rows(), C.cols());
    abort();
  }
  assert(A.rows() == B.rows());
  assert(A.cols() == C.cols());
  for(unsigned int r=0;r<A.rows();r++){
    for(unsigned int c=0;c<A.cols();c++){
      A[r][c] = 0.0;
      for(unsigned int i=0;i<B.cols();i++)
	A[r][c]+=B[r][i] * C[i][c];
    }
  }

}

/* matrix Mat is numRows by numCols
 * remove row skipRow and column skipCol
 * row skipRow+1 will become row 0
 * column skipCol+1 will become column 0
 * the result is placed in M, which is allocated for (numRows-1) by (numCols-1)
 */
static void cropMatrix(unsigned int numRows, unsigned int numCols,
		   unsigned int skipRow, unsigned int skipCol, 
		   const double *Mat, double *M){
#if 0
  for(unsigned int r=0;r<numRows-1;r++){
    for(unsigned int c=0;c<numCols-1;c++){
      unsigned int sourceR = (r+skipRow+1)%numRows;
      unsigned int sourceC = (c+skipCol+1)%numCols;
      M[ r * (numCols-1) + c ] = Mat[sourceR * numCols + sourceC];
    }
  }
#else
  unsigned int index=0;
  for(unsigned int r=0;r<numRows;r++){
    if(r==skipRow)continue;
    for(unsigned int c=0;c<numCols;c++){
      if(c==skipCol)continue;
      M[ index++ ] = Mat[r * numCols + c];
    }
  }
#endif
}

static double det(unsigned int size, const double *Mat){
  if(1 == size)return Mat[0];
  if(size<1){
    printf("Error, can't get determinant of size %d matrix\n", size);
    abort();
  }

  double ret = 0;
  double *M = (double *)alloca((size-1)*(size-1)*sizeof(double));

  int skipR = -1;
  int skipC = -1;

  //TODO optimize and pick a particular row/column
  //skipR = 0;
  skipC = 1;

  for(unsigned int i=0;i<size;i++){
    unsigned int cropR = (skipR<0)?i:(unsigned int)skipR;
    unsigned int cropC = (skipC<0)?i:(unsigned int)skipC;

    /* copy the matrix, but not row skipR or column skipC */
    cropMatrix(size, size, cropR, cropC, Mat, M);

    /* get the determinant of the sub matrix */
    double d = ::det(size-1, M);
    double add = ((i%2)?d:-d) * Mat[cropR * size + cropC];
    ret+=add;
  }

  return ret;
}

double matrix::det()const{
  if(cols() != rows()){
    printf("Can't get determinant of non-square %dx%d matrix\n",
	   rows(), cols());
    abort();
  }

  /* 1 by 1 matrix */
  if(cols() == 1)return data[0];

  if(cols() < 2){
    printf("Can't get determinant of degenerate %d square matrix\n",
	   rows());
  }

  return ::det(cols(), data);
}

/* inverts the matrix
 * (matrix must be square with non-zero determinant)
 */
matrix &matrix::invert(){
  if(cols() != rows()){
    printf("Can't invert %dx%d matrix\n", rows(), cols());
    abort();
  }

  if(cols()<1){
    printf("Can't invert degenerate matrix with size %d\n", cols());
    abort();
  }
  if(cols()==1)return *this;

  double determinant = det();
  if(fabs(determinant)<1e-20){
    printf("Can't invert matrix, determinant close to zero\n");
    abort();
  }

  matrix result(rows(), cols());
  unsigned int size = rows();
  double *M = (double *)alloca((size-1)*(size-1)*sizeof(double));

  for(unsigned int r=0;r<size;r++){
    for(unsigned int c=0;c<size;c++){

      /* copy the matrix, but not row r or column c */
      cropMatrix(size, size, c, r, data, M);

      /* get the determinant of that matrix */
      double d = ::det(size-1, M);

      /* fill in row r column c of the result */
      result[r][c] = ((r+c)%2?-1.0:1.0) * /* get(r, c) * */ d;
    }
  }

  /* normalize the result */
  result/=determinant;

  /* store the result */
  copy(result);
  return *this;
}

matrix &matrix::transpose(){
  unsigned int oldRows = numRows;
  unsigned int oldCols = numCols;
  double *oldData = data;
  
  numRows = oldCols;
  numCols = oldRows;
  double *newData = new double[numRows*numCols];

  for(unsigned int r=0;r<oldRows;r++)
    for(unsigned int c=0;c<oldCols;c++)
      newData[c*numCols + r] = oldData[r*oldCols + c];

  delete []data;
  data = newData;
  return *this;
}

void matrix::multiplyBy(const matrix &mat){
  matrix result(numRows, mat.numCols);
  matrixMult(result, *this, mat);
  copy(result);
}


void matrix::addMat(const matrix &mat){
  assert(numRows == mat.numRows);
  assert(numCols == mat.numCols);
  for(unsigned int r=0;r<numRows;r++)
    for(unsigned int c=0;c<numCols;c++)
      data[c*numCols + r] += mat.data[c*numCols + r];
}

void matrix::subMat(const matrix &mat){
  assert(numRows == mat.numRows);
  assert(numCols == mat.numCols);
  for(unsigned int r=0;r<numRows;r++)
    for(unsigned int c=0;c<numCols;c++)
      data[c*numCols + r] -= mat.data[c*numCols + r];
}

/* multiply and add a scalar */
void matrix::multiplyScalar(double scalar){
  for(unsigned int r=0;r<numRows;r++)
    for(unsigned int c=0;c<numCols;c++)
      data[c*numCols + r] *= scalar;
}
void matrix::addScalar(double scalar){
  for(unsigned int r=0;r<numRows;r++)
    for(unsigned int c=0;c<numCols;c++)
      data[c*numCols + r] += scalar;
}

matrix matrix::zeroes(unsigned int size){
  return matrix(size, size);
}

matrix matrix::eye(unsigned int size){
  matrix ret(size, size);
  for(unsigned int i=0;i<size;i++)
    ret[i][i]=1.0;
  return ret;
}

matrix horcat(const matrix &A, const matrix &B){
  assert(A.rows() == B.rows());
  matrix ret(A.rows(), A.cols() + B.cols());

  for(unsigned int r=0;r<A.rows();r++)
    for(unsigned int c=0;c<A.cols();c++)
      ret[r][c] = A[r][c];
  for(unsigned int r=0;r<B.rows();r++)
    for(unsigned int c=0;c<B.cols();c++)
      ret[r][A.cols()+c] = B[r][c];

  return ret;
}

matrix vercat(const matrix &A, const matrix &B){
  assert(A.cols() == B.cols());
  matrix ret(A.rows() + B.rows(), A.cols());

  for(unsigned int r=0;r<A.rows();r++)
    for(unsigned int c=0;c<A.cols();c++)
      ret[r][c] = A[r][c];
  for(unsigned int r=0;r<B.rows();r++)
    for(unsigned int c=0;c<B.cols();c++)
      ret[A.rows()+r][c] = B[r][c];

  return ret;
}

#if 1
int saneNonzero(double a){
  //return a!=0;
  return ((fabs(a) > 1e-100) && (fabs(a) < 1e100));
}

/* M is m x n
   U is m x m
   S is m x n (with m >= n and only the n elements 0,0 1,1... n-1,n-1 are given
   V is n x n
   Start with this 
     M = U*S*transpose(V)
   right-multiply by the inverses to get:
     U = M * inverse(transpose(V)) * inverse(S)
   since transpose(V)*V = identity
     U = M * V * inverse(S)
   since S is a diagonal matrix, just take the reciprocal of each element
     U = M * V * (1/S)
 */
static void computeU(const matrix &M, int m, int n, matrix &U, const matrix &S, const matrix &V){

  matrix tmp = M*V;
  //M.print("M");
  //V.print("V");
  //tmp.print("M*V");

  matrix Sinverse(S);
  for(unsigned int i=0;i<M.cols();i++)
    if(saneNonzero(S[i][i]))
      Sinverse[i][i] = 1.0 / S[i][i];

  U = M*V*Sinverse;
}

/**
 * based on code from http://www.idiom.com/~zilla/Computer/Javanumeric/
 * returns U in a. normaly U is nr*nr,
 * but if nr>nc only the first nc columns are returned
 * (nice, saves memory).
 * The columns of U have arbitrary sign,
 * also the columns corresponding to near-zero singular values
 * can vary wildly from other implementations.
 */
void static_svd(const matrix &Morig, matrix &U, matrix &S, matrix &V){
  matrix M(Morig);
  int i,its,j,jj,k,l=0,nm=0;
  int m = M.rows();
  int n = M.cols();
  int flag;
  double c,f,h,s,x,y,z;
  double anorm = 0., g = 0., scale=0. ;
  double *rv1 = (double *)alloca(n*sizeof(double));
  memset(rv1, 0, n*sizeof(double));

  assert(m>=n) ;

  for (i = 0; i<n; i++) {
    l = i+1;
    rv1[i] = scale*g;
    g = s = scale  = 0. ;
    if  (i<m) {
      for  (k = i; k<m; k++)  scale += fabs(M[k][i]) ;
      //if (scale!=0.0) {
      if (saneNonzero(scale)){
	for (k = i; k<m; k++) {
	  M[k][i] /= scale;
	  s += M[k][i]*M[k][i] ;
	}
	f = M[i][i];
	g = -SIGN(sqrt(s),f) ;
	h=f*g-s;
	M[i][i]=f-g;
	/*if (i!=(n-1)) {		// CHECK*/
	for (j = l; j<n; j++) {
	  for (s = 0,k = i; k<m; k++)
	    s += M[k][i]*M[k][j];
	  f = s/h;
	  for (k = i; k<m; k++)
	    M[k][j] += f*M[k][i];
	}
	/*}*/
	for (k = i; k<m; k++) M[k][i] *= scale;
      }
    }
    S[i][i] = scale*g;
    g = s = scale = 0.0 ;
    if (i<m && i!=n-1) {
      for (k = l; k<n; k++)
	scale += fabs(M[i][k]) ;
      //if  (scale != 0.) {
      if (saneNonzero(scale)){
	for  (k = l; k<n; k++) {
	  M[i][k]  /= scale;
	  s += M[i][k]*M[i][k] ;
	}
	f = M[i][l];
	g = -SIGN(sqrt(s),f);
	h = f*g-s;
	M[i][l] = f-g;
	for  (k = l; k<n; k++)
	  rv1[k] = M[i][k]/h;
	if (i!=m-1) {
	  for (j = l; j<m; j++) {
	    for (s = 0, k = l; k<n; k++)
	      s += M[j][k]*M[i][k] ;
	    for (k = l; k<n; k++)
	      M[j][k] += s*rv1[k] ;
	  }
	}
	for (k = l; k<n; k++)
	  M[i][k] *= scale;
      }
    } /*i<m && i!=n-1*/
    anorm = max(anorm,(fabs(S[i][i])+fabs(rv1[i])));
  } /*i*/
  for (i = n-1; i>=0; --i) {
    if (i<n-1) {
      //if (g != 0.) {
      if (saneNonzero(g)){
	for (j = l; j<n; j++)
	  V[j][i] = (M[i][j]/M[i][l])/g;
	for (j = l; j<n; j++) {
	  for (s = 0,k = l; k<n; k++)
	    s += M[i][k]*V[k][j];
	  for (k = l; k<n; k++)
	    V[k][j] += s*V[k][i];
	}
      }
      for (j = l; j<n; j++)
	V[i][j] = V[j][i] = 0.0;
    }
    V[i][i] = 1.0;
    g = rv1[i];
    l = i;
  }
  /*for (i=IMIN(m,n);i>=1;i--) {*/
  /*for (i = n-1; i>=0; --i)  {*/
  for (i = (int)min((double)(m-1),(double)(n-1)); i>=0; --i) {
    l = i+1;
    g = S[i][i];
    if (i<n-1)
      for (j = l; j<n; j++)
	M[i][j] = 0.0;
    //if (g != 0.) {
    if (saneNonzero(g)){
      g = 1./g;
      if (i!= n-1) {
	for(j = l; j<n; j++) {
	  for (s = 0, k = l; k<m; k++)
	    s += M[k][i]*M[k][j];
	  f = (s/M[i][i])*g;
	  for (k = i; k<m; k++)
	    M[k][j] += f*M[k][i];
	}
      }
      for (j = i; j < m; j++)
	M[j][i] *= g;
    }
    else {
      for (j = i; j<m; j++)
	M[j][i] = 0.0;
    }
    M[i][i] += 1.0;
  }
  for (k = n-1; k>=0; --k)  {
    for (its = 1; its<=30; ++its) {
      flag = 1;
      for (l = k; l>=0;  --l) {
	nm = l-1;
	if ((fabs(rv1[l])+anorm) == anorm) {
	  flag = 0;
	  break ;
	}
	if ((fabs(S[nm][nm])+anorm)  == anorm) break;
      }
      if (flag) {
	c = 0.0;
	s = 1.0;
	for (i = l; i<=k; i++)  {
	  f = s*rv1[i];
	  rv1[i] = c*rv1[i];
	  if ((fabs(f)+anorm)==anorm)
	    break;
	  g = S[i][i];
	  h = pythag(f,g) ;
	  S[i][i] = h;
	  h = 1.0/h;
	  c = g*h;
	  s = -f*h;
	  for (j = 0; j<m; j++) {
	    y = M[j][nm] ;
	    z = M[j][i];
	    M[j][nm] = y*c+z*s;
	    M[j][i] = z*c-y*s;
	  }
	}
      } /*flag*/
      z = S[k][k];
      if (l==k) {
	if (z<0.) {
	  S[k][k] = -z;
	  for (j = 0; j<n; j++)
	    V[j][k] = -V[j][k];
	}
	break;
      } /*l==k*/
      assert(its<50);
      x = S[l][l];
      nm = k-1;
      y = S[nm][nm];
      g = rv1[nm] ;
      h = rv1[k] ;
      f = ((y-z)*(y+z)+(g-h)*(g+h))/(2*h*y);
      g = pythag(f,1.0) ;
      f = ((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
      c = s = 1.0;
      for (j = l; j<=nm; j++) {
	i = j+1;
	g = rv1[i];
	y = S[i][i];
	h = s*g;
	g = c*g;
	z = pythag(f,h) ;
	rv1[j] = z;
	c = f/z;
	s = h/z;
	f = x*c+g*s;
	g = g*c-x*s;
	h = y*s;
	y *= c;
	for (jj = 0; jj<n; jj++) {
	  x = V[jj][j];
	  z = V[jj][i];
	  V[jj][j] = x*c+z*s;
	  V[jj][i] = z*c-x*s;
	}
	z = pythag(f,h) ;
	S[j][j] = z;
	//if (z != 0.0) {
	if (saneNonzero(z)){
	  z = 1.0/z;
	  c = f*z;
	  s = h*z;
	}
	f = c*g+s*y;
	x = c*y-s*g;
	for (jj = 0; jj<m; ++jj) {
	  y = M[jj][j];
	  z = M[jj][i];
	  M[jj][j] = y*c+z*s;
	  M[jj][i] = z*c-y*s;
	}
      } /*j<nm*/
      rv1[l] = 0.0;
      rv1[k] = f;
      S[k][k] = x;
    } /*its*/
  } /*k*/

  computeU(Morig, m, n, U, S, V);
} /*svd*/
#else

#error "You probably did not want to include svd.c..." 
#define NOMAIN
#include "svd.c"

void static_svd(const matrix &Morig, matrix &U, matrix &S, matrix &V){
  int m = Morig.rows();
  int n = Morig.cols();

  double **M2 = createEmptyMatrix(m, n);
  double **U2 = createEmptyMatrix(m, m);
  double *S2 = (double *)alloca(n, sizeof(double));
  memset(S2, 0, n*sizeof(double));
  double **V2 = createEmptyMatrix(n, n);

  for(int r = 0;r<m;r++)for(int c = 0;c<n;c++)M2[r][c] = Morig[r][c];
  for(int r = 0;r<m;r++)for(int c = 0;c<m;c++)U2[r][c] = U[r][c];
  for(int r = 0;r<n;r++)S2[r] = S[r][r];
  for(int r = 0;r<n;r++)for(int c = 0;c<n;c++)V2[r][c] = V[r][c];

  svd(M2, m, n, U2, S2, V2);

  for(int r = 0;r<m;r++)for(int c = 0;c<n;c++)Morig[r][c] = M2[r][c];
  for(int r = 0;r<m;r++)for(int c = 0;c<m;c++)U[r][c] = U2[r][c];
  for(int r = 0;r<n;r++)S[r][r] = S2[r];
  for(int r = 0;r<n;r++)for(int c = 0;c<n;c++)V[r][c] = V2[r][c];

  freeMatrix(M2, m, n);
  freeMatrix(U2, m, m);
  freeMatrix(V2, n, n);
}

#endif

#if 0
int main(int argc, char **argv){
  int nr = 3; int nc = 3;
  matrix M( nr, nc);
  matrix U( nr, nr);
  matrix S( nr, nc);
  matrix V( nc, nc);
  matrix R2(nc, nr);

  sscanf("3289.8  354.6  89.1", "%lf %lf %lf", &M[0][0], &M[0][1], &M[0][2]);
  sscanf(" 376.4 7441.5 663.5", "%lf %lf %lf", &M[1][0], &M[1][1], &M[1][2]);
  sscanf("  91.8  663.8 101.7", "%lf %lf %lf", &M[2][0], &M[2][1], &M[2][2]);

  M.svd(U, S, V);
  matrix Ut(U);
  Ut.transpose();
  matrixMult(R2, V, Ut);


  M.print("M");
  S.print("S");
  V.print("V");
  U.print("U");
  R2.print("R2");

  M.svdRotation().print("R2");

  return 0;
}
#endif
