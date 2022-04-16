#ifndef MATRIX_CLASS_H
#define MATRIX_CLASS_H

#include <assert.h>

class matrix{
 protected:
  unsigned int numRows;
  unsigned int numCols;
  double *data;
  int doChecks;

  unsigned int getIndex(unsigned int row, unsigned int col) const;

  void copy(const matrix &copy);
  void copy(const matrix *copy);

  /* right multiplies this matrix by mat
     performs this = this * mat
   */
  void multiplyBy(const matrix &mat);

  /* adds the components piecewise,
   * the matrix dimensions must match
   */
  void addMat(const matrix &mat);

  /* subtracts the components piecewise,
   * the matrix dimensions must match
   */
  void subMat(const matrix &mat);

  /* multiply and add a scalar */
  void multiplyScalar(double scalar);
  void addScalar(double scalar);

 public:
  matrix(unsigned int rows, unsigned int cols);
  matrix(const matrix &mat);
  matrix(const matrix *mat);
  ~matrix();
  void disableSafetyChecks(){doChecks=0;}

  double *get(unsigned int row) const;
  double &get(unsigned int row, unsigned int col) const;
  unsigned int rows() const;
  unsigned int cols() const;
  void printRow(unsigned int row, const char *name) const;
  void print(const char *name) const;
  void consistencyCheck() const;

  /* get the determinant of this matrix 
   * (matrix must be square)
   */
  double det()const;

  /* inverts the matrix
   * (matrix must be square with non-zero determinant)
   */
  matrix &invert();

  /* transposes the matrix, swaps the rows and cols
     data[i][j] becomes data[j][i]
   */
  matrix &transpose();

  /* fills in U, S and V such that
     1) this = U*S*transpose(V) 
     2) U*transpose(U) is an identity
     3) V*transpose(V) is an identity
     4) and S only has entries on the diagonal
  */
  void svd(matrix &U, matrix &S, matrix &V) const;

  /* runs svd and returns V*transpose(U) */
  matrix svdRotation() const;

  double *operator [] ( unsigned int index ) const
    { return get(index); }

  matrix &operator = ( const matrix &mat )
    { copy(mat);return *this; }

  matrix &operator *= ( const matrix &mat )
    { multiplyBy(mat);return *this; }

  matrix &operator += ( const matrix &mat )
    { addMat(mat);return *this; }

  matrix &operator -= ( const matrix &mat )
    { subMat(mat);return *this; }

  matrix &operator *= ( double scalar )
    { multiplyScalar(scalar);return *this; }

  matrix &operator /= ( double scalar )
    { multiplyScalar(1.0/scalar);return *this; }

  matrix &operator += ( double scalar )
    { addScalar(scalar);return *this; }

  matrix &operator -= ( double scalar )
    { addScalar(-scalar);return *this; }

  static matrix zeroes(unsigned int size);
  static matrix eye(unsigned int size);
};


matrix inline operator * ( const matrix &A, const matrix &B ){
  matrix result(A);
  result *= B;
  return result;
}

matrix inline operator - ( const matrix &A, const matrix &B ){
  assert(A.rows() == B.rows());
  assert(A.cols() == B.cols());
  matrix result(A);
  for(unsigned int r=0;r<A.rows();r++)
    for(unsigned int c=0;c<A.cols();c++)
      result[r][c] = A[r][c] - B[r][c];
  return result;
}

/* concatenates the rows of these matrices */
matrix horcat(const matrix &A, const matrix &B);
/* concatenates the cols of these matrices */
matrix vercat(const matrix &A, const matrix &B);

#endif
