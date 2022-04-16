/*
 * Operations to construct and multiply rotation matrices
 * convention: axis 0=x=pitch, axis 1=y=roll, axis 2=z=yaw
 *
 * Dom@cmu.edu
 */
#include "rotation_matrix.h"
#include "math.h"
#include <assert.h>
#include <stdio.h>
#include "rasm_common_types.h"

/* helper function
 * generates a 3x3 rotation matrix around the given axis
 * (axis 0=x=pitch, axis 1=y=roll, axis 2=z=yaw)
 */
static void rotationMatrix(double angle, int axis, float M[3][3]){
  assert(0 == axis || 1 == axis || 2 == axis);
  float c = cos(angle);
  float s = sin(angle);

  M[axis][0] = M[axis][1] = M[axis][2] = 0.0;
  M[0][axis] = M[1][axis] = M[2][axis] = 0.0;
  M[axis][axis] = 1.0;

  M[(axis+2)%3][(axis+2)%3] = c;
  M[(axis+2)%3][(axis+1)%3] = s;
  M[(axis+1)%3][(axis+2)%3] = -s;
  M[(axis+1)%3][(axis+1)%3] = c;
}

void matrixMult(float A[3][3], const float B[3][3], const float C[3][3]){
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      A[i][j] = B[i][0] * C[0][j] + B[i][1] * C[1][j] + B[i][2] * C[2][j];
}

void matrixMult(float A[4][4], const float B[4][4], const float C[4][4]){
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      A[i][j] = 0;
      for(int k=0;k<4;k++)
	A[i][j] += B[i][k] * C[k][j];
    }
  }
}

void rotationMatrix(double roll, double pitch, double yaw, float M[3][3]){
  float Rr[3][3], Rp[3][3], Ry[3][3];
  float Rrp[3][3]; /* temporary matrix Rrp = Rp*Rr */

  /* get the individual rotation matrices */
  rotationMatrix(roll,  1, Rr);
  rotationMatrix(pitch, 0, Rp);
  rotationMatrix(yaw,   2, Ry);

  /* do Rrp = Rp*Rr and M = Rrpy = Ry*Rrp to get the 3x3 rotation*/
  matrixMult(Rrp, Rp, Rr);
  matrixMult(M, Ry, Rrp);
}

void rotationMatrix(double roll, double pitch, double yaw, float M[4][4]){
  /* fill in the rotation */
  float r[3][3];
  rotationMatrix(roll, pitch, yaw, r);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      M[i][j] = r[i][j];

  /* clear the translation and bottom row */
  for(int i=0;i<3;i++)
    M[3][i] = M[i][3] = 0.0;
  M[3][3] = 1.0;
}
