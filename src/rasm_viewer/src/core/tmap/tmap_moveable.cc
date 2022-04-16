#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <tmap_moveable.h>
#include <../common/rotation_matrix.h>

/* translates the map by the specified amount */
void TMAP::tmap_moveable::translate(const RASM::point2d &amount){
  translate(RASM::point3d(amount.coord2d[0], amount.coord2d[1], 0));
}
void TMAP::tmap_moveable::translate(const RASM::point3d &amount){
  /* shift each point */
  for(unsigned int i=0;i<numPoints();i++)
    vertices[i] += amount;
}


void TMAP::tmap_moveable::generateTransformationMatrix(float M[4][4], 
						       const RASM::point3d &p,
						       double roll,
						       double pitch, 
						       double yaw){
  /* initialize and fill in the rotation */
  rotationMatrix(roll, pitch, yaw, M);

  /* fill in the translation */
  for(int i=0;i<3;i++)
    M[i][3] = p.coord3d[i];
}

static void applyTransformationMatrix(const float M[4][4], RASM::point3d &p){
  RASM::point3d tmp;
  for(int i=0;i<3;i++)
    tmp.coord3d[i] = (RASM_UNITS)(M[i][0]*(float)p.X() + M[i][1]*(float)p.Y() + M[i][2]*(float)p.Z() + M[i][3]);
  p = tmp;
}


/* rotate by this amount, all angles in radians */
void TMAP::tmap_moveable::rotate(double roll, double pitch, double yaw){
  translateAndRotate(RASM::point3d(0,0,0), roll, pitch, yaw);
}

/* a single homogeneous transform */
void TMAP::tmap_moveable::translateAndRotate(const RASM::point3d &amount, 
					     double roll,
					     double pitch,
					     double yaw){
  float M[4][4]; 
  generateTransformationMatrix(M, amount, roll, pitch, yaw);
  freeTransform(M);
#if 0
  TMAP_RASM::point3d t(0,0,0);
  double r=0, p=0, y=0;
  int success = recoverTransform(M, t, r, p, y);
  assert(success != -1);
  if(fabs(roll-r) + fabs(pitch-p) + fabs(yaw-y) > 1e-6){
    printf("Error, called with angles %f, %f, %f\n", roll, pitch, yaw);
    printf("But extracted angles %f, %f, %f\n", r, p, y);
    assert(0);
  }
  assert(distSq3d(t, amount) < RASM_TO_METERS(RASM_EPSILON));
#endif
}

void TMAP::tmap_moveable::freeTransform(const float matrix[4][4]){
  /* transform each point */
  for(unsigned int i=0;i<numPoints();i++)
    applyTransformationMatrix(matrix, vertices[i]);
}

/* given a transformation matrix, 
   recovers the translation and roll, pitch, yaw angles
   only works for pitch angles up to (but not including) +90 to -90
   returns -1 on error (eg, bottom row/matrix[3][...] is not 0,0,0,1)
*/
int TMAP::tmap_moveable::recoverTransform(const float matrix[4][4],
					  RASM::point3d &amount, 
					  double &roll,
					  double &pitch,
					  double &yaw){
  if(fabs(matrix[3][0]) + fabs(matrix[3][1]) + fabs(matrix[3][2]) >= RASM_TO_METERS(RASM_EPSILON)){
    printf("Error, called recoverTransform with non-zeros %f,%f,%f\n",
	   matrix[3][0], matrix[3][1], matrix[3][2]);
    return -1;
  }
  if(fabs(matrix[3][3] - 1.0) >= RASM_TO_METERS(RASM_EPSILON)){
    printf("Error, called recoverTransform with non-one %f\n",
	   matrix[3][3]);
    return -1;
  }

  for(int i=0;i<3;i++)
    amount.coord3d[i] = (RASM_UNITS)(matrix[i][3]);

  /* first get the pitch by using row 2, col 1
     this is sin(pitch) and we'll assume the pitch is -90 to +90
  */
  double sp = matrix[2][1];
  pitch = asin(sp);
  double cp = cos(pitch);

  double cy = matrix[1][1]/cp;
  double sy = -matrix[0][1]/cp;

  yaw = atan2(sy, cy);

  double cr = matrix[2][2]/cp;
  double sr = -matrix[2][0]/cp;

  roll = atan2(sr, cr);

  return 0;
}
