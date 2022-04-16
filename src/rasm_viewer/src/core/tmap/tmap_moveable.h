/*
 * Subclass of tmap_base to apply translation and rotation.
 *
 * Dominic Jonak
 */
#ifndef TMAP_MOVEABLE_H
#define TMAP_MOVEABLE_H

#include <tmap_base.h>

namespace TMAP {

  class tmap_moveable : public virtual tmap_base{

  public:

    tmap_moveable():tmap_base(){}
    tmap_moveable(const char *filename):tmap_base(filename){}

    /* translates the map by the specified amount */
    void translate(const RASM::point2d &amount);
    void translate(const RASM::point3d &amount);

    /* rotate by this amount, all angles in radians */
    void rotate(double roll, double pitch, double yaw);

    /* a single homogeneous transform */
    void translateAndRotate(const RASM::point3d &amount, 
			    double roll, double pitch, double yaw);

    /* apply an arbitrary transformation
     * matrix[i][j] is row i, column j
     */
    void freeTransform(const float matrix[4][4]);

    /* Generates a homogeneous transformation matrix
     */
    static void generateTransformationMatrix(float M[4][4], 
					     const RASM::point3d &p,
					     double roll,
					     double pitch,
					     double yaw);

    /* given a transformation matrix, 
     * recovers the translation and roll, pitch, yaw angles
     * only works for pitch angles up to (but not including) +90 to -90
     * returns -1 on error (eg, bottom row/matrix[3][...] is not 0,0,0,1)
     */
    static int recoverTransform(const float matrix[4][4],
				RASM::point3d &amount, 
				double &roll, double &pitch, double &yaw);
  };/* end of class tmap_moveable */

}/* end of namespace TMAP */

#endif
