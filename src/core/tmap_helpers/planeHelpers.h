/*
 * Helper functions related to planes/triangles
 *
 * Dominic Jonak
 */
#ifndef PLANE_HELPERS_H
#define PLANE_HELPERS_H

#include <rasm_common_types.h>

namespace TMAP {

  /* given a point p that lies in the plane of the three points tri
   * return the point closest to p (possibly itself)
   * that lies inside the triangle
   */
  void clipToTriangle(double px, double py, double pz, 
		      const RASM::point3d tri[3],
		      double &x, double &y, double &z);

  /* Ax + By + C = z */
  void getPlane(double &A, double &B, double &C, const RASM::point3d tri[3]);

  /* given three points of a triangle and a point P
   * project P onto the plane defined by the triangle
   */
  void pointInPlane(double px, double py, double pz,
		    const RASM::point3d tri[3], 
		    double &x, double &y, double &z);

  /* like above, but if that point falls outside the triangle,
   * then instead return the closest point inside the triangle
   */
  RASM::point3d evaluatePoint(const RASM::point3d &p, const RASM::point3d tri[3]);

  /* given three points of a triangle and a 2-D point P
   * get the plane determined by the triangle and the altitude at P
   */
  double getHeightAt(const RASM::point2d &p, const RASM::point3d tri[3]);

  /* returns true if point p falls inside triangle T
   * since T contains three indicies, pass the array of vertices
   */
  bool contains(const RASM::point3d *vertices, 
		const RASM::triangle &t,
		const RASM::point2d &p, int verbose=0);

  /* given three points of a triangle
   * fill in the array of edge lengths
   */
  void triangleLengths(const RASM::point3d tri[3], double dists[3]);
  void triangleLengths(const RASM::point3d &a, const RASM::point3d &b, const RASM::point3d &c, 
		       double dists[3]);

  /* returns the perimeter of the triangle */
  double trianglePerimeter(const RASM::point3d tri[3]);
  double trianglePerimeter(const RASM::point3d &a,const RASM::point3d &b,const RASM::point3d &c);

}/* end of namespace TMAP */

#endif /*  PLANE_HELPERS_H */
