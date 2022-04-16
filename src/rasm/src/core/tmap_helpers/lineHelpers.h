/*
 * Helper functions related to lines/segments
 *
 * Dominic Jonak
 */
#ifndef LINE_HELPERS_H
#define LINE_HELPERS_H

#include <rasm_common_types.h>


namespace TMAP {

  /* returns true if the 3 points are colinear */
  bool isColinear(const RASM::point2d &a,
		  const RASM::point2d &b,
		  const RASM::point2d &c);

  bool isColinear(const RASM::point3d *p, const RASM::triangle &t);

  /* returns true if points a and b lie on the same side of the
   * line defined by edgeA and edgeB
   */
  bool sameSide(float ax, float ay,   float bx, float by,
		const RASM::point2d &edgeA, const RASM::point2d &edgeB, 
		int verbose=0);

  bool sameSide(const RASM::point2d &a,     const RASM::point2d &b,
		const RASM::point2d &edgeA, const RASM::point2d &edgeB, 
		int verbose=0);

  /* return true if line segment ab crosses cd */
  bool intersects(const RASM::point3d &a,   const RASM::point3d &b,
		  const RASM::point3d &c,   const RASM::point3d &d);

  /* given a point p, and a line segmend defined by points a and b
   * return the point on the line segment closest to p
   */
  void closestPointOnLineSegment(double px, double py, double pz,
				 const RASM::point3d &a, const RASM::point3d &b,
				 double &x, double &y, double &z);

  /* given the endpoint of two line segments
   * determine where they intersect
   * returns -1 on error (eg, no intersection)
   */
  int getIntersection(const RASM::point3d &a, const RASM::point3d &b,
		      const RASM::point3d &c, const RASM::point3d &d,
		      RASM::point3d &intersection);

}/* end of namespace TMAP */


#endif /*  LINE_HELPERS_H */
