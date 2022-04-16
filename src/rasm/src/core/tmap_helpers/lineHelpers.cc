#include <lineHelpers.h>
#include <stdio.h>
#include <assert.h>
#include <rasm_common_types.h>

/* returns true if the 3 points are colinear */
bool TMAP::isColinear(const RASM::point2d &a, 
		      const RASM::point2d &b, 
		      const RASM::point2d &c){
  /* get the vector from a to b and from a to c */
  RASM::point2d db = a - b;
  RASM::point2d dc = a - c;

  /* check if the slopes are the same
     ie, if db.Y()/db.X() equals dc.Y()/dc.X()
  */
  return RASM_UNITS_equals(db.Y() * dc.X(), dc.Y() * db.X());
}

bool TMAP::isColinear(const RASM::point3d *p, const RASM::triangle &t){
  return isColinear(p[t.points[0]], p[t.points[1]], p[t.points[2]]);
}


/* returns true if points a and b lie on the same side of the
 * line defined by edgeA and edgeB
 */
bool TMAP::sameSide(float ax, float ay,         float bx, float by,
		    const RASM::point2d &edgeA, const RASM::point2d &edgeB, 
		    int verbose){
  /* vertical line */
  if(RASM_UNITS_equals(edgeA.X(), edgeB.X())){
    if(verbose)
      printf("Vertical line\n");
    return ((ax > RASM_TO_METERS(edgeA.X())) == 
	    (bx > RASM_TO_METERS(edgeA.X())) );
  }

  /* get the line parameters */
  const RASM::point2d d = edgeA - edgeB;
  const float slope = (float)(d.Y())/(float)(d.X());
  const float intercept = RASM_TO_METERS(edgeA.Y()) - 
    (slope * RASM_TO_METERS(edgeA.X()));
  if(verbose)
    printf("line with formula y = %fx + %f\n", slope, intercept);

  return ((ay > (intercept + (slope * ax))) ==
	  (by > (intercept + (slope * bx))) );
}

bool TMAP::sameSide(const RASM::point2d &a,     const RASM::point2d &b,
		    const RASM::point2d &edgeA, const RASM::point2d &edgeB, 
		    int verbose){
  /* vertical line */
  if(RASM_UNITS_equals(edgeA.X(), edgeB.X())){
    if(verbose)
      printf("Vertical line\n");
    return ((a.X() > edgeA.X()) == (b.X() > edgeA.X()));
  }

  /* get the line parameters */
  const RASM::point2d d = edgeA - edgeB;
  const float slope = (float)(d.Y())/(float)(d.X());
  const float intercept = (float)(edgeA.Y()) - (slope * (float)(edgeA.X()));
  if(verbose)
    printf("line with formula y = %fx + %f\n", slope, intercept);

  return (((float)(a.Y()) > (intercept + (slope * (float)(a.X())))) ==
	  ((float)(b.Y()) > (intercept + (slope * (float)(b.X())))) );
}


/* return true if line segment ab crosses cd */
bool TMAP::intersects(const RASM::point3d &a, const RASM::point3d &b,
		      const RASM::point3d &c, const RASM::point3d &d){
  /* check if a and b are on the same side of line cd */
  if(sameSide(a, b, c, d, 0))return false;
  /* check if c and d are on the same side of line ab */
  if(sameSide(c, d, a, b, 0))return false;

  /* otherwise they must intersect */
  return true;
}


/* given a point p, and a line segmend defined by points a and b
 * return the point on the line segment closest to p
 */
void TMAP::closestPointOnLineSegment(double px, double py, double pz,
				     const RASM::point3d &a,
				     const RASM::point3d &b,
				     double &x, double &y, double &z){
  double ax = RASM_TO_METERS(a.X());
  double ay = RASM_TO_METERS(a.Y());
  double az = RASM_TO_METERS(a.Z());
  double bx = RASM_TO_METERS(b.X());
  double by = RASM_TO_METERS(b.Y());
  double bz = RASM_TO_METERS(b.Z());

  /*
  printf("p=[%f %f %f];a=[%f %f %f];b=[%f %f %f];line=[a;b];\n",
	 RASM_TO_METERS(p.X()), RASM_TO_METERS(p.Y()), RASM_TO_METERS(p.Z()),
	 RASM_TO_METERS(a.X()), RASM_TO_METERS(a.Y()), RASM_TO_METERS(a.Z()),
	 RASM_TO_METERS(b.X()), RASM_TO_METERS(b.Y()), RASM_TO_METERS(b.Z()));
  printf("hold off;plot(line(:,1), line(:,2), 'r*-');hold on;plot(b(1,1), b(1,2), 'b*');plot(p(1,1), p(1,2), 'g*');\n");
  */


  /* the line through a and b: a + t*(b-a)
     t = 0 is one end point, t = 1 is the other end point
     solve for the best t (ie, minimize the distance between p and a+t*(b-a))
     with the formula t = -dot((a-p), (b-a))/distSq3d(a, b);

     if that falls outside 0-1 then p is closer to an end point
   */
#define DOT_PRODUCT(ax, ay, az, bx, by, bz) (((ax)*(bx)) + ((ay)*(by)) + ((az)*(bz)))
  double t = -DOT_PRODUCT(ax-px, ay-py, az-pz,  bx-ax, by-ay, bz-az) / RASM::distSq3d(ax,ay,az, bx,by,bz);
#undef DOT_PRODUCT
  /*
  printf("Line segment from %0.3f,%0.3f,%0.3f to %0.3f,%0.3f,%0.3f, target: %0.3f,%0.3f,%0.3f, found t=%f\n",
	 ax,ay,az,  bx,by,bz,  px,py,pz,  t);
  */

  int error=0;
  if(t<0.5-1e-5)
    error = !(RASM::distSq3d(ax,ay,az, px,py,pz) < RASM::distSq3d(bx,by,bz, px,py,pz) + 1e-5);
  else if(t>0.5+1e-5)
    error = !(RASM::distSq3d(ax,ay,az, px,py,pz) > RASM::distSq3d(bx,by,bz, px,py,pz) - 1e-5);
  if(error){
    printf("Line segment from %0.3f,%0.3f,%0.3f to %0.3f,%0.3f,%0.3f, target: %0.3f,%0.3f,%0.3f, found t=%f\n",
	   ax,ay,az,  bx,by,bz,  px,py,pz,  t);
    printf("Dist from A: %f, Dist from B: %f\n", RASM::distSq3d(ax,ay,az, px,py,pz), RASM::distSq3d(bx,by,bz, px,py,pz));
    assert(0);
  }

  /* select an end point if necessary */
  if(t<=0.0){
    x = ax;
    y = ay;
    z = az;
    return;
  }
  if(t>=1.0){
    x = bx;
    y = by;
    z = bz;
    return;
  }

  /* interpolate between a and b */
  double dx = bx-ax;
  double dy = by-ay;
  double dz = bz-az;
  x = (double)(dx*t + ax);
  y = (double)(dy*t + ay);
  z = (double)(dz*t + az);
}


int TMAP::getIntersection(const RASM::point3d &a, const RASM::point3d &b,
			  const RASM::point3d &c, const RASM::point3d &d,
			  RASM::point3d &intersection){
  /* if there is a shared point, use that */
  if(a.equals(c) || a.equals(d)){
    intersection = a;
    return 0;
  }
  if(b.equals(c) || b.equals(d)){
    intersection = b;
    return 0;
  }

  /* make sure they're not both vertical */
  if((RASM_UNITS_equals(a.X(), b.X()) &&
      RASM_UNITS_equals(c.X(), d.X()) )){

    /* check if they are overlapping */
    if(RASM_UNITS_equals(c.X(), d.X())){
      if(a.Y() >= c.Y() && a.Y() <= d.Y()){
	intersection = a;
	return 0;
      }
      if(b.Y() >= c.Y() && b.Y() <= d.Y()){
	intersection = b;
	return 0;
      }
      if(c.Y() >= a.Y() && c.Y() <= b.Y()){
	intersection = c;
	return 0;
      }
      if(d.Y() >= d.Y() && d.Y() <= d.Y()){
	intersection = d;
	return 0;
      }
    }


    printf("Error, trying to get the intersection of two vertical lines\n");
    printf("AB: ");
    a.print();
    printf(" to ");
    b.print();
    printf("\n");

    printf("CD: ");
    c.print();
    printf(" to ");
    d.print();
    printf("\n");

    return -1;
  }

  /* looking for t such that a + t*vecAB lies on CD */
  RASM::point3d vecAB = b-a;
  RASM::point3d vecCD = d-c;
  float t=0;

#if DEBUG && 0
  printf(" vecAB = ");
  b.print();
  printf(" - ");
  a.print();
  printf(" = ");
  vecAB.print();
  printf("\n");

  printf(" vecCD = ");
  d.print();
  printf(" - ");
  c.print();
  printf(" = ");
  vecCD.print();
  printf("\n");
#endif

  /* AB is vertical */
  if(RASM_UNITS_equals(a.X(), b.X())){
    /* AB is vertical, get the intersection point is where CD crosses a.X() */

    /* first get t such that c + t*vecCD has that x value */
    t = ((float)(a.X()-c.X()))/((float)(vecCD.X()));

    /* get the y component of that point */
    float y = (float)c.Y() + t*(float)(vecCD.Y());
    
    /* use the y component to get t such that a + t*vecAB has that y value */
    t = ((float)(y-a.Y()))/((float)(vecAB.Y()));
  }else if(RASM_UNITS_equals(c.X(), d.X())){
    /* CD is vertical, get the intersection point is where AB crosses c.X() */

    /* get t such that a + t*vecAB has that x value */
    t = ((float)(c.X()-a.X()))/((float)(vecAB.X()));
  }else{

    /* neither is vertical, get the slope of each */
    float slopeAB = ((float)vecAB.Y())/((float)vecAB.X());
    float interceptAB = ((float)a.Y()) - ((float)a.X()) * slopeAB;
    float slopeCD = ((float)(vecCD.Y()))/((float)(vecCD.X()));
    float interceptCD = ((float)c.Y()) - ((float)c.X()) * slopeCD;

    /* find X such that slopeAB*X + interceptAB = slopeCD*X + interceptCD */
    float x = (interceptCD - interceptAB)/(slopeAB - slopeCD);

    /* get t such that a + t*vecAB has that x value */
    t = (x- ((float)a.X()))/((float)(vecAB.X()));
#if DEBUG && 0
    printf("line AB: y = %0.3fX + %0.3f\n", slopeAB, interceptAB);
    printf("line CD: y = %0.3fX + %0.3f\n", slopeCD, interceptCD);
    printf("Intersect at X=%0.3f (t=%0.3f)\n", x, t);
#endif
  }

  /* clamp to range */
  if(t>1.0)t=1.0;
  if(t<0.0)t=0.0;

  /* so the intersection point is a + vecAB*t */
#if DEBUG && 0
  printf(" Intersection is ");
  a.print();
  printf(" + (");
  vecAB.print();
  printf(") * %0.3f\n", t);
#endif

  for(int i=0;i<3;i++)
    vecAB.coord3d[i] = (RASM_UNITS)(t*(float)vecAB.coord3d[i]);
  intersection = a + vecAB;
  return 0;
}
