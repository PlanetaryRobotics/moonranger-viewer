#include <planeHelpers.h>
#include <lineHelpers.h>
#include <rasm_common_types.h>

#include <assert.h>

static int inline min3(double dists[3]){
  if(dists[0] < dists[1] && dists[0] < dists[2])return 0;
  if(dists[1] < dists[2] && dists[1] < dists[0])return 1;
  return 2;
}

/* given a point p that lies in the plane of the three points tri
 * return the point closest to p (possibly itself)
 * that lies inside the triangle
 */
void TMAP::clipToTriangle(double px, double py, double pz, 
			  const RASM::point3d tri[3],
			  double &x, double &y, double &z){
  /* check if it falls inside */
  if(TMAP::sameSide(px, py,
		    RASM_TO_METERS(tri[0].X()), RASM_TO_METERS(tri[0].Y()),
		    tri[1], tri[2]) &&
     TMAP::sameSide(px, py,
		    RASM_TO_METERS(tri[1].X()), RASM_TO_METERS(tri[1].Y()),
		    tri[2], tri[0]) &&
     TMAP::sameSide(px, py,
		    RASM_TO_METERS(tri[2].X()), RASM_TO_METERS(tri[2].Y()),
		    tri[0], tri[1])){
    x=px;
    y=py;
    z=pz;
    return;
  }

  /* pick the best point on each side */
  double candidates[3][3];
  double dists[3];
  for(int i=0;i<3;i++){
    TMAP::closestPointOnLineSegment(px, py, pz,
				    tri[(i+1)%3], tri[(i+2)%3], 
				    candidates[i][0],
				    candidates[i][1],
				    candidates[i][2]);
    dists[i] = RASM::distSq3d(px, py, pz,
			      candidates[i][0],
			      candidates[i][1],
			      candidates[i][2]);
  }

  /* pick the best side */
  int i = min3(dists);
  x = candidates[i][0];
  y = candidates[i][1];
  z = candidates[i][2];
}


/* Ax + By + C = z */
void TMAP::getPlane(double &A, double &B, double &C,
		    const RASM::point3d tri[3]){
#if 0
  /* this uses the full plane fit formula, only inserting three points */
  double x0 = RASM_TO_METERS(tri[0].X());
  double y0 = RASM_TO_METERS(tri[0].Y());
  double z0 = RASM_TO_METERS(tri[0].Z());

  double x1 = RASM_TO_METERS(tri[1].X());
  double y1 = RASM_TO_METERS(tri[1].Y());
  double z1 = RASM_TO_METERS(tri[1].Z());

  double x2 = RASM_TO_METERS(tri[2].X());
  double y2 = RASM_TO_METERS(tri[2].Y());
  double z2 = RASM_TO_METERS(tri[2].Z());

  double S = 3.0;

  double Sx = x0 + x1 + x2;
  double Sy = y0 + y1 + y2;
  double Sz = z0 + z1 + z2;

  double Sxx = x0*x0 + x1*x1 + x2*x2;
  double Syy = y0*y0 + y1*y1 + y2*y2;
  //double Szz = z0*z0 + z1*z1 + z2*z2;

  double Sxy = x0*y0 + x1*y1 + x2*y2;
  double Syz = y0*z0 + y1*z1 + y2*z2;
  double Szx = z0*x0 + z1*x1 + z2*x2;

  double det = (Sx*(-(Syy*Sx) + Sxy*Sy) +
	       Sxy*( (Sy*Sx) - Sxy*S) +
	       Sxx*(-(Sy*Sy) + Syy*S));

  A = ((Sz *  (-(Sx*Syy) + Sxy*Sy) +
	Syz * ( (Sx*Sy) - Sxy*S) +
	Szx * (-(Sy*Sy) + Syy*S))/det);
  B = ((Sz *  ( (Sx*Sxy) - Sxx*Sy) +
	Syz * (-(Sx*Sx) + Sxx*S) +
	Szx * ( (Sy*Sx) - Sxy*S))/det);
  C = ((Sz *  (-(Sxy*Sxy) + Sxx*Syy) +
	Syz * ( (Sxy*Sx) - Sxx*Sy) +
	Szx * (-(Syy*Sx) + Sxy*Sy))/det);
#else
  /* this method solves the plane equation directly */

  double x10 = RASM_TO_METERS(tri[1].X() - tri[0].X());
  double y10 = RASM_TO_METERS(tri[1].Y() - tri[0].Y());
  double z10 = RASM_TO_METERS(tri[1].Z() - tri[0].Z());
  double x20 = RASM_TO_METERS(tri[2].X() - tri[0].X());
  double y20 = RASM_TO_METERS(tri[2].Y() - tri[0].Y());
  double z20 = RASM_TO_METERS(tri[2].Z() - tri[0].Z());

  static const double PLANE_EP = 1e-9;

  if(fabs(y20*x10 - y10*x20) < PLANE_EP){
    x20 = RASM_TO_METERS(tri[2].X() - tri[1].X());
    y20 = RASM_TO_METERS(tri[2].Y() - tri[1].Y());
    z20 = RASM_TO_METERS(tri[2].Z() - tri[1].Z());
  }
  B = (z20*x10 - z10*x20)/(y20*x10 - y10*x20);

  if(fabs(x10)< PLANE_EP)
    A = (z20 - B*y20)/x20;
  else
    A = (z10 - B*y10) / x10;

  C = RASM_TO_METERS(tri[0].Z()) - (A*RASM_TO_METERS(tri[0].X()) + B*RASM_TO_METERS(tri[0].Y()));
#endif
}

void TMAP::pointInPlane(double px, double py, double pz,
			const RASM::point3d tri[3], 
			double &x, double &y, double &z){
  double A, B, C;
  getPlane(A, B, C, tri);

  double x0 = RASM_TO_METERS(tri[0].X());
  double y0 = RASM_TO_METERS(tri[0].Y());
  double z0 = RASM_TO_METERS(tri[0].Z());

#if 0
  x = px;
  y = py;
  z = A*x + B*y + C;
#else
  /* Ax + By + C = z
     Ax + By + (-1)z + C = 0

     a = A, b = B, c = -1, d = C
   */
#if 0
  float D = (A*px + B*py - pz + C)/sqrt(A*A + B*B + 1);
  x = px - D*A;
  y = py - D*B;
  z = pz + D;
#else
  /* Ax + By + C = z
     Ax + By + (-1)z + C = 0

     the normal is [A B -1],
     the unit normal is [A B -1]/sqrt(A*A + B*B + 1.0)
     
     given a unit normal nA,nB,nC and a plane 0 = nA*x + nB*y + nC*z + nD,
     the distance to point p is D = nA*px + nB*py + nC*pz + nD
     and the closest point on the plane is (p - D*[nA nB nC])

     as with getPlane(), we use doubles during the intermediate steps
  */
  double D = sqrt(A*A + B*B + 1.0);
  double nA = A/D;
  double nB = B/D;
  double nC = -1.0/D;
  double nD = -(nA*x0 + nB*y0 + nC*z0);

  D = nA*px + nB*py + nC*pz + nD;

  x = ((double)px) - D*nA;
  y = ((double)py) - D*nB;
  z = ((double)pz) - D*nC;

  /*
  printf("Plane %10fx + %10fy + %10fz = %10f\n", nA, nB, nC, -nD);
  for(int i=0;i<3;i++){
    double tmpX = RASM_TO_METERS(tri[i].X());
    double tmpY = RASM_TO_METERS(tri[i].Y());
    double tmpZ = RASM_TO_METERS(tri[i].Z());
    printf("%10.10f*%0.3f + %10.10f*%0.3f + %10.10f*%0.3f = ", nA, tmpX, nB, tmpY, nC, tmpZ);
    printf("%f + %f + %f = ", nA*tmpX, nB*tmpY, nC*tmpZ);
    printf("%10.10f = %10.10f\n", nA*tmpX + nB*tmpY + nC*tmpZ, -nD);
  }
  printf("%10.10f*%10.10f + %10.10f*%10.10f + %10.10f*%10.10f = ", nA, x, nB, y, nC, z);
  printf("%10.10f = %10.10f\n", nA*x + nB*y + nC*z, -nD);
  printf("computedpoint=[%10.10f %10.10f %10.10f]\n", x, y, z);
  printf("target=[%10.10f %10.10f %10.10f]\n", px, py, pz);
  printf("dist=%10.10f\n", D);
  printf("norm=[%10.10f %10.10f %10.10f]\n", nA, nB, nC);
  printf("point = target - dist*norm\n");
  */
#endif

#endif
}

RASM::point3d TMAP::evaluatePoint(const RASM::point3d &p,
				  const RASM::point3d tri[3]){
  double x = RASM_TO_METERS(p.X());
  double y = RASM_TO_METERS(p.Y());
  double z = RASM_TO_METERS(p.Z());

  double x0,y0,z0;
  TMAP::pointInPlane(x,y,z, tri, x0,y0,z0);
  double x1,y1,z1;
  TMAP::clipToTriangle(x0,y0,z0, tri, x1,y1,z1);

  double d0 = RASM::distSq3d(x,y,z, x0,y0,z0);
  double d1 = RASM::distSq3d(x,y,z, x1,y1,z1);

  if(d0 > d1 + 1e-5){
    printf("\n\nError finding point near ");
    p.print();
    printf(" in:\n");
    printf("point3d a(");tri[0].print();printf(");\n");
    printf("point3d b(");tri[1].print();printf(");\n");
    printf("point3d c(");tri[2].print();printf(");\n");

    printf("Target point:           px=%f, py=%f, pz=%f\n",x,y,z);
    printf("Closest point in plane: %10.10f, %10.10f, %10.10f  distSq %10.10f\n",x0,y0,z0, d0);
    printf("Point in triangle:      %10.10f, %10.10f, %10.10f  distSq %10.10f\n",x1,y1,z1, d1);

    assert(0);
  }
  return RASM::point3d(METERS_TO_RASM(x1),
		       METERS_TO_RASM(y1),
		       METERS_TO_RASM(z1));
}


double TMAP::getHeightAt(const RASM::point2d &p, const RASM::point3d tri[3]){
#if 1
  double ax = RASM_TO_METERS(tri[0].X() - tri[1].X());
  double ay = RASM_TO_METERS(tri[0].Y() - tri[1].Y());
  double bx = RASM_TO_METERS(tri[0].X() - tri[2].X());
  double by = RASM_TO_METERS(tri[0].Y() - tri[2].Y());
  double hx = RASM_TO_METERS(tri[0].X() - p.X());
  double hy = RASM_TO_METERS(tri[0].Y() - p.Y());

  double det = ax*by - bx*ay;

  /* barycentric coordinates of p */
  double y = (by*hx - bx*hy)/det;
  double z = (ax*hy - ay*hx)/det;
  double x = 1 - y - z;

  return (x * RASM_TO_METERS(tri[0].Z()) + 
	  y * RASM_TO_METERS(tri[1].Z()) + 
	  z * RASM_TO_METERS(tri[2].Z()) );
#else
  double A, B, C;
  getPlane(A, B, C, tri);

  double x = RASM_TO_METERS(p.X());
  double y = RASM_TO_METERS(p.Y());
  double z = A*x + B*y + C;

  return z;
#endif
}


#if 1
bool TMAP::contains(const RASM::point3d *vertices, 
		    const RASM::triangle &t,
		    const RASM::point2d &p, int verbose){
  for(int i=0;i<3;i++){
    RASM::point2d a(vertices[t.points[i]]);
    RASM::point2d b(vertices[t.points[(i+1)%3]]);
    if(isColinear(p, a, b)){
      /* make sure p falls between a and b */
      a -= p;
      b -= p;
      /* a and b should now have opposite signs in X and Y (or be 0) */
      if((a.X() >=  RASM_EPSILON && b.X() >=  RASM_EPSILON) ||
	 (a.X() <= -RASM_EPSILON && b.X() <= -RASM_EPSILON) ||
	 (a.Y() >=  RASM_EPSILON && b.Y() >=  RASM_EPSILON) ||
	 (a.Y() <= -RASM_EPSILON && b.Y() <= -RASM_EPSILON) )
	return false;

      if(verbose){
	printf("contains() colinear test\n");
	p.print(stdout);
	printf(", ");
	vertices[t.points[i]].print(stdout);
	printf(" and ");
	vertices[t.points[(i+1)%3]].print(stdout);
	printf("\n");
      }
      return true;
    }
  }

  for(int i=0;i<3;i++)
    if(!sameSide(p, vertices[t.points[i]], 
		 vertices[t.points[(i+1)%3]], 
		 vertices[t.points[(i+2)%3]], 
		 verbose))
      return false;

  if(verbose){
    p.print();
    printf(" is in this triangle: \n");
    for(int i=0;i<3;i++){
      vertices[t.points[i]].print();
      printf("\n");
    }
  }
  return true;
}
#else
bool TMAP::contains(const RASM::point3d *vertices,
		    const RASM::triangle &t,
		    const RASM::point2d &p, int verbose){

  const RASM::point2d a = vertices[t.points[0]];
  const RASM::point2d b = vertices[t.points[1]];
  const RASM::point2d c = vertices[t.points[2]];

  if(verbose){
    printf("checking ");
    p.print();
    printf(" in ");
    a.print();
    printf(", ");
    b.print();
    printf(", ");
    c.print();
    printf("\n");
  }

  /* we need to determine if that point is inside the triangle
     look at the triangles formed by the corners and p (pab, pbc and pca)
     p is interior if the normals all point of the same way
     so we will examine the z component of the norm of each
   */
  const RASM::point2d pa = a-p;
  const RASM::point2d pb = b-p;
  const RASM::point2d pc = c-p;

  if(verbose){
    printf("Relative coords: ");
    pa.print();
    printf(" (dist sq %f), ", distSq2d(a, p));
    pb.print();
    printf(" (dist sq %f), ", distSq2d(b, p));
    pc.print();
    printf(" (dist sq %f)\n", distSq2d(c, p));
  }

  /* to avoid overflow problems, do a bound box test */
  if(pa.X() < 0 && pb.X() < 0 && pc.X() < 0)return false;
  if(pa.X() > 0 && pb.X() > 0 && pc.X() > 0)return false;
  if(pa.Y() < 0 && pb.Y() < 0 && pc.Y() < 0)return false;
  if(pa.Y() > 0 && pb.Y() > 0 && pc.Y() > 0)return false;

  /* get the z component of the cross product of pa and pb, pb and pc, and pc and pa */
  /* TODO stress test this code to make sure there are really no overflow problems */
  //RASM_UNITS normABz = pa.X()*pb.Y() - pa.Y()*pb.X();
  //RASM_UNITS normBCz = pb.X()*pc.Y() - pb.Y()*pc.X();
  //RASM_UNITS normCAz = pc.X()*pa.Y() - pc.Y()*pa.X();
  const float normABz = (float)(pa.X())*(float)(pb.Y()) - 
    (float)(pa.Y())*(float)(pb.X());
  const float normBCz = (float)(pb.X())*(float)(pc.Y()) - 
    (float)(pb.Y())*(float)(pc.X());
  const float normCAz = (float)(pc.X())*(float)(pa.Y()) - 
    (float)(pc.Y())*(float)(pa.X());

  if(verbose){
    printf("norms: ");
    //printf(RASM_UNITS_FORMAT, normABz);
    printf("%f", normABz);
    printf(" ");
    //printf(RASM_UNITS_FORMAT, normBCz);
    printf("%f", normBCz);
    printf(" ");
    //printf(RASM_UNITS_FORMAT, normCAz);
    printf("%f", normCAz);
    printf("\n");
  }

  if(normABz > 0 && normBCz > 0 && normCAz > 0)return true;
  if(normABz < 0 && normBCz < 0 && normCAz < 0)return true;
  return false;
}
#endif

void TMAP::triangleLengths(const RASM::point3d tri[3], double dists[3]){
  dists[0] = dist3d(tri[0], tri[1]);
  dists[1] = dist3d(tri[1], tri[2]);
  dists[2] = dist3d(tri[2], tri[0]);
}

void TMAP::triangleLengths(const RASM::point3d &a, 
			   const RASM::point3d &b, 
			   const RASM::point3d &c, 
			   double dists[3]){
  dists[0] = dist3d(a, b);
  dists[1] = dist3d(b, c);
  dists[2] = dist3d(c, a);
}

double TMAP::trianglePerimeter(const RASM::point3d tri[3]){
  return (dist3d(tri[0], tri[1]) +
	  dist3d(tri[1], tri[2]) +
	  dist3d(tri[2], tri[0]) );
}

double TMAP::trianglePerimeter(const RASM::point3d &a,
			       const RASM::point3d &b,
			       const RASM::point3d &c){
  return (dist3d(a, b) + dist3d(b, c) + dist3d(c, a));
}

