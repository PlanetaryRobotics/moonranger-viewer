#include "helpers.h"
#include <sys/time.h>
#include "myplane.h"
#include <assert.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

/*** start of OpenGL related functions ***/
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

void point::glVertex() const{glVertex3f(x, y, z);}
void point::glNormal() const{glNormal3f(x, y, z);}

const int useSlopeColor=0;
void setSlopeColor(const point &normal){
  point p = normal/normal.norm();
  METERS minZ=0.7;
  METERS maxZ = 0.99;
  if(p.z < minZ){
    glColor4f(1,0,0,1);
  }else if(p.z>maxZ){
    glColor4f(0,1,0,1);
  }else{
    float percent = (p.z - minZ)/(maxZ - minZ);
    glColor4f(1.0-percent, percent, 0,1);
  }
}

void triangle::draw(){
  if(useSlopeColor)setSlopeColor(norm);
  norm.glNormal();
  glBegin(GL_TRIANGLES);
  a.glVertex();
  b.glVertex();
  c.glVertex();
  glEnd();
}

void triangle::drawWire(){
  glLineWidth(1.0);
  glBegin(GL_LINE_STRIP);
  a.glVertex();
  b.glVertex();
  c.glVertex();
  a.glVertex();
  glEnd();
}

void rotation::apply()const{
  //printf("Applying rotation of %g, %g, %g radians\n", roll, pitch, yaw);
  glRotatef(roll   * 180.0/M_PI, 0, 1, 0);
  glRotatef(pitch  * 180.0/M_PI, 1, 0, 0);
  glRotatef(yaw    * 180.0/M_PI, 0, 0, 1);
}

void rotation::undo()const{
  glRotatef(-yaw  * 180.0/M_PI, 0, 0, 1);
  glRotatef(-pitch* 180.0/M_PI, 1, 0, 0);
  glRotatef(-roll * 180.0/M_PI, 0, 1, 0);
}

void translation::apply()const{
  //printf("Applying translation of %g, %g, %g\n", x, y, z);
  glTranslatef(x, y, z);
}

void translation::undo()const{
  glTranslatef(-x, -y, -z);
}

/*** end of OpenGL related functions ***/


void point::applyTransform(const float rpy[3][3],
			   float dx, float dy, float dz){
  METERS x2 = x, y2 = y, z2 = z;

  x = rpy[0][0]*x2 + rpy[0][1]*y2 + rpy[0][2]*z2 + dx;
  y = rpy[1][0]*x2 + rpy[1][1]*y2 + rpy[1][2]*z2 + dy;
  z = rpy[2][0]*x2 + rpy[2][1]*y2 + rpy[2][2]*z2 + dz;
}

SECONDS getCurTime(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return ((double)tv.tv_sec) + (((double)tv.tv_usec) / 1e7);
}

triangle::triangle(point _a, point _b, point _c):a(_a),b(_b),c(_c),norm(0,0,0){
  ind_A=ind_B=ind_C = -1;

  point i(b.x-a.x, b.y-a.y, b.z-a.z);
  point j(c.x-a.x, c.y-a.y, c.z-a.z);
  norm.x = i.y*j.z - i.z*j.y;
  norm.y = i.z*j.x - i.x*j.z;
  norm.z = i.x*j.y - i.y*j.x;
  norm /= norm.norm();
}

void triangle::setInd(int a, int b, int c){
  ind_A = a;
  ind_B = b;
  ind_C = c;
}

void triangle::print(){
  printf("index %d: ", ind_A);a.print("Corner A: ", "\n");
  printf("index %d: ", ind_B);b.print("Corner B: ", "\n");
  printf("index %d: ", ind_C);c.print("Corner C: ", "\n");
  norm.print("Normal: ", "\n");
}

/* prefer a, then b, then c */
point triangle::getLow() const{
  if(c.z<a.z && c.z<b.z)return c;
  if(b.z<a.z)return b;
  return a;
}

/* prefer b, then c, then a */
point triangle::getMid() const{
  point low = getLow();
  point high = getHigh();
  if(distSqBetween(a, low)>0 &&
     distSqBetween(a, high)>0)return a;
  if(distSqBetween(c, low)>0 &&
     distSqBetween(c, high)>0)return c;
  return b;
}

/* prefer c, then a, then b */
point triangle::getHigh() const{
  if(b.z>a.z && b.z>c.z)return b;
  if(a.z>c.z)return a;
  return c;
}

bool triangle::contains2D(const point &p) const{
#if 1
  const METERS epsilon=1e-11;
  if((fabs(p.x-a.x)<=epsilon) && (fabs(p.y-a.y)<=epsilon))return true;
  if((fabs(p.x-b.x)<=epsilon) && (fabs(p.y-b.y)<=epsilon))return true;
  if((fabs(p.x-c.x)<=epsilon) && (fabs(p.y-c.y)<=epsilon))return true;
#endif

  /* project the 3 vertices and this point to z=0  */
  point az(a.x, a.y, 0);
  point bz(b.x, b.y, 0);
  point cz(c.x, c.y, 0);
  point pz(p.x, p.y, 0);


  /* now we need to determine if that point is inside the triangle
     look at the three triangles formed by the corners and intersection
     the point is interior of the normals all points of the same way
   */
  triangle ta(pz, bz, cz), tb(pz, cz, az), tc(pz, az, bz);
  if(ta.norm.z > 0 && tb.norm.z > 0 && tc.norm.z > 0)return true;
  if(ta.norm.z < 0 && tb.norm.z < 0 && tc.norm.z < 0)return true;
  return false;
}


namespace rayCast{

#define GEOMETRY_EPSILON (1e-6)

class tuple3f{
 public:
  float x, y, z;
  tuple3f():x(0), y(0), z(0){}
  tuple3f(float _x, float _y, float _z):x(_x), y(_y), z(_z){}
};

float dot(const tuple3f &a, const tuple3f &b){
  assert(isfinite(a.x) && isfinite(a.y) && isfinite(a.z));
  assert(isfinite(b.x) && isfinite(b.y) && isfinite(b.z));

  return a.x*b.x + a.y*b.y + a.z*b.z;
}

tuple3f operator-(const tuple3f &a, const tuple3f &b){
  assert(isfinite(a.x) && isfinite(a.y) && isfinite(a.z));
  assert(isfinite(b.x) && isfinite(b.y) && isfinite(b.z));

  return tuple3f(a.x - b.x, a.y - b.y, a.z - b.z);
}

tuple3f operator+(const tuple3f &a, const tuple3f &b){
  assert(isfinite(a.x) && isfinite(a.y) && isfinite(a.z));
  assert(isfinite(b.x) && isfinite(b.y) && isfinite(b.z));

  return tuple3f(a.x + b.x, a.y + b.y, a.z + b.z);
}

tuple3f operator*(const tuple3f &a, float f){
  assert(isfinite(a.x) && isfinite(a.y) && isfinite(a.z));
  assert(isfinite(f));

  return tuple3f(a.x*f, a.y*f, a.z*f);
}

int rayPlane(const tuple3f &r0,     /* ray start                       */
             const tuple3f &rDir,   /* direction of ray                */
             const tuple3f &p,      /* point on plane                  */
             const tuple3f &n,      /* plane normal                    */
             tuple3f &intersect     /* intersection point              */){

  /*********************************************************
   Derivation:

   any point on the ray can be described as r0 + K*rDir

   since the intersection point lies on the plane,
   the vector (intersect - p) is perpendicular to the normal

   That gives two equations with two unknowns:
   intersect = r0 + K*rDir
   (intersect - p) dot n = 0

   Solve for K by substituting intersect:
   0 = (r0 + K*rDir - p) dot n
     = r0 dot n + K*rDir dot n - p dot n
   K*rDir dot n = p dot n - r0 dot n
   K*rDir dot n = (p - r0) dot n

   That yields this solution for K:
   K = (p - r0) dot n / rDir dot n

  *********************************************************/

  const float denom = dot(rDir, n);

  //check for line parallel to plane
  if(fabs(denom) < GEOMETRY_EPSILON)return -1;

  const float K = dot(p - r0, n)/denom;

  //check for intersection 'behind' the ray
  if(K < GEOMETRY_EPSILON)return -1;

  /* success */
  intersect = r0 + rDir * K;
  return 0;
}

  /* test if the point is inside the triangle
   * returns 1 if the point is inside
   * returns 0 if the point is outside
   * returns -1 if the point is on or near the edge
   */
static int pointInTriangle(const tuple3f &A,
                           const tuple3f &B,
                           const tuple3f &C,
                           const tuple3f &p){
  /*********************************************************
   Derivation:

   any point on the edge AB can be described as A + (B-A)*x
   any point on the edge AC can be described as A + (C-A)*y
   any point in the plane can be described as A + (B-A)*x + (C-A)*y

   p = A + (B-A)*x + (C-A)*y

   define pA as a vector from A to p:
   pA = p - A

   This can be described using x and y:
   pA = (B-A)*x + (C-A)*y

   now project pA onto AB by taking the dot product with B-A

   (pA) dot (B-A) = ((B-A)*x + (C-A)*y) dot (B-A)
   (pA) dot (B-A) = x * (B-A) dot (B-A) + y * (C-A) dot (B-A)

   also project p - A onto AC by taking the dot product with C-A

   (pA) dot (C-A) = ((B-A)*x + (C-A)*y) dot (C-A)
   (pA) dot (C-A) = x * (B-A) dot (C-A) + y * (C-A) dot (C-A)

   to clean things up, define
     dpb = (pA) dot (B-A)
     dpc = (pA) dot (C-A)
     dbb = (B-A) dot (B-A)
     dcb = (C-A) dot (B-A)
     dbc = (C-A) dot (B-A)  (note that this is the same as dcb)
     dcc = (C-A) dot (C-A)

   That gives two equations with two unknowns:
   dpb = x * dbb + y * dcb
   dpc = x * dbc + y * dcc

   Solve both equations for y:
   y * dcb = dpb - x * dbb  and  y * dcc = dpc - x * dbc
   y = (dpb - x * dbb)/dcb  and  y = (dpc - x * dbc)/dcc

   Now combine and solve for x:
     (dpb - x * dbb)/dcb = (dpc - x * dbc)/dcc
     (dpb - x * dbb)*dcc = (dpc - x * dbc)*dcb
   dpb*dcc - x * dbb*dcc = dpc*dcb - x * dbc*dcb
   x*(dbc*dcb - dbb*dcc) = dpc*dcb - dpb*dcc
   x*(dcb*dcb - dbb*dcc) = dpc*dcb - dpb*dcc  (since dbc=dcb)

   x = (dpc*dcb - dpb*dcc)/(dcb*dcb - dbb*dcc)

   Similarly for y, start by solving both equations for x:
   dpb = x * dbb + y * dcb  and  dpc = x * dbc + y * dcc
   x * dbb = dpb - y * dcb  and  x * dbc = dpc - y * dcc
   x = (dpb - y * dcb)/dbb  and  x = (dpc - y * dcc)/dbc

   Now combine and solve for y:
     (dpb - y * dcb)/dbb = (dpc - y * dcc)/dbc
     (dpb - y * dcb)*dbc = (dpc - y * dcc)*dbb
   dpb*dbc - y * dcb*dbc = dpc*dbb - y * dcc*dbb
   y*(dcc*dbb - dcb*dbc) = dpc*dbb - dpb*dbc
   y*(dcc*dbb - dcb*dcb) = dpc*dbb - dpb*dcb  (again, dbc=dcb)

   y = (dpc*dbb - dpb*dcb)/(dcc*dbb - dcb*dcb)

   Finally make the denominator match:
   x = (dpc*dcb - dpb*dcc)/(dcb*dcb - dbb*dcc)
   y = (dpb*dcb - dpc*dbb)/(dcb*dcb - dcc*dbb)

   denom = (dcb*dcb - dbb*dcc)
   x = (dpc*dcb - dpb*dcc)/denom
   y = (dpb*dcb - dpc*dbb)/denom


   any point described by (x,y) is
     inside triangle iff x,y>0 and x+y<1
     outside the triangle iff x or y<0 or x+y>1
     on the edge AB if y=0 and 0<=x<=1
     on the edge AC if x=0 and 0<=y<=1
     on the edge BC if x,y>=0 and x+y=1

  *********************************************************/

  const tuple3f pA = p - A;
  const tuple3f BA = B - A;
  const tuple3f CA = C - A;
  const float dpb = dot(pA, BA);
  const float dpc = dot(pA, CA);
  const float dbb = dot(BA, BA);
  const float dcb = dot(CA, BA);
  const float dcc = dot(CA, CA);

  const float denom = (dcb*dcb - dbb*dcc);
  const float x = (dpc*dcb - dpb*dcc)/denom;
  const float y = (dpb*dcb - dpc*dbb)/denom;

  //test if the point is inside
  if((x > GEOMETRY_EPSILON) && (y > GEOMETRY_EPSILON) &&
     ((x+y) < (1.0-GEOMETRY_EPSILON)))
    return 1;

  //test if the point is outside
  if((x < -GEOMETRY_EPSILON) || (y < -GEOMETRY_EPSILON) ||
     (x+y) > (1.0+GEOMETRY_EPSILON))
    return 0;

  //otherwise it is near the edge
  return -1;
}

int rayTriangle(const tuple3f &r0,  /* ray start                       */
                const tuple3f &rDir,/* direction of ray                */
                const tuple3f &a,   /* triangle vertex                 */
                const tuple3f &b,   /* triangle vertex                 */
                const tuple3f &c,   /* triangle vertex                 */
                const tuple3f &n,   /* triangle normal                 */
                tuple3f &intersect  /* intersection point              */){

  /* intersect with a plane first */
  if(-1 == rayPlane(r0, rDir, a, n, intersect))return -1;

  /* now check the intersection point, is it inside the triangle? */
  int code = pointInTriangle(a, b, c, intersect);
  if(1 == code)return 0;//inside
  if(0 == code)return -1;//outside
  return -1;//near the edge (should this be treated as inside or out?)
}

}/* end of rayCast namespace */

bool triangle::containsRay(const point &origin, const point &target,
			   point &intersect) const{
  /* make sure this is not a degenerate triangle */
  if(!isfinite(norm.x))return false;

  rayCast::tuple3f dir(target.x - origin.x,
		       target.y - origin.y,
		       target.z - origin.z);
  rayCast::tuple3f i;
  if(-1 == rayCast::rayTriangle(rayCast::tuple3f(origin.x, origin.y, origin.z),
				dir,
				rayCast::tuple3f(a.x, a.y, a.z),
				rayCast::tuple3f(b.x, b.y, b.z),
				rayCast::tuple3f(c.x, c.y, c.z),
				rayCast::tuple3f(norm.x, norm.y, norm.z),
				i))
    return false;

  intersect.x = i.x;
  intersect.y = i.y;
  intersect.z = i.z;
  return true;
}



bool triangle::containsLine(const point &origin, const point &target,
			    point &intersect) const{
#if 0
  printf("Checking if (%g,%g,%g),(%g,%g,%g),(%g,%g,%g) with normal (%g,%g,%g) contains line through (%g,%g,%g) and (%g,%g,%g)\n",
	 a.x,a.y,a.z,
	 b.x,b.y,b.z,
	 c.x,c.y,c.z,
	 norm.x, norm.y, norm.z,
	 origin.x,origin.y,origin.z,
	 target.x,target.y,target.z);
#endif

  PlaneFitMoments pm;
  pm.addPoint(a.x, a.y, a.z);
  pm.addPoint(b.x, b.y, b.z);
  pm.addPoint(c.x, c.y, c.z);
  Plane *p = pm.getPlane();

  point unitdir = target-origin;
  unitdir /= unitdir.norm();

  point unitTarget = origin + unitdir;

  point projectedOrigin = origin;
  point projectedTarget = unitTarget;
  projectedOrigin.z = p->getHeight(origin.x, origin.y);
  projectedTarget.z = p->getHeight(unitTarget.x, unitTarget.y);

  METERS dOrigin = sqrt(distSqBetween(origin, projectedOrigin));
  METERS dTarget = sqrt(distSqBetween(unitTarget, projectedTarget));

  if(origin.z < projectedOrigin.z)dOrigin = -dOrigin;
  if(unitTarget.z < projectedTarget.z)dTarget = -dTarget;

  /* parallel */
  if(fabs(dOrigin - dTarget)<1e-9)return false;
  
  METERS tTarget = dTarget/(dOrigin - dTarget);
  METERS tOrigin = 1 + tTarget;

  /* here's where the line intersects the plane of the triangle */
  intersect = origin + unitdir*tOrigin;

  /* now we need to determine if that point is inside the triangle
   * look at the three triangles formed by the corners and intersection
   * the point is interior of the normals all points of the same way
   */
  triangle ta(intersect, b, c), tb(intersect, c, a), tc(intersect, a, b);
  if(ta.norm.z > 0 && tb.norm.z > 0 && tc.norm.z > 0)return true;
  if(ta.norm.z < 0 && tb.norm.z < 0 && tc.norm.z < 0)return true;
  return false;
}

point triangle::nearestPoint(const point &p) const{
  PlaneFitMoments pm;
  pm.addPoint(a.x, a.y, a.z);
  pm.addPoint(b.x, b.y, b.z);
  pm.addPoint(c.x, c.y, c.z);
  Plane *plane = pm.getPlane();

  return plane->getPointNear(p);
}

METERS triangle::distTo(const point &p) const{
  return sqrt(distSqBetween(p, nearestPoint(p)));
}


void triangle::getHeight(point &p)const{
  PlaneFitMoments pm;
  pm.addPoint(a.x, a.y, a.z);
  pm.addPoint(b.x, b.y, b.z);
  pm.addPoint(c.x, c.y, c.z);
  Plane *pl = pm.getPlane();
  p.z = pl->getHeight(p.x, p.y);
}

void triangles::getHeight(point &p)const{
  for(unsigned int i=0;i<size();i++){
    if(get(i).contains2D(p)){
      get(i).getHeight(p);
      return;
    }
  }
}


point triangles::getPointNear(const point &p) const{
  int bestInd = 0;
  METERS bestDist = get(0).distTo(p);
  for(unsigned int i=1;i<size();i++){
    METERS thisDist = get(i).distTo(p);
    if(thisDist < bestDist){
      bestDist = thisDist;
      bestInd = i;
    }
  }
  return get(bestInd).nearestPoint(p);
}

static double inline min3(double a, double b, double c){
  if(a<b && a<c)return a;
  if(b<c)return b;
  return c;
}

static double inline max3(double a, double b, double c){
  if(a>b && a>c)return a;
  if(b>c)return b;
  return c;
}

void triangle::boundingBox(point &minPoint, point &maxPoint)const{
  minPoint.x = min3(a.x, b.x, c.x);
  minPoint.y = min3(a.y, b.y, c.y);
  minPoint.z = min3(a.z, b.z, c.z);

  maxPoint.x = max3(a.x, b.x, c.x);
  maxPoint.y = max3(a.y, b.y, c.y);
  maxPoint.z = max3(a.z, b.z, c.z);
}

void triangle::getLengthsSq(METERS l[3]) const{
  l[0] = distSqBetween(getLow(),  getMid());
  l[1] = distSqBetween(getMid(),  getHigh());
  l[2] = distSqBetween(getHigh(), getLow());
}

void triangle::getLengths(METERS l[3]) const{
  l[0] = distBetween(a, b);
  l[1] = distBetween(b, c);
  l[2] = distBetween(c, a);
}

METERS triangle::getPerimeter()const{
  return (distBetween(a, b) +
	  distBetween(b, c) +
	  distBetween(c, a) );
}


points triangles::findInteriorPoints(const points &p) const{
  points ret;
  for(unsigned int i=0;i<p.size();i++){
    point curPoint = p.get(i);
    point abovePoint = curPoint + point(0,0,1);
    for(unsigned int j=0;j<size();j++){
      point dummy(0,0,0);
      if(get(j).containsLine(curPoint, abovePoint, dummy))
	ret.add(curPoint);
    }
  }
  return ret;
}

points triangles::findExteriorPoints(const points &p) const{
  points ret;
  for(unsigned int i=0;i<p.size();i++){
    point curPoint = p.get(i);
    int exterior=1;
    for(unsigned int j=0;exterior && j<size();j++)
      if(get(j).contains2D(curPoint))
	exterior=0;
    if(exterior)ret.add(curPoint);
  }
  return ret;
}

transformMatrix::transformMatrix(){
  /* initialize to a 4x4 identity matrix */
  setIdentity();
}

void transformMatrix::setIdentity(){
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++)mat[i][j] = 0.0;
    mat[i][i] = 1.0;
  }
}

transformMatrix::transformMatrix(const point &p){
  setIdentity();
  mat[0][3] = p.x;
  mat[1][3] = p.y;
  mat[2][3] = p.z;
}

transformMatrix::transformMatrix(const orient &o){
  transformMatrix r = transformMatrix::rollMatrix(o.roll);
  transformMatrix p = transformMatrix::pitchMatrix(o.pitch);
  transformMatrix y = transformMatrix::yawMatrix(o.yaw);
  copyFrom(y*p*r);
}

void transformMatrix::copyFrom(const transformMatrix *t){
  for(int r=0;r<4;r++)
    for(int c=0;c<4;c++)
      mat[r][c] = t->mat[r][c];
}

void transformMatrix::applyToPoint(point &p) const {
  METERS x = mat[0][0]*p.x + mat[0][1]*p.y + mat[0][2]*p.z + mat[0][3];
  METERS y = mat[1][0]*p.x + mat[1][1]*p.y + mat[1][2]*p.z + mat[1][3];
  METERS z = mat[2][0]*p.x + mat[2][1]*p.y + mat[2][2]*p.z + mat[2][3];
  p.x = x;
  p.y = y;
  p.z = z;
}

point transformMatrix::getCenter() const{
  return point(mat[0][3], mat[1][3], mat[2][3]);
}

orient transformMatrix::getAngles() const{
  RADIANS pitch = asin(mat[2][1]);
  RADIANS yaw = asin(-mat[0][1] / cos(pitch));
  RADIANS roll = asin(-mat[2][0] / cos(pitch));
  return orient(roll, pitch, yaw);
}

transformMatrix transformMatrix::rollMatrix(RADIANS angle){
  transformMatrix ret;
  ret.mat[0][0] = cos(-angle);
  ret.mat[0][2] = -sin(-angle);
  ret.mat[2][0] = sin(-angle);
  ret.mat[2][2] = cos(-angle);
  return ret;
}

transformMatrix transformMatrix::pitchMatrix(RADIANS angle){
  transformMatrix ret;
  ret.mat[1][1] = cos(-angle);
  ret.mat[1][2] = sin(-angle);
  ret.mat[2][1] = -sin(-angle);
  ret.mat[2][2] = cos(-angle);
  return ret;
}

transformMatrix transformMatrix::yawMatrix(RADIANS angle){
  transformMatrix ret;
  ret.mat[0][0] =  cos(-angle);
  ret.mat[0][1] =  sin(-angle);
  ret.mat[1][0] = -sin(-angle);
  ret.mat[1][1] =  cos(-angle);
  return ret;
}

void transformMatrix::multiplyBy(const transformMatrix *t){
  double result[4][4];
  for(int r=0;r<4;r++){
    for(int c=0;c<4;c++){
      result[r][c] = 0.0;
      for(int i=0;i<4;i++)
	result[r][c] += mat[r][i]*t->mat[i][c];
    }
  }
  for(int r=0;r<4;r++){
    for(int c=0;c<4;c++){
      mat[r][c] = result[r][c];
    }
  }
}

transformMatrix transformMatrix::multiply (const transformMatrix &t) const{
  transformMatrix ret;
  for(int r=0;r<4;r++){
    for(int c=0;c<4;c++){
      ret.mat[r][c] = 0.0;
      for(int i=0;i<4;i++)
	ret.mat[r][c] += mat[r][i]*t.mat[i][c];
    }
  }
  return ret;
}

void transformMatrix::print(const char *prefix) const{
  for(int r=0;r<4;r++){
    printf("%s", prefix);
    for(int c=0;c<4;c++){
      printf("%9g ", mat[r][c]);
    }
    printf("\n");
  }
}


void rotation::calcMatrix(){
  transformMatrix *t = new transformMatrix(orient(roll, pitch, yaw));
  mat.copyFrom(t);
  delete t;
}

void rotation::adjust(rotation delta){
  roll  += delta.roll;
  pitch += delta.pitch;
  yaw   += delta.yaw;
  calcMatrix();
  //printf("Rotation adjustment: %g, %g, %g radians\n", roll, pitch, yaw);
}

void rotation::print(const char *prefix)const{
  char buf[256];
  sprintf(buf, "%s  ", prefix);
  printf("%sRotation: %g, %g, %g with matrix:\n", prefix, roll, pitch, yaw);
  mat.print(buf);
}

void translation::calcMatrix(){
  transformMatrix *t = new transformMatrix(point(x, y, z));
  mat.copyFrom(t);
  delete t;
}

void translation::adjust(translation delta){
  x += delta.x;
  y += delta.y;
  z += delta.z;
  calcMatrix();
  //printf("Translation adjustment: %g, %g, %g radians\n", x, y, z);
}

void translation::print(const char *prefix)const{
  char buf[256];
  sprintf(buf, "%s  ", prefix);
  printf("%sTranslation: %g, %g, %g with matrix:\n", prefix, x, y, z);
  mat.print(buf);
}

void composite::copyFrom(const composite *copy){
  assert(copy);
  assert(copy->a);
  assert(copy->b);
  a = copy->a->copy();
  b = copy->b->copy();
  mat.copyFrom(&(copy->mat));
}

composite::composite(const composite &copy)
  :a(NULL),b(NULL){
  copyFrom(&copy);
}

composite::composite(const composite *copy)
  :a(NULL),b(NULL){
  copyFrom(copy);
}

void composite::calcMatrix(){
  mat.copyFrom(b->mat * a->mat);
}

void composite::apply()const{
  b->apply();
  a->apply();
}
void composite::undo()const{
  a->apply();
  b->apply();
}

void composite::print(const char *prefix)const{
  char buf[256];
  sprintf(buf, "%s  ", prefix);

  printf("%sComposite with A=\n", prefix);
  a->print(buf);
  printf("%sComposite with B=\n", prefix);
  b->print(buf);
  printf("%sComposite with matrix=\n", prefix);
  mat.print(buf);
  
}


const double INPUT_RESOLUTION = 0.00001;
static double inline toResolution(double input){
  return INPUT_RESOLUTION * round(input/INPUT_RESOLUTION);
}


points::points(const char *filename, unsigned int subsample){
  FILE *f = stdin;
  if(filename){
    printf("Reading from %s\n", filename);
    f = fopen(filename, "r");
  }

  if(!f){
    printf("Unable to open \"%s\", returning empty set of points\n", filename);
    return;
  }
  assert(subsample>0);

  METERS x, y, z;
  int i=0;
  clearerr(f);
  char buf[128];
  while(fgets(buf, sizeof(buf), f)){
    char *ptr = buf;
    if('v' == buf[0] && ' ' == buf[1])ptr+=2;
    if(3 != sscanf(ptr, "%lg %lg %lg", &x, &y, &z))continue;
    x = toResolution(x);
    y = toResolution(y);
    z = toResolution(z);
    //if(x>4)continue;/* hack for area of bad calibration */
    if(0 == (i++)%subsample)add(point(x, y, z));
  }

  printf("Done after %d lines, eof? %d\n", i, feof(f));

  if(filename)
    fclose(f);
  /*
  printf("Created new collection of %d (x%d) points from \"%s\"\n", 
	 size(), subsample, file);
  */
}

void points::applyTransform(const transformation *t){
  for(unsigned int i = 0;i<size();i++)t->applyToPoint(data[i]);
}
points points::copy() const{
  points ret;
  for(unsigned int i = 0;i<size();i++)ret.add(get(i));
  return ret;
}

point points::mean() const{
  point ret(0,0,0);
  if(size()<=0)return ret;
  for(unsigned int i=0;i<size();i++)
    ret += get(i);
  return (ret / ((double)size()));
}

void points::writeToFile(const char *filename) const{
  FILE *f = stdout;
  if(filename)
    f = fopen(filename, "w");

  if(!f){
    printf("Error, unable to open \"%s\" for writing\n", filename);
    exit(0);
  }

  for(unsigned int i=0;i<size();i++)
    fprintf(f, "%g %g %g\n", get(i).x, get(i).y, get(i).z);

  if(filename)
    fclose(f);
}

points points::subsample(unsigned int factor) const{
  points ret;
  for(unsigned int i=0;i<size();i+=factor)
    ret.add(get(i));
  return ret;
}

void points::boundingBox(point &minPoint, point &maxPoint){
  if(size()<=0){
    minPoint.x = minPoint.y = minPoint.z = 0;
    maxPoint.x = maxPoint.y = maxPoint.z = 0;
    return;
  }

  maxPoint = minPoint = get(0);
  for(unsigned int i=1;i<size();i++){
    if(get(i).x < minPoint.x)minPoint.x = get(i).x;
    if(get(i).y < minPoint.y)minPoint.y = get(i).y;
    if(get(i).z < minPoint.z)minPoint.z = get(i).z;
    if(get(i).x > maxPoint.x)maxPoint.x = get(i).x;
    if(get(i).y > maxPoint.y)maxPoint.y = get(i).y;
    if(get(i).z > maxPoint.z)maxPoint.z = get(i).z;
  }
}

double getError(points a, points b){
  assert(a.size() == b.size());
  if(a.size()<=0)return 0.0;
  point sumdiffsq(0,0,0);
  for(unsigned int i=0;i<a.size();i++)
    sumdiffsq += (a.get(i) - b.get(i)).square();
  return sumdiffsq.norm();
}

static bool pointWithin(const points &model, const point &p, METERS nearDist){
  for(unsigned int i=0;i<model.size();i++)
    if(distSqBetween(model.get(i), p) < nearDist)return true;
  return false;
}

/* returns the points from the scene that are in the model */
points findInsidePoints(const points &model, const points &scene,
			METERS nearDist){
  points ret;
  for(unsigned int i=0;i<scene.size();i++){
    if(pointWithin(model, scene.get(i), nearDist))
      ret.add(scene.get(i));
  }
  return ret;
}

#if 0
poses::poses(const char *file, unsigned int subsample){
  FILE *f = fopen(file, "r");
  if(!f){
    printf("Unable to open \"%s\", returning empty set of poses\n", file);
    return;
  }
  assert(subsample>0);

  double lasttime = 0.0;

  double datatime=0;
  double velocity=0;
  METERS x, y, z=0;
  RADIANS roll, pitch, yaw;
  RADIANS frontAxleYaw, rearAxleYaw, frontAxleRoll, rearAxleRoll;
  int i=0;
  int linenumber=0;
  while(f&&(12==fscanf(f, "%lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg %lg", 
		       &datatime, &x, &y, &z, &roll, &pitch, &yaw, 
		       &frontAxleYaw, &rearAxleYaw, 
		       &frontAxleRoll, &rearAxleRoll, &velocity))){
    x = toResolution(x);
    y = toResolution(y);
    z = toResolution(z);
    roll  = toResolution(roll);
    pitch = toResolution(pitch);
    yaw   = toResolution(yaw);
    frontAxleYaw  = toResolution(frontAxleYaw);
    rearAxleYaw   = toResolution(rearAxleYaw);
    frontAxleRoll = toResolution(frontAxleRoll);
    rearAxleRoll  = toResolution(rearAxleRoll);
    velocity = toResolution(velocity);

    linenumber++;
    if(datatime < lasttime){
      printf("bad data on line %d\n", linenumber);
      continue;
    }
    lasttime=datatime;
    if(0 == (i++)%subsample){
      point p(x, y, z);
      orient o(roll, pitch, yaw);
      pose mypose(p, o, frontAxleYaw, frontAxleRoll, rearAxleYaw, rearAxleRoll, velocity, datatime);
      assert(mypose.transformToGlobal);
      add(mypose);
    }
  }

  fclose(f);
  printf("Created new collection of %d (x%d) poses from \"%s\"\n", 
	 size(), subsample, file);
}

void poses::getMeanAxleRolls(RADIANS &front, RADIANS &rear)const{
  double totaldist = 0.0;
  const double maxdist = 10;
  front = rear = 0.0;
  for(unsigned int i=1;i<size();i++){
    double dist=fabs(get(i).velocity) * (get(i).datatime - get(i-1).datatime);
    if((dist<=0.0) || !isfinite(dist) || (dist>=maxdist)){
      //printf("Ignoring item %d, bad dist %g (dt: %g, vel: %g)\n", i, dist, dt, get(i).velocity);
      continue;
    }
    front += get(i).frontAxleRoll * dist;
    rear  += get(i).rearAxleRoll  * dist;
    totaldist += dist;
  }

  if(totaldist == 0)return;
  assert(isfinite(totaldist));
  front /= totaldist;
  rear  /= totaldist;
}

void poses::getMeanAxleYaws(RADIANS &front, RADIANS &rear)const{
  double totaldist = 0.0;
  const double maxdist = 10;
  front = rear = 0.0;
  for(unsigned int i=1;i<size();i++){
    double dist=fabs(get(i).velocity) * (get(i).datatime - get(i-1).datatime);
    if((dist<=0.0) || !isfinite(dist) || (dist>=maxdist)){
      //printf("Ignoring item %d, bad dist %g (dt: %g, vel: %g)\n", i, dist, dt, get(i).velocity);
      continue;
    }
    front += get(i).frontAxleYaw * dist;
    rear  += get(i).rearAxleYaw  * dist;
    totaldist += dist;
  }

  if(totaldist == 0)return;
  assert(isfinite(totaldist));
  front /= totaldist;
  rear  /= totaldist;
}

orient poses::getMeanOrientation()const{
  orient ret(0,0,0);
  double totaldist = 0.0;
  const double maxdist = 10;
  for(unsigned int i=1;i<size();i++){
    double dt = (float)(get(i).datatime - get(i-1).datatime);
    if(dt<=0){
      printf("Bad time at index %d (%g - %g = %g)\n", i, get(i).datatime, get(i-1).datatime, dt);
      exit(0);
    }
    double dist = fabs(get(i).velocity) * dt;
    if((dist<=0.0) || !isfinite(dist) || (dist>=maxdist)){
      //printf("Ignoring item %d, bad dist %g (dt: %g, vel: %g)\n", i, dist, dt, get(i).velocity);
      continue;
    }
    ret.roll  += (get(i).orientation.roll  * dist);
    ret.pitch += (get(i).orientation.pitch * dist);
    ret.yaw   += (get(i).orientation.yaw   * dist);
    totaldist += dist;
    if(!isfinite(totaldist) || !isfinite(ret.roll) || !isfinite(ret.pitch) || !isfinite(ret.yaw)){
      printf("getMeanOrientation failed on %d of %d (dist %f)\n", i, size(), dist);
      exit(0);
    }
  }

  if(totaldist == 0)return ret;
  assert(isfinite(totaldist));
  printf("Hack alert, artificially doubling distance travelled\n");
  totaldist *= 2.0;
  ret.roll  /= totaldist;
  ret.pitch /= totaldist;
  ret.yaw   /= totaldist;
  if(!isfinite(ret.roll) || !isfinite(ret.pitch) || !isfinite(ret.yaw)){
    printf("getMeanOrientation failed at normalization (total distance %f)\n", totaldist);
    exit(0);
  }
  return ret;
}

double poses::getTotalDistance()const{
  double totaldist = 0.0;
  for(unsigned int i=1;i<size();i++)
    totaldist += fabs(get(i).velocity) * (get(i).datatime - get(i-1).datatime);
  return totaldist;
}

double poses::getTotalTime()const{
  if(size()<=1)return 0.0;
  return get(size()-1).datatime - get(0).datatime;
}
#endif

transformation *transformStack::getTransform()const{
  if(size() == 0)return NULL;

  transformation *ret = get(0);
  for(unsigned int i=1;i<size();i++)
    ret = new composite(ret, get(i));
  return ret;
}

char *toBinName(const char *file){
  assert(file);
  const unsigned int LEN = 1024;//max length of path for symlinks
  assert(strlen(file)<LEN);

  char buf[LEN];
  int bytes = readlink(file, buf, sizeof(buf));
  if(bytes>0){
    assert(bytes<(int)(sizeof(buf)-1));
    buf[bytes] = '\0';
    //printf("Following symlink \"%s\" to \"%s\"\n", file, buf);

    if(buf[0] != '/'){
      /* relative link, replace the basename with the link contents */
      char buf2[2*LEN];
      strcpy(buf2, file);

      char *p = rindex(buf2, '/');
      p = (p?p+1:buf2);
      strcpy(p, buf);  
      //printf("relative symlink turned into \"%s\"\n", buf2);
      return toBinName(buf2);
    }

    /* absolute link, follow it normally */
    return toBinName(buf);
  }

  int len = strlen(file);
  char *ret = (char *)malloc((6+len)*sizeof(char));
  assert(ret);

  memcpy(ret, file, len+1);

  /* find where the filename starts (eg basename),
     either after the last / or at the beginning
  */
  const char *p = rindex(file, '/');
  p = (p?p+1:file);

  /* replace FILE with .FILE.bin */
  sprintf(ret + (p-file), ".%s.bin", p);

  return ret;
}

