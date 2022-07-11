/*
 * Dominic Jonak (dom@cmu.edu)
 *
 * Helper for rendering a triangle as a set of contour levels.
 * Expected to be used to render an entire mesh as a contour map.
 */
#include "helpers.h"
#include "solidContours.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

static int DEBUG = 0;

/* rounds the value down to the nearest contour level
   contours are spaced every n meters
 */
static inline METERS myFloor(METERS a, METERS n){
  METERS modval = -fmod(a,n);
  if(a<0) modval -= n;
  else if(fabs(modval+n)<1e-9) modval=0;
  METERS ret = a + modval;
  //printf("myFloor(%g, %g) -> %g -> %g\n", a, n, modval, ret);
  return ret;
}

/* Given two altitudes and contour spacing
   Compute the lowest and highest altitudes that need to be rendered
 */
static void getMinMax(METERS minz, METERS maxz, METERS spacing,
		      METERS &minspace, METERS &maxspace){
  /*
  myFloor(0, 0.1);
  myFloor(-0.0001, 0.1);
  myFloor(0.0001, 0.1);
  myFloor(1.0001, 0.1);
  myFloor(0.9999, 0.1);
  myFloor(1.001, 0.1);
  myFloor(1.999, 0.1);
  myFloor(1.499, 0.1);
  myFloor(0.01, 0.1);
  myFloor(-1, 0.1);
  myFloor(1, 0.1);
  myFloor(-1.04, 0.1);
  myFloor(1.04, 0.1);
  exit(0);
  */

  assert(minz<=maxz);

  minspace = myFloor(minz, spacing);
  maxspace = myFloor(maxz, spacing);
  assert(minspace <= minz);
  assert(maxspace <= maxz);
}

/* Given a line defined by two points
   Return the point on the line at the specified altitude
   In the case of horizontal lines, returns one of the two points
 */
static point getPointAt(METERS z, const point &a, const point &b){
  double t = (z-a.z)/(b.z-a.z);
  if(!isfinite(t))t=0;
  return point(a.x + t*(b.x-a.x), a.y + t*(b.y-a.y), z);
}

/* given a color percentage,
 * sets the current color using a rainbow palette
 */
static void heightToColorRainbow(double percent){
  /*
  0/6 to 1/6  100 to 110
  1/6 to 2/6  110 to 010
  2/6 to 3/6  010 to 011
  3/6 to 4/6  011 to 001
  4/6 to 5/6  001 to 101
  5/6 to 6/6  101 to 100
  */
  percent *= 6.0;
  if(percent<1.0)
    glColor4f(1.0, fmod(percent, 1.0), 0.0, 1.0);
  else if(percent<2.0)
    glColor4f(1.0-fmod(percent, 1.0), 1.0, 0.0, 1.0);
  else if(percent<3.0)
    glColor4f(0.0, 1.0, fmod(percent, 1.0), 1.0);
  else if(percent<4.0)
    glColor4f(0.0, 1.0-fmod(percent, 1.0), 0.0, 1.0);
  else if(percent<5.0)
    glColor4f(fmod(percent, 1.0), 0.0, 1.0, 1.0);
  else
    glColor4f(1.0, 0.0, 1.0-fmod(percent, 1.0), 1.0);
}

/* given a color percentage,
 * sets the current color using a greyscale palette
 */
static void heightToColorGrey(double percent){
  if(percent>0.95 || percent<0.05)
    glColor4f(1.0, 0, 0, 1.0);
  else
    glColor4f(percent, percent, percent, 1.0);
}

/* given an altitude sets the current color (with glColor) */
static inline void heightToColor(METERS z,
				 METERS colorRepeat, int rainbow){
  /* convert the altitude to a percentage */
  double percent = fmod(z/colorRepeat, 1.0);
  if(percent<0.0)percent+=1.0;
  if(DEBUG){
    assert(percent>=0.0 && percent <= 1.0);
  }

  if(rainbow)
    heightToColorRainbow(percent);
  else
    heightToColorGrey(percent);
}

/* draws a vertical rectangle */
static void drawVertical(const point &a, const point &b, 
			 const point &c, const point &d,
			 METERS colorRepeat, int rainbow){
  if(DEBUG){
    printf("drawVertical(");
    a.print("(",")");
    b.print("(",")");
    c.print("(",")");
    d.print("(",")");
    printf(")\n");
  }

  glBegin(GL_TRIANGLE_STRIP);
  if(fabs(b.x-a.x)<0.01)
    glNormal3f(1,0,0);
  else if(fabs(b.y-a.y)<0.01)
    glNormal3f(0,1,0);
  else
    glNormal3f(-1, -(b.x-a.x)/(b.y-a.y), 0);

  assert(fabs(a.z-b.z)<0.01);
  assert(fabs(c.z-d.z)<0.01);

  heightToColor(a.z, colorRepeat, rainbow);
  a.glVertex();
  b.glVertex();
  c.glVertex();
  d.glVertex();
  glEnd();
}

/* draws a horizontal trapazoid at the given altitude */
static void drawHorizontal(const point &a, const point &b, const point &c, 
			   const point &d, METERS height,
			   METERS colorRepeat, int rainbow){
  if(DEBUG){
    printf("Drawing horizontal4:");
    a.print(" a=","");
    b.print(" b=","");
    c.print(" c=","");
    d.print(" d=","");
    printf("\n");
  }

  glBegin(GL_TRIANGLE_STRIP);
  glNormal3f(0,0,1);
  heightToColor(height, colorRepeat, rainbow);
  glVertex3f(a.x, a.y, height);
  glVertex3f(b.x, b.y, height);
  glVertex3f(c.x, c.y, height);
  glVertex3f(d.x, d.y, height);
  glEnd();
}

/* draws a horizontal triangle at the given altitude */
static void drawHorizontal(const point &a, const point &b, const point &c,
			   METERS height,
			   METERS colorRepeat, int rainbow){
  if(DEBUG){
    printf("Drawing horizontal3 at height %f:", height);
    a.print(" a ","");
    b.print(" b ","");
    c.print(" c ","");
    printf("\n");
  }

  glBegin(GL_TRIANGLE_STRIP);
  glNormal3f(0,0,1);
  heightToColor(height, colorRepeat, rainbow);
  glVertex3f(a.x, a.y, height);
  glVertex3f(b.x, b.y, height);
  glVertex3f(c.x, c.y, height);
  glEnd();
}

/* given the three vertices of a triangle (in order of ascending altitude)
   draws a single contour at the specified level
   the contour consists of a vertical component
   and usually a horizontal component
   both components and the triangle contain a line at the specified altitude
 */
#define EPSILON 1e-4
static void drawSolidContourAt(const point &a, const point &b, const point &c, 
			       METERS z, METERS spacing,
			       METERS colorRepeat, int rainbow){

  point lone=c;
  point pairedA=a;
  point pairedB=b;
  int upward=1;
  int verticalOnly = !(c.z + EPSILON >= z);
  if((b.z+EPSILON)>z || verticalOnly){
    lone=a;
    upward=0;
    pairedA=c;
  }

  if(DEBUG){
    printf("drawSolidContourAt(%0.3f) %0.3f,%0.3f,%0.3f (spacing %0.3f, upward %d vert %d) ", 
	   z, a.z, b.z, c.z, spacing, upward, verticalOnly);
    printf("z={%0.3f} vs z={%0.3f, %0.3f}\n",
	   lone.z, pairedA.z, pairedB.z);
  }

  if(verticalOnly){
    z=c.z;
  }

  assert(a.z <= b.z && b.z <= c.z);
  assert(a.z - EPSILON <= z);
  assert(c.z + EPSILON >= z);

  point p1 = getPointAt(z, lone, pairedA);
  point p2 = getPointAt(z, lone, pairedB);
  point p3(p1.x, p1.y, z-spacing);
  point p4(p2.x, p2.y, z-spacing);
  point p5(pairedA.x, pairedA.y, (upward)?(z-spacing):z);
  point p6(pairedB.x, pairedB.y, (upward)?(z-spacing):z);
  point p7(lone.x,    lone.y,   (!upward)?(z-spacing):z);

  drawVertical(p1, p2, p3, p4, colorRepeat, rainbow);
  if(!verticalOnly){
    if(upward)
      drawHorizontal(p7, p1, p2, p7.z, colorRepeat, rainbow);
    else
      drawHorizontal(p1, p2, p5, p6, p5.z, colorRepeat, rainbow);
  }
}

/* this is the exposed function
   given the three vertices of a triangle
   contours are drawn underneath the triangle with the given spacing
   draws a single base contour underneath the entire triangle,
   then calls drawSolidContourAt for each contour stacked above
 */
void drawSolidContour(const point &a, const point &b, const point &c,
		      METERS spacing, METERS colorRepeat, int rainbow){
  if(DEBUG){
    printf("\n\nDrawSolidContour(");
    a.print("low=","");
    b.print(" mid=","");
    c.print(" high=","");
    printf(" spacing=%0.3f)\n", spacing);
  }

  /* a is lower than b, is lower than c */
#if 0
  if(b.z<a.z){drawSolidContour(b, a, c, spacing);return;}
  if(c.z<a.z){drawSolidContour(c, b, a, spacing);return;}
  if(c.z<b.z){drawSolidContour(a, c, b, spacing);return;}
#endif
  assert(a.z <= b.z && b.z <= c.z);

  METERS minZ=0.0, maxZ = 0.0;
  getMinMax(a.z, c.z, spacing, minZ, maxZ);

  /* first draw the base */
  if(DEBUG){
    printf("Drawing base at %f\n", minZ);
  }
  drawHorizontal(a, b, c, minZ, colorRepeat, rainbow);

  if(DEBUG){
    printf("There are contours between %0.3f and %0.3f: %f:%f:%f\n",
	   a.z, c.z, minZ, spacing, maxZ);
  }

  for(METERS z = minZ+spacing; z < maxZ+(spacing/2.0); z += spacing)
    drawSolidContourAt(a, b, c, z, spacing, colorRepeat, rainbow);

  if(DEBUG){
    glDisable(GL_LIGHTING);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    a.print("low:  ","\n");
    b.print("mid:  ","\n");
    c.print("high: ","\n");
    glBegin(GL_TRIANGLE_STRIP);
    a.glVertex();
    b.glVertex();
    c.glVertex();
    glEnd();
    glBegin(GL_LINE_STRIP);
    a.glVertex();
    b.glVertex();
    c.glVertex();
    a.glVertex();
    glEnd();
  }

  if(DEBUG){
    DEBUG--;
  }
}
