/**
 * @file rasm_common_types.h
 *
 * @section LICENSE
 * Copyright 2012, Carnegie Mellon University and ProtoInnovations, LLC. 
 * All rights reserved. This document contains information that is proprietary
 * to Carnegie Mellon University and ProtoInnovations, LLC. Do not distribute
 * without approval.
 *
 */

/*!
 * 12/10/2020:
 *
 * This file contains definitions for:
 * - RASM_UNITS
 * - point3d
 * - point2d
 * - DriveCmd
 * - bounds3d
 * - bounds2d
 * - triangle
 * - edge
 * - mesh
 * - goal
 * - matrix operations
 */

#ifndef RASM_TYPES_H
#define RASM_TYPES_H

#include <stdio.h>
#include <math.h>
#include "rotation_matrix.h"

#ifdef USE_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif // USE_PCL

/*** set what kind of units you want, ints (default) or floats? ***/
#ifndef RASM_TYPE_INT
#define RASM_TYPE_INT 1
#endif

#if RASM_TYPE_INT  /*** integer arithmetic ***/

typedef int RASM_UNITS;
#define METERS_TO_RASM(x) ((RASM_UNITS)round((x)*1000.0))
#define RASM_TO_METERS(x) (((double)(x))/1000.0)
#define RASM_UNITS_FORMAT "%d"
#define RASM_EPSILON 1
bool inline RASM_UNITS_equals(const RASM_UNITS &a, const RASM_UNITS &b){return (a==b);}

#else  /*** floating point arithmetic ***/

typedef float RASM_UNITS;
#define METERS_TO_RASM(x) ((RASM_UNITS)(x))
#define RASM_TO_METERS(x) ((double)(x))
#define RASM_UNITS_FORMAT "%f"
#define RASM_EPSILON 1e-6
bool inline RASM_UNITS_equals(const RASM_UNITS &a, const RASM_UNITS &b)
  {return ((a-b)<RASM_EPSILON && (b-a)<RASM_EPSILON);}

#endif

/*** shouldn't need to change anything below here ***/

namespace RASM {

  class DriveCmd
  {
  public:
    double m_radius_meters;
    double m_yaw_radians;
    double m_speed_meters_per_sec;
    double m_timeout_sec;
    double m_timestamp;
    DriveCmd() 
      {m_radius_meters = m_yaw_radians = m_speed_meters_per_sec = m_timeout_sec = m_timestamp;}

    DriveCmd(double radius,
	     double yaw,
	     double speed,
	     double timeout,
	     double timestamp)
      {
	m_radius_meters        = radius;
	m_yaw_radians          = yaw;
	m_speed_meters_per_sec = speed;
	m_timeout_sec          = timeout;
	m_timestamp            = timestamp;
      }

    DriveCmd(const DriveCmd &cmd)
      {
	m_radius_meters        = cmd.m_radius_meters;
	m_yaw_radians          = cmd.m_yaw_radians;
	m_speed_meters_per_sec = cmd.m_speed_meters_per_sec;
	m_timeout_sec          = cmd.m_timeout_sec;
	m_timestamp            = cmd.m_timestamp;
      }
  };

  class point3d{
  public:
    RASM_UNITS coord3d[3]; /* the x, y and z coordinates of this point */

    /* constructors */
    point3d()
      {coord3d[0]=coord3d[1]=coord3d[2]=(RASM_UNITS)0;}

    point3d(RASM_UNITS x, RASM_UNITS y, RASM_UNITS z)
      {coord3d[0]=x;coord3d[1]=y;coord3d[2]=z;}

    point3d(const point3d &point){
      coord3d[0]=point.coord3d[0];
      coord3d[1]=point.coord3d[1];
      coord3d[2]=point.coord3d[2];
    }

#ifdef USE_PCL
    point3d(const pcl::PointXYZ &point){
      coord3d[0]=METERS_TO_RASM(point.x);
      coord3d[1]=METERS_TO_RASM(point.y);
      coord3d[2]=METERS_TO_RASM(point.z);
    }
#endif //USE_PCL

    void print(FILE *output = stdout) const{
      fprintf(output,
	      RASM_UNITS_FORMAT " " RASM_UNITS_FORMAT " " RASM_UNITS_FORMAT,
	      coord3d[0], coord3d[1], coord3d[2]);
    }


    void generateTransformationMatrix(float M[4][4], 
				      const RASM::point3d &p,
				      double roll,
				      double pitch, 
				      double yaw)
    {
      /* initialize and fill in the rotation */
      rotationMatrix(roll, pitch, yaw, M);
      
      /* fill in the translation */
      for(int i=0;i<3;i++)
	{
	  M[i][3] = p.coord3d[i];
	}
    }

    void transform(const RASM::point3d &amount, 
		   double roll, double pitch, double yaw)
    {
      float M[4][4]; 
      generateTransformationMatrix(M, amount, roll, pitch, yaw);
      transform(M);
    }

    void transform(const float M[4][4])
    {
      RASM::point3d tmp; 
      tmp.set(X(), Y(), Z());
      for(int i=0;i<3;i++)
	{
	  tmp.coord3d[i] = (RASM_UNITS)(M[i][0]*(float)X() + M[i][1]*(float)Y() + M[i][2]*(float)Z() + M[i][3]);
	}
      set(tmp.X(), tmp.Y(), tmp.Z());
    }

    /* overloaded operators */
    inline point3d & operator += ( const point3d &p ){
      for(int i=0;i<3;i++)
	coord3d[i]+=p.coord3d[i];
      return *this;
    }

    inline point3d & operator -= ( const point3d &p ){
      for(int i=0;i<3;i++)
	coord3d[i]-=p.coord3d[i];
      return *this;
    }

    inline point3d & operator *= ( RASM_UNITS u ){
      for(int i=0;i<3;i++)
	coord3d[i]*=u;
      return *this;
    }

    inline point3d & operator /= ( RASM_UNITS u ){
      for(int i=0;i<3;i++)
	coord3d[i]/=u;
      return *this;
    }

    inline point3d & operator *= ( double d ){
      for(int i=0;i<3;i++)
	coord3d[i] = (RASM_UNITS)(((double)(coord3d[i]))*d);
      return *this;
    }

    inline point3d & operator /= ( double d ){
      for(int i=0;i<3;i++)
	coord3d[i] = (RASM_UNITS)(((double)(coord3d[i]))/d);
      return *this;
    }

    /* handy accessors, makes the code more readable */
    RASM_UNITS inline X() const {return coord3d[0];}
    RASM_UNITS inline Y() const {return coord3d[1];}
    RASM_UNITS inline Z() const {return coord3d[2];}

    void set(RASM_UNITS x, RASM_UNITS y, RASM_UNITS z)
      {coord3d[0]=x; coord3d[1]=y; coord3d[2]=z;}

    bool inline equals(const point3d &point) const{
      return (RASM_UNITS_equals(coord3d[0], point.coord3d[0]) &&
	      RASM_UNITS_equals(coord3d[1], point.coord3d[1]) &&
	      RASM_UNITS_equals(coord3d[2], point.coord3d[2]) );
    }
    bool inline operator == ( const point3d &p ) const {return equals(p);}

#ifdef USE_PCL
    operator pcl::PointXYZ () const {
      pcl::PointXYZ newpt;
      newpt.x = RASM_TO_METERS(coord3d[0]);
      newpt.y = RASM_TO_METERS(coord3d[1]);
      newpt.z = RASM_TO_METERS(coord3d[2]);
      return newpt;
    }
    
#endif // USE_PCL

  }; /* end of class point3d */

  class point2d{
 public:
    RASM_UNITS coord2d[2]; /* the x, y coordinates of this point */

    /* constructors */
    point2d()
      {coord2d[0]=coord2d[1]=(RASM_UNITS)0;}

    point2d(RASM_UNITS x, RASM_UNITS y)
      {coord2d[0]=x;coord2d[1]=y;}

    point2d(const point2d &point)
      {coord2d[0]=point.coord2d[0];coord2d[1]=point.coord2d[1];}

    point2d(const point3d &point) /* handy for casting from 3d to 2d */
      {coord2d[0]=point.coord3d[0];coord2d[1]=point.coord3d[1];}

    void print(FILE *output=stdout) const{
      fprintf(output,
	      RASM_UNITS_FORMAT " " RASM_UNITS_FORMAT,
	      coord2d[0], coord2d[1]);
    }

    void set(RASM_UNITS x, RASM_UNITS y){coord2d[0]=x; coord2d[1]=y;}

    inline point2d & operator += ( const point2d &p ){
      for(int i=0;i<2;i++)
	coord2d[i]+=p.coord2d[i];
      return *this;
    }

    inline point2d & operator -= ( const point2d &p ){
      for(int i=0;i<2;i++)
	coord2d[i]-=p.coord2d[i];
      return *this;
    }

    inline point2d & operator *= ( RASM_UNITS u ){
      for(int i=0;i<2;i++)
	coord2d[i]*=u;
      return *this;
    }

    inline point2d & operator /= ( RASM_UNITS u ){
      for(int i=0;i<2;i++)
	coord2d[i]/=u;
      return *this;
    }

    inline point2d & operator *= ( double d ){
      for(int i=0;i<2;i++)
	coord2d[i] = (RASM_UNITS)(((double)(coord2d[i]))*d);
      return *this;
    }

    inline point2d & operator /= ( double d ){
      for(int i=0;i<2;i++)
	coord2d[i] = (RASM_UNITS)(((double)(coord2d[i]))/d);
      return *this;
    }

    /* handy accessors, makes the code more readable */
    RASM_UNITS inline X() const {return coord2d[0];}
    RASM_UNITS inline Y() const {return coord2d[1];}

    bool inline equals(const point2d &point) const{
      return (RASM_UNITS_equals(coord2d[0], point.coord2d[0]) &&
	      RASM_UNITS_equals(coord2d[1], point.coord2d[1]) );
    }
    bool inline operator == ( const point2d &p )const{return equals(p);}
  }; /* end of class point2d */

  /* additional operators and functions that do not mutate any existing data */
  inline point3d operator + ( const point3d &A, const point3d &B )
    {return point3d(A)+=B;}
  inline point3d operator - ( const point3d &A, const point3d &B )
    {return point3d(A)-=B;}
  inline point2d operator + ( const point2d &A, const point2d &B )
    {return point2d(A)+=B;}
  inline point2d operator - ( const point2d &A, const point2d &B )
    {return point2d(A)-=B;}

  inline point3d operator * ( const point3d &A, double d )
    {return point3d(A)*=d;}
  inline point2d operator * ( const point2d &A, double d )
    {return point2d(A)*=d;}


  /* cross product */
  inline point3d operator * ( const point3d &A, const point3d &B ){
    point3d ret(0,0,0);
    for(int i=0;i<3;i++)
      ret.coord3d[i] = (A.coord3d[(i+1)%3]*B.coord3d[(i+2)%3] - 
			A.coord3d[(i+2)%3]*B.coord3d[(i+1)%3] );
    return ret;
  }

  /* dot product */
  inline double dot( const point3d &A, const point3d &B ){
    double ret=0.0;
    for(int i=0;i<3;i++)
      ret += RASM_TO_METERS(A.coord3d[i])*RASM_TO_METERS(B.coord3d[i]);
    return ret;
  }

  inline double distSq3d(const point3d &A, const point3d &B){
    double dx = RASM_TO_METERS(A.X() - B.X());
    double dy = RASM_TO_METERS(A.Y() - B.Y());
    double dz = RASM_TO_METERS(A.Z() - B.Z());
    return dx*dx + dy*dy + dz*dz;
  }

  inline double distSq3d(double ax, double ay, double az,
			 double bx, double by, double bz){
    double dx = ax-bx;
    double dy = ay-by;
    double dz = az-bz;
    return dx*dx + dy*dy + dz*dz;
  }

  inline double dist3d(const point3d &A, const point3d &B){
    return sqrt(distSq3d(A, B));
  }

  inline double distSq2d(const point3d &A, const point3d &B){
    double dx = RASM_TO_METERS(A.X() - B.X());
    double dy = RASM_TO_METERS(A.Y() - B.Y());
    return dx*dx + dy*dy;
  }

  inline double dist2d(const point3d &A, const point3d &B){
    return sqrt(distSq2d(A, B));
  }

  inline double distSq2d(const point2d &A, const point2d &B){
    double dx = RASM_TO_METERS(A.X() - B.X());
    double dy = RASM_TO_METERS(A.Y() - B.Y());
    return dx*dx + dy*dy;
  }

  inline double dist2d(const point2d &A, const point2d &B){
    return sqrt(distSq2d(A, B));
  }


  class bounds3d{
  public:
    point3d minPoint, maxPoint;

    bounds3d()
      :minPoint(),maxPoint(){}
    bounds3d(const point3d &minP, const point3d &maxP)
      :minPoint(minP),maxPoint(maxP){}

    bool inline contains(const point3d &p) const{
      return (p.X()>=minPoint.X() && p.X()< maxPoint.X() && 
	      p.Y()>=minPoint.Y() && p.Y()< maxPoint.Y() &&
	      p.Z()>=minPoint.Z() && p.Z()< maxPoint.Z() );
    }

    void inline expandTo(const point3d &p){
      for(int i=0;i<3;i++){
	if(p.coord3d[i]<minPoint.coord3d[i])
	  minPoint.coord3d[i]=p.coord3d[i];
	if((p.coord3d[i]+RASM_EPSILON)>maxPoint.coord3d[i])
	  maxPoint.coord3d[i]=p.coord3d[i]+RASM_EPSILON;
      }
    }

    inline bounds3d & operator += ( const point3d &p ){
      minPoint+=p;
      maxPoint+=p;
      return *this;
    }

    inline bounds3d & operator -= ( const point3d &p ){
      minPoint-=p;
      maxPoint-=p;
      return *this;
    }

  }; /* end of class bounds3d */


  class bounds2d{
  public:
    point2d minPoint, maxPoint;

    bounds2d()
      :minPoint(),maxPoint(){}
    bounds2d(const point2d &minP, const point2d &maxP)
      :minPoint(minP),maxPoint(maxP){}
    bounds2d(const bounds3d &b)/* handy for casting from 3d to 2d */
      :minPoint(b.minPoint),maxPoint(b.maxPoint){}

    bool inline contains(const point2d &p) const{
      return (p.X()>=minPoint.X() && p.Y()>=minPoint.Y() &&
	      p.X()< maxPoint.X() && p.Y()< maxPoint.Y() );
    }

    void inline expandTo(const point2d &p){
      for(int i=0;i<2;i++){
	if(p.coord2d[i]<minPoint.coord2d[i])
	  minPoint.coord2d[i]=p.coord2d[i];
	if((p.coord2d[i]+RASM_EPSILON)>maxPoint.coord2d[i])
	  maxPoint.coord2d[i]=p.coord2d[i]+RASM_EPSILON;
      }
    }

    inline bounds2d & operator += ( const point2d &p ){
      minPoint+=p;
      maxPoint+=p;
      return *this;
    }

    inline bounds2d & operator -= ( const point2d &p ){
      minPoint-=p;
      maxPoint-=p;
      return *this;
    }

  }; /* end of class bounds 2d */


  class triangle{
  public:
    unsigned int points[3]; /* indices of the vertices that describe this triangle */
    int neighbors[3]; /* indices of neighbors
		       * -2 means no neighbor,
		       * -1 means the neighbors haven't been computed
		       */
    triangle(){
      points[0]=points[1]=points[2]=0;
      neighbors[0]=neighbors[1]=neighbors[2]=-1;
    }
    triangle(unsigned int a, unsigned int b, unsigned int c){
      points[0]=a; points[1]=b; points[2]=c;
      neighbors[0]=neighbors[1]=neighbors[2]=-1;
    }
    triangle(const unsigned int *p){
      points[0]=p[0]; points[1]=p[1]; points[2]=p[2];
      neighbors[0]=neighbors[1]=neighbors[2]=-1;
    }
  }; /* end of class triangle */


  class edge{
    int pointA, pointB;
    int faceA, faceB;
    edge(int pa, int pb, int fa):pointA(pa), pointB(pb), faceA(fa), faceB(-1){}
    bool match(int pa, int pb) const{
      return (pa==pointA && pb==pointB) || (pa==pointB && pb==pointA);
    }
    bool match(int pa, int pb, int pc) const{
      return match(pa, pb) || match(pb, pc) || match(pc, pa);
    }
    int setNeighbor(int fb){
      if(faceB != -1){
	printf("Error, edge used by >2 faces, can't add face %d\n", fb);
	printf("Edge defined by points %d and %d, faces %d and %d\n",
	       pointA, pointB, faceA, faceB);
	return -1;
      }
      faceB = fb;
      return 0;
    }
  };/* end of class edge */


  class mesh{
  public:
    point3d* m_vertices;
    triangle* m_faces;
    unsigned int m_num_vertices;
    unsigned int m_num_faces;
    unsigned int m_sequence_number;
    mesh() : m_vertices(NULL), m_faces(NULL), m_num_vertices(0), m_num_faces(0) 
      {
      }
    void copy_from(mesh& m)
      {
        if(m.m_num_vertices < 3) return;

        if(NULL != m_vertices) delete[] m_vertices;
        if(NULL != m_faces)    delete[] m_faces;

        m_num_vertices = m.m_num_vertices;
        m_num_faces    = m.m_num_faces;

        m_vertices = new point3d[m_num_vertices];
        if(m_num_faces > 0) m_faces = new triangle[m_num_faces];

        for(unsigned int i=0; i < m_num_vertices; i++) 
          {
            for(unsigned int j=0; j < 3; j++) 
              {
                m_vertices[i].coord3d[j] = m.m_vertices[i].coord3d[j];
              }
          }

        for(unsigned int i=0; i < m_num_faces; i++) 
          {
            for(unsigned int j=0; j < 3; j++) 
              {
                m_faces[i].points[j]    = m.m_faces[i].points[j];
                m_faces[i].neighbors[j] = m.m_faces[i].neighbors[j];
              }
          }
      }

  }; /* end of class mesh */


  class goal
  {
  public:
    point2d m_position;
    double  m_semimajor_axis;
    double  m_semiminor_axis;
    double  m_orientation_radians;
    
    goal() : m_position(0,0), m_semimajor_axis(0.0), m_semiminor_axis(0.0), m_orientation_radians(0.0)
      {}
    
    goal(const goal &g)
      {
	m_position = g.m_position; 
	m_semimajor_axis = g.m_semimajor_axis;
	m_semiminor_axis = g.m_semiminor_axis;
	m_orientation_radians = g.m_orientation_radians;
      }

    double dist_to_goal_center_meters(RASM::point3d &p)
    {
      return RASM::dist2d(m_position, p);
    }

    /*
     * Checking whether inside the oriented goal ellipse:
     * Let theta (t) be the orientation of the major axis makes with respect to the x-axis. 
     * Let a and b be the semi-major and semi-minor axes, respectively. If (x,y) is an arbitrary 
     * point then translate the point coordinates to align with the ellipse orienation:
     *   tx = (x-gx)*cos(t)+(y-gy)*sin(t)
     *   tx = -(x-gx)*sin(t)+(y-gy)*cos(t)
     * Then check whether (tx,ty) is within ellipse by checking X^2/a^2+Y^2/b^2 is less than 1 (inside 
     * the ellipse). If it equals 1, it is right on the ellipse. If greater than 1, it is outside.
     */
    bool contains(RASM::point3d& p)
    {
        double gx = RASM_TO_METERS(m_position.X());
	double gy = RASM_TO_METERS(m_position.Y());
	double rx = RASM_TO_METERS(p.X());
	double ry = RASM_TO_METERS(p.Y());
	double tx, ty; //transformed point
	bool within_goal_region = false;

	if(dist_to_goal_center_meters(p) > m_semimajor_axis) 
	  {
	    // Not within the major axis, must not be at goal
	    within_goal_region = false;
	  } 
	else if(dist_to_goal_center_meters(p) < m_semiminor_axis) 
	  {
	    // within minor axis, then at the goal very close		
	    within_goal_region = true;
	  } 
	else 
	  {
	    // Last check for intercept with actual ellipse (function of major and minor axis)

	    // Rotates rover pose into goal ellipse frame 
	    tx = (rx-gx)*cos(m_orientation_radians)+(ry-gy)*sin(m_orientation_radians);
	    ty = -(rx-gx)*sin(m_orientation_radians)+(ry-gy)*cos(m_orientation_radians);	

	    //Checks to see if rover is inside the ellipse (<1)
	    if ( ((tx*tx)/(m_semimajor_axis * m_semimajor_axis) + ((ty*ty)/(m_semiminor_axis * m_semiminor_axis)) ) <1.0) 
	      {
		within_goal_region = true;	      
	      } 
	    else 
	      {
		within_goal_region = false;
	      }
	  }

	return within_goal_region;

    } // contains()

  }; /* end of class goal */



}/* end of namespace RASM */

#endif /* end of RASM_TYPES_H */
