/**
 * @file rasm_pose.h
 *
 * @section LICENSE
 * Copyright 2012, Carnegie Mellon University and ProtoInnovations, LLC. 
 * All rights reserved. This document contains information that is proprietary
 * to Carnegie Mellon University and ProtoInnovations, LLC. Do not distribute
 * without approval.
 *
 */

#ifndef RASM_POSE_H
#define RASM_POSE_H

#include <rasm_common_types.h>
#include <math.h>
#include <cmath>
#include <assert.h>

namespace RASM {

  class pose{
  public:

    /*** helper classes ***/

    /* RASM::pose::position is a rebranded RASM::point3d */
    class position{
    private:
      RASM_UNITS x, y, z;
    public:
      /* default constructor */
      position():x(0),y(0),z(0){}

      /* conversion to and from RASM::point3d */
      position(const RASM::point3d &rhs)
	:x(rhs.coord3d[0]),y(rhs.coord3d[1]),z(rhs.coord3d[2]){}
      operator RASM::point3d()const{
	return RASM::point3d(x,y,z);
      }

      /* overloaded operators */
      position operator + (const position &rhs)const{
	return position((RASM::point3d)(*this) + (RASM::point3d)(rhs));
      }
      position operator - (const position &rhs)const{
	return position((RASM::point3d)(*this) - (RASM::point3d)(rhs));
      }
      position operator * (double d)const{
	return position((RASM::point3d)(*this) * d);
      }

      bool isValid() const{
	return (std::isfinite(x) && std::isfinite(y) && std::isfinite(z));
      }
    };/* end of class RASM::pose::position */

    /* RASM::pose::orientation is another simple class */
    class orientation{
    public:
      float roll, pitch, yaw;/* in radians */
      orientation():roll(0.0),pitch(0.0),yaw(0.0){}
      orientation(float r, float p, float y):roll(r),pitch(p),yaw(y){
	fixYaw();
      }
      orientation(const orientation &o):roll(o.roll),pitch(o.pitch),yaw(o.yaw){
	fixYaw();
      }
      void set(float r, float p, float y){
	roll=r;pitch=p;yaw=y;
	fixYaw();
      }
      void fixYaw(){
	assert(fabs(yaw) < 2.0*M_PI*100); // an arbitrary limit, but avoid never breaking out of below loops
	while(yaw> M_PI)yaw-=2.0*M_PI;
	while(yaw<-M_PI)yaw+=2.0*M_PI;
      }

      static float deltaYaw(float a, float b){
	while(a-b > M_PI)a-=2.0*M_PI;
	while(b-a > M_PI)b-=2.0*M_PI;
	return (a-b);
      }

      /* overloaded operators */
      orientation operator + (const orientation &rhs)const{
	return orientation(roll+rhs.roll, pitch+rhs.pitch, yaw+rhs.yaw);
      }
      orientation operator - (const orientation &rhs)const{
	return orientation(roll-rhs.roll, pitch-rhs.pitch, yaw-rhs.yaw);
      }
      orientation operator * (double d)const{
	return orientation(roll*d, pitch*d, yaw*d);
      }

      bool isValid() const{
	return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
      }
    };/* end of class RASM::pose::orientation */
    /*** end of helper classs ***/

  protected:

    /* member variables of pose class */
    bool valid;
    position pos;
    orientation orient;
    double time;/* in seconds */

  public:

    /* pose constructors */
    pose():valid(false){time=0.0;}
    pose(const position &_pos, float r, float p, float y, double t)
      :valid(true),pos(_pos),orient(r,p,y),time(t){}
    pose(const position &_pos, const orientation &_orient, double t)
      :valid(true),pos(_pos),orient(_orient),time(t){}
    pose(const pose &_pose)
      :valid(true),pos(_pose.pos),orient(_pose.orient),time(_pose.time){}

    
    /* accessors and mutators */
    bool isValid()const{return valid && pos.isValid() && orient.isValid() && std::isfinite(time) && (time > 1000000000);}
    double getTime()const{return time;}
    const position &getPosition()const{return pos;}
    const orientation &getOrientation()const{return orient;}
    void get(RASM::point3d &_pos,float &r,float &p,float &y)const
      {_pos=pos;r=orient.roll;p=orient.pitch;y=orient.yaw;}
    void set(const RASM::point3d &_pos, float r, float p, float y, double t)
      {valid=true;pos=_pos;orient.set(r,p,y);time=t;}


    /* overloaded operators */
    pose operator + (const pose &rhs)const{
      return pose(pos+rhs.pos, orient+rhs.orient, time+rhs.time);
    }
    pose operator - (const pose &rhs)const{
      return pose(pos-rhs.pos, orient-rhs.orient, time-rhs.time);
    }
    pose operator * (double d)const{
      return pose(pos*d, orient*d, time*d);
    }


    /* comparison functions,
     * see if the two poses differ by some amount in any component
     * (unrelated fields are ignored)
     */
    static bool beyondDist(const pose &a, const pose &b, double dist){
      return (RASM::distSq2d(a.pos, b.pos)>(dist*dist));
    }
    static bool beyondTime(const pose &a, const pose &b, double time){
      return (fabs(a.time-b.time)>time);
    }
    static bool beyondDistTime(const pose &a, const pose &b, 
			       double dist, double time){
      return ((fabs(a.time-b.time)>time)                 ||
	      (RASM::distSq2d(a.pos, b.pos)>(dist*dist)) );
    }
    static bool beyondDistYaw(const pose &a, const pose &b, 
			      double dist, double yaw){
      return ((orientation::deltaYaw(a.orient.yaw, b.orient.yaw) > yaw) ||
	      (RASM::distSq2d(a.pos, b.pos)>(dist*dist))                );
    }
    static bool beyondDistYawTime(const pose &a, const pose &b, 
				  double dist, double yaw, double time){
      return ((fabs(a.time-b.time)>time)                                ||
	      (orientation::deltaYaw(a.orient.yaw, b.orient.yaw) > yaw) ||
	      (RASM::distSq2d(a.pos, b.pos)>(dist*dist))                );
    }


    static void linearFit(const pose &cur, const pose &prev, 
			  double t, double alpha/* attenuation factor */,
			  RASM::point3d &pos, float &r, float &p, float &y){
      const RASM::pose dp = prev-cur;

      printf("[%s:%d] cur.getTime() = %f , prev.getTime() = %f\n",
	     __FILE__, __LINE__, cur.getTime(), prev.getTime());

      /* 
       * Interpolate/extrapolate if possible. If the pose source happens to 
       * send two pose messages too close together then we can't 
       * meaningfully extrapolate. How close is "too close" is determined
       * by min_pose_time_step_sec.
       */
      const double min_pose_time_step_sec = 1e-4;
      const double max_pose_extrapolation_time_sec = 10.0; // avoid exterpolating too far
      if( (cur.getTime() > (prev.getTime() + min_pose_time_step_sec)) && /* not too close an interpolation */
	  ((cur.getTime() - prev.getTime()) < max_pose_extrapolation_time_sec) ) /* not too far an extrapolation */
	{
	  const double k=alpha*(t - cur.getTime()) / (prev.getTime() - cur.getTime());
	  const RASM::pose poseT(cur + dp*k);
	  poseT.get(pos, r, p, y);
	}
      else
	{
	  /*
	   * If we can't interpolate/extrapolate then just return the more 
	   * recent available pose.
	   */
	  cur.get(pos, r, p, y);
	}
    }

  }; /* end of class pose */

} /* end of namespace RASM */

#endif
