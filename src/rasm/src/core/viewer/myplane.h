#ifndef PLANE_H
#define PLANE_H

#include <iostream>
#include "helpers.h"

/**
   This class holds a plane represented by Ax+By+C=z and 
   a residual value describing how well the plane fit the
   data it was fit to.
*/
class Plane {
 public:
    double A, B,C;

    /**
       The residual of the data this plane was fit
       to.  The residual is he chi^2/n value of the data fit
    */
    double residual;

    /**
       The angle of the maximum slope
    */
    double gradient;

    /**
       The quality of the data fit.  See numerical recipies in C pg 664
       a value of >0.9 is probably good.
    */
    double quality;
    friend class PlaneFitMoments;
    
    /**
       creates a new plane.
     */
    Plane(void) {A=0;B=0;C=0; residual=0;gradient=0;};

    /**
       creates a new plane with the given parameters
       The plane is defined as: z= Ax+by+C
    */
    Plane(double a,double b,double c) {A=a;B=b;C=c;residual=0;gradient=0;};


    /**
       gets the height at the given location
    */
    METERS getHeight(METERS x, METERS y) const;


    /**
       computes pitch and roll values for a robot on this plane 
       @param pitch the pitch that a rover would have, sitting on the plane 
       with heading yaw
       @param roll the roll that a rover would have, sitting on the plane with 
       heading yaw
       @param yaw the heading of the rover
    */
    void computePitchRoll(RADIANS &pitch, RADIANS & roll,
                          RADIANS yaw);

  void print(const char *pre, const char *post) const{
    printf("%sZ = %gX + %gY + %g%s", pre, A, B, C, post);
  }

  /* returns the point on the plane closest to p */
  point getPointNear(const point &p);
};

/**
   This class holds the statistical point data
*/
class PlaneFitMoments { 
    double S, Sx, Sy, Sz;
    double Sxx, Sxy, Szz;
    double Sxz, Syy, Syz;
    int num;
    /** 
        a flag to determine if the data has changed
        since we last calculated a plane
    */
    int dataChanged;
    Plane currentPlane;
protected:
    /**
       calculates the plane given the current data
    */
    void calculatePlane(void);
    /**
       once a plane is calculated, this determines the
       chi^2/N error in the fit.  I don't understand the 
       math here, but when I do, I'll put in a comment.
    */
    void computeResidual(void);
    /**
       once a plane is calculated, this determines the angle
       of the maximum slope.
    */
    void computeGradient(void);
    /**
       once a plane is calculated, this determines the quality of the
       chi^2/N fit.  This function should be called if pts were entered 
       with their standard deviations.  The closer the result is to one,
       the higher the quality of the result.
       @param residual, is the residual from the plane we fitted to the data
       @return The quality of the fit is returned
    */
    double computeQuality(double residual);
    /**
       returns the ln of the gamma function 
       stolen straight from numerical recipies in c, pg 214
    */
    double gammln(float xx);

public:
    /** 
        creates a new PlaneFitMoments object
    */
    PlaneFitMoments(void) {clear();};
    /**
       clears all collected statistics;
    **/
    void clear(void);

    /**
       returns the number of points in the given distribution
    */
    inline int getNumPts(void) {
        return num;
    }

    /**
      returns the average height (Z value) of the patch.
    */
    inline double getHeight(void) const { return Sz/S; }

    /**
       merges the given moment into this one
     */    
    void addMoment(PlaneFitMoments *moment);

    /**
       adds the given x,y,z point into the data set.
       
    */
    void addPoint(METERS x, METERS y, METERS z);
    
    /**
       returns the plane that fits the most current data.
       If the data changes, this call will recalculate the 
       plane, otherwise it will return the previously 
       calculated plane
    */
    Plane *getPlane(void);

    /**
       once a plane is calculated, this determins the
       chi^2/N error in the fit.  This is essentially performing:
       Sigma( (z - (Ax+By+c))^2)/N
       @param plane the plane we'd like to compute the residual of 
       this data against.
       @param quality the quality of the fit is returned here.  If points
       were passed without error measurements, then the quality is set to 0.
       Otherwise, a quality value of > 0.9 is probably indicative of a good fit
       @return The residual is returned.
    */
    double computeResidual(Plane &plane,double &quality);
    /**
       once a plane is calculated, this determins the
       chi^2/N error in the fit.  This is essentially performing:
       Sigma( (z - (Ax+By+c))^2)/N.  This function should be called
       if you aren't interested in getting the quality information.
       @param plane the plane we'd like to compute the residual of 
       this data against.
       @return The residual is returned.
    */
    double computeResidual(Plane &plane);

    /**
       once a plane is calculated, this determines the angle
       of the maximum slope.
    */
    double computeGradient(Plane &plane);

    /**
       Compute the expected variance of the height in the cell:
       E[S(Xi - Xbar)^2/(n-1)], where Xbar is the mean height of the cell.
    */
    double computeHeightVariance(void);

    double meanX(){return ((num>0)?Sx/S:0.0);}
    double meanY(){return ((num>0)?Sy/S:0.0);}
    double meanZ(){return ((num>0)?Sz/S:0.0);}
    point getMeanPoint(){return ((num<=0)?point(0,0,0):(point(Sx, Sy, Sz)/S));}

    /* returns the closest relevant point to p
       if there is not enough data, the mean point is returned
       otherwise the point on the plane closest to p is returned
     */
    point getPointNear(const point &p);
}; 

#endif
