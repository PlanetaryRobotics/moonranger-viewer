#include "myplane.h"
#include <assert.h>
#include <math.h>

#ifndef SQR
#define SQR(X) ((X)*(X))
#endif

/**
   gets the height at the given location
*/
METERS Plane::getHeight(METERS x, METERS y) const {
    return A*x+B*y+C;
}

/**
   computes pitch and roll values for a robot on this plane
   @param pitch the pitch that a rover would have, sitting on the plane with
   heading yaw
   @param roll the roll that a rover would have, sitting on the plane with 
   heading yaw
   @param yaw the heading of the rover
*/
void Plane::computePitchRoll(RADIANS &pitch, RADIANS & roll, RADIANS yaw) {
    double q;
    double cy,sy;
    cy = cos(yaw);
    sy = sin(yaw);
    q = sqrt(SQR(A)+SQR(B)+1); // the length of the hypotenuse of a triangle
    // of dz of 1
    
    pitch = asin((B*cy - A*sy)/q);
    roll = atan(-A*cy-B*sy);
}

/* returns the point on the plane closest to p */
point Plane::getPointNear(const point &p){
  /* Ax + By + C = z 
     Ax + By + (-1)z + C = 0

     a = A, b = B, c = -1, d = C
   */
  double D = (A*p.x + B*p.y - p.z + C)/sqrt(A*A + B*B + 1);
  return point(p.x - D*A, p.y - D*B, p.z + D);
}


/*************************************************
 * PLANEFITMOMENTS CLASS                         *
 ************************************************/

/**
   calculates the plane given the current data
   It's basically performing linear regression see Numerical Recipies in C
p 662 of the 2nd Edition
*/
void PlaneFitMoments::calculatePlane(void) {
    register double det;
    if (num < 3) {
        currentPlane.A = currentPlane.B = 0.0;
        currentPlane.C = 0.0;
        currentPlane.residual = 999.0;
        currentPlane.gradient = 0.0;
    } else {

    det = (Sx*(-(Syy*Sx) + Sxy*Sy) + 
	   Sxy*( (Sy*Sx) - Sxy*S) +
	   Sxx*(-(Sy*Sy) + Syy*S));

    currentPlane.A = ((Sz *  (-(Sx*Syy) + Sxy*Sy) + 
			  Syz * ( (Sx*Sy) - Sxy*S) + 
			  Sxz * (-(Sy*Sy) + Syy*S))/det);
    currentPlane.B = ((Sz *  ( (Sx*Sxy) - Sxx*Sy) + 
			  Syz * (-(Sx*Sx) + Sxx*S) + 
			  Sxz * ( (Sy*Sx) - Sxy*S))/det);
    currentPlane.C = ((Sz *  (-(Sxy*Sxy) + Sxx*Syy) +
			  Syz * ( (Sxy*Sx) - Sxx*Sy) + 
			  Sxz * (-(Syy*Sx) + Sxy*Sy))/det);
  }

}


/**
   once a plane is calculated, this determins the
   chi^2/N error in the fit.  I don't understand the 
   math here, but when I do, I'll put in a comment.
*/
void PlaneFitMoments::computeResidual(void) {
    currentPlane.residual = computeResidual(currentPlane,currentPlane.quality);
}

/**
   once a plane is calculated, this determines the maximum slope.
*/
void PlaneFitMoments::computeGradient(){
  //ignore planes with incomplete data
  if(num<3){
    currentPlane.gradient=0;
    return;
  }

  //pick a point (x, y, Zxy) along the gradient
  double x = sqrt(1/(1+ (currentPlane.A * currentPlane.A)));
  double y = sqrt(1/(1+ (currentPlane.B * currentPlane.B)));
  double Zxy = fabs(x*currentPlane.A + y*currentPlane.B);

  //get the slope of the line through this point and the origin
  currentPlane.gradient = Zxy / sqrt(x*x + y*y);
}

/**
   once a plane is calculated, this determins the
   chi^2/N error in the fit
*/
double PlaneFitMoments::computeResidual(Plane &plane,double &quality) {
    double A, B, C, chiSquared;
    double residual;
    if (num <= 1) {
        residual = 0.0;
    } else {
        A = plane.A;
        B = plane.B;
        C = plane.C;
        /* next line is:
           SQR((Sz)-A(Sx)-B(Sy)-C(S))*/
        chiSquared = (Szz + (A*A*Sxx) + (B*B*Syy) + (C*C*S)
                      + 2*(A*(C*Sx - Sxz + B*Sxy)+ B*(C*Sy - Syz) - C*Sz));
//    assert(chiSquared > -1E-4 /* must be positive, modulo rounding */);
    if (chiSquared < 0) {
      chiSquared = -chiSquared;
    }
    residual = (chiSquared/num);
    }
    quality=0;
    return residual;
}

/**
   once a plane is calculated, this determins the
   chi^2/N error in the fit
*/
double PlaneFitMoments::computeResidual(Plane &plane) {
    double dummy;
    return computeResidual(plane,dummy);
}

/**
   returns the ln of the gamma function 
   stolen straight from numerical recipies in c, pg 214
*/
double PlaneFitMoments::gammln(float xx) {
    double x,y, tmp,ser;
    double cofs[6] = {76.1800917,-86.505320,24.01410,-1.2317396,0.12087,
                      -0.539524e-5};
    int j;
    y=x=xx;
    tmp=x+5.5;
    tmp-=(x+0.5)*log(tmp);
    ser = 1.00000000019015;
    for(j=0;j<=5;j++)
        ser+=cofs[j]/++y;
    return -tmp+log(2.50662827*ser/x);
}

#define ITMAX 100
#define EPS 3e-7
#define FPMIN 1.0e-30
/**
   once a plane is calculated, this determines the quality of the
   chi^2/N fit.  This function should be called if pts were entered 
   with their standard deviations.
   @param residual, is the residual from the plane we fitted to the data
   @return The quality of the fit is returned
    */
double PlaneFitMoments::computeQuality(double residual) {
    double a; // the a paramater of the incomplete gamma function
    double x; // the x parameter of the incoplete gamma function
    double sum,del,ap;
    int n;
    double gln;
    /* this is from Numerical recipies in C, pg 219 also see pg664 */
    a = 0.5*num;
    x = 0.5*residual;

    if (x<0 || a<=0) {
        return 0;
    } 
    if (x > (a+1.0)) {
        //this is the gcf routine
        double b,c,d,h,an;
        int i;
        gln = gammln(a);
        b=x+1.0-a;
        c=1.0/FPMIN;
        d = 1.0/b;
        h =d;
        for (i=0;i<=ITMAX;i++) {
            an = -i*(i-a);
            b+=2;
            d=an*d+b;
            if (fabs(d)<FPMIN) d = FPMIN;
            c=b+an/c;
            if (fabs(c)<FPMIN) c = FPMIN;
            d =1.0/d;
            del=d*c;
            h*=del;
            if (fabs(del-1.0) <EPS) break;
        }
        return exp(-x+a*log(x)-(gln))*h;
    } 
    // this is the gser routine
    if (x ==0) {
        return 1;
    }
    gln = gammln(a);
    ap =a; 
    del=sum=1.0/a;
    for (n=1;n<=ITMAX;n++) {
        ++ap;
        del *= x/ap;
        sum +=del;
        if (fabs(del) < fabs(sum)*EPS) {
            return  1- sum*exp(-x+a*log(x)-gln);            
        }
    }
    return 1- sum*exp(-x+a*log(x)-gln);
}

#undef ITMAX
#undef EPS
#undef FPMIN
/**
   returns the plane that fits the most current data.
   If the data changes, this call will recalculate the 
   plane, otherwise it will return the previously 
   calculated plane
*/
Plane *PlaneFitMoments::getPlane(void) {
    if (dataChanged) {
      calculatePlane();
      computeResidual();
      computeGradient();
      dataChanged=0;
    }
    return &currentPlane;

}

/**
   clears all collected statistics;
**/
void PlaneFitMoments::clear(void) {
    /* tells getPlane that the plane needs to be recalculated */
    dataChanged=1;
    S=0;Sx=0;Sy=0;Sz=0;
    Sxx=0;Syy=0;Szz=0;
    Sxy=0;Sxz=0;Syz=0;
    num=0;
}


/**
   merges the given moment into this one
*/
void PlaneFitMoments::addMoment(PlaneFitMoments *moment) {
    dataChanged=1;
    num+= moment->num;
    S += moment->S;
    Sx += moment->Sx;
    Sy += moment->Sy;
    Sz += moment->Sz;
    Sxx += moment->Sxx;
    Syy += moment->Syy;
    Szz += moment->Szz;
    Sxy += moment->Sxy;
    Sxz += moment->Sxz;
    Syz += moment->Syz;
}  

/**
   adds the given x,y,z point into the data set
*/
void PlaneFitMoments::addPoint(METERS x, METERS y, METERS z) {
    dataChanged=1;
    num++;
    S += 1.0;
    Sx += x;
    Sy += y;
    Sz += z;
    Sxx += (x*x);
    Syy += (y*y);
    Szz += (z*z);
    Sxy += (x*y);
    Sxz += (x*z);
    Syz += (y*z);
}

/**
   Compute the expected variance of the height in the cell:
   E[S(Xi - Xbar)^2/(n-1)], where Xbar is the mean height of the cell.
*/
double PlaneFitMoments::computeHeightVariance (void)
{
  //return (num <= 1 ? 0 : (Szz - SQR(Sz)/num + (num-1)*SQR(sigma))/(num - 1));
  return (num <= 1 ? 0 : (Szz - SQR(Sz)/num)/(num - 1));
}


/* returns the closest relevant point to p
   if there is not enough data, the mean point is returned
   otherwise the point on the plane closest to p is returned
*/
point PlaneFitMoments::getPointNear(const point &p){
  if(!getPlane())return point(meanX(), meanY(), meanZ());
  return getPlane()->getPointNear(p);
}

