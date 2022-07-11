/*
 * Camera classes for OpenGL rendering, used in glObject
 *
 * $Author: luisa
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>

#include "helpers.h"


#ifndef DEG_TO_RAD
#define DEG_TO_RAD(deg) ((deg) * M_PI/180.0)
#endif

static point anglesToPoint(double az, double el){
  double sa = sin(DEG_TO_RAD(az));
  double ca = cos(DEG_TO_RAD(az));
  double se = sin(DEG_TO_RAD(el));
  double ce = cos(DEG_TO_RAD(el));
  return point(se*sa, se*ca, -ce);
}

/*
 * Virtual class, used in glObject since v1.4
 */
class Camera{
public:
  virtual void setupCamera() = 0;
  virtual double getNear() = 0;
  virtual double getFar() = 0;

  /* vertical field of view */
  virtual double getFOV() = 0;
  virtual void setFOV(double fov) = 0;

  virtual int mouse(int button, int state, int x, int y) = 0;
  virtual int mouseMotion(int x, int y) = 0;
  virtual int key(unsigned char key,int x, int y) = 0;
  virtual int specialKey(int key,int x, int y) = 0;
  virtual ~Camera(){}
};

/*
 * Basic camera, taken from glObject v1.4
 */
class SimpleCamera : public Camera{
protected:
  int buttonDown;
  int mouseStartX;
  int mouseStartY;
  double mouseStartViewD, mouseStartViewAz, mouseStartViewEl;
  double nearPlane, farPlane, fov;

public:
  double elevation;
  double azimuth;
  double distance;
  double X, Y, Z;
  double stepSize;

  double ambientAzRange, ambientAzPeriod;
  double ambientElRange, ambientElPeriod;
  int enableAmbient;

  SimpleCamera() {
    ambientAzRange = ambientAzPeriod = 1.0;
    ambientElRange = ambientElPeriod = 1.0;
    enableAmbient = 0;

    nearPlane=0.5;
    farPlane=1000;
    elevation= 45;
    azimuth = 45;
    distance = 50;
    X=Y=Z=0;
    stepSize=0.05;
    fov=30.0;
  }



  point getCamLoc()const{
    point view(X, Y, Z);
    double az = azimuth, el = elevation;
    if(enableAmbient){
      SECONDS t = getCurTime();
      static SECONDS lastT = t;
      const SECONDS MAX_TIME = 1.0/24.0;
      if(t > lastT+MAX_TIME)
	t = lastT+MAX_TIME;
      lastT = t;

      az += ambientAzRange * sin((t/ambientAzPeriod)*M_PI) *(180.0/M_PI);
      el += ambientElRange * sin((t/ambientElPeriod)*M_PI) *(180.0/M_PI);
      if(el>89.999)el=89.999;
    }
    point rot = anglesToPoint(az, el);
    point origin = view + rot*(-distance);
    return origin;
  }
  void printCam()const{
    printf(" Target at %g, %g, %g\n", X, Y, Z);
    getCamLoc().print(" Camera at ", "");
    printf(" az %g el %g dist %g\n", azimuth, elevation, distance);
    printf("  (fov %g, near %g, far %g)\n", fov, nearPlane, farPlane);
  }


  void setupCamera(){
    const int lookat=1;
    if(lookat){
      point cam = getCamLoc();
      gluLookAt(cam.x, cam.y, cam.z,
		X, Y, Z,
		0, 0, 1);
    }else{
      glTranslated(0.0,0.0,-distance);
      glRotated(-elevation,1.0,0.0,0.0);
      glRotated(azimuth,0.0,0.0,1.0);
      glTranslated(-X,-Y,-Z);
    }
  }

  void setView(double x, double y, double z, double az, double el, double d){
      X=x; Y=y; Z=z;
      azimuth=az; elevation=el; distance=d;

      nearPlane = distance/20.0;
      farPlane = distance*5.0;
      stepSize = distance/5.0;

      //update linear fog
      glFogf(GL_FOG_START, distance*1.5);
      glFogf(GL_FOG_END, distance*4.0);
      glutPostRedisplay();

  }

  double getFOV(){
    return fov;
  }

  void setFOV(double _fov){
    fov = _fov;
  }

  double getNear(){return nearPlane;}
  double getFar(){return farPlane;}

  int mouse(int button, int state, int x, int y) {
    if(state==GLUT_UP){
      buttonDown = -1;
      setView(X, Y, Z, azimuth, elevation, distance);
      int w = glutGet(GLUT_WINDOW_WIDTH);
      int h = glutGet(GLUT_WINDOW_HEIGHT);
      gluPerspective(getFOV(), (float)w/(float)h, getNear(), getFar());
      //printf("gluPerspective( %g, %g)\n", getNear(), getFar());
      return 0;
    }else{
      buttonDown  = button;
      mouseStartX = x;
      mouseStartY = y;
//    printf("button down is :%d\n ", button);
      mouseStartViewD  = distance;
      mouseStartViewAz = azimuth;
      mouseStartViewEl = elevation;
      return 1;
    }
  }

  int mouseMotion(int x, int y) {
    double zoomConstant = farPlane/1000.0;
    if(zoomConstant<1e-2)zoomConstant = 1e-2;
    const double MAX_EL = 90.0 - 1e-3;
    const double MIN_EL = 1e-3;
    switch (buttonDown) {
    case -1:
      return 0;
    case 2:
      distance = mouseStartViewD + ((double)(y-mouseStartY))*zoomConstant;
      if(distance < 1e-2)distance = 1e-2;
      break;
    case 0:
      azimuth = mouseStartViewAz -((double)(x-mouseStartX))/5.0;
      elevation = mouseStartViewEl - ((double)(y-mouseStartY))/5.0;
      if (elevation > MAX_EL)
        elevation = MAX_EL;
      if (elevation < MIN_EL)
        elevation = MIN_EL;
      break;
    default:
       return 0;
    }
    glutPostRedisplay();
    return 1;
  }

  int key(unsigned char key,int x, int y) {
	  return 0;
  }

  int specialKey(int key,int x, int y) {
    switch(key) {
    case GLUT_KEY_UP:
      X+=sin(DEG_TO_RAD(azimuth))*stepSize;
      Y+=cos(DEG_TO_RAD(azimuth))*stepSize;
      break;
    case GLUT_KEY_DOWN:
      X-=sin(DEG_TO_RAD(azimuth))*stepSize;
      Y-=cos(DEG_TO_RAD(azimuth))*stepSize;
      break;
    case GLUT_KEY_LEFT:
      X-=cos(DEG_TO_RAD(azimuth))*stepSize;
      Y+=sin(DEG_TO_RAD(azimuth))*stepSize;
      break;
    case GLUT_KEY_RIGHT:
      X+=cos(DEG_TO_RAD(azimuth))*stepSize;
      Y-=sin(DEG_TO_RAD(azimuth))*stepSize;
      break;
    case GLUT_KEY_PAGE_UP:
      Z+=stepSize;
      break;
    case GLUT_KEY_PAGE_DOWN:
      Z-=stepSize;
      break;
    default:
      return 0;
    }
    glutPostRedisplay();
    return 1;
  }

  void moveCam(double deltaAz, double deltaEl){
    printf("\nmoveCam(%g, %g)\n", deltaAz, deltaEl);
    printf("Start at:\n");printCam();

    point camLoc = getCamLoc();

    azimuth+=deltaAz;
    elevation+=deltaEl;

    point rot = anglesToPoint(180+azimuth, -elevation);
    point newTarget = camLoc + rot*distance;

    X = newTarget.x;
    Y = newTarget.y;
    Z = newTarget.z;

    printf("End at:\n");printCam();
    printf("\n");
  }

}; /* SimpleCamera */

#endif /* CAMERA_H */
