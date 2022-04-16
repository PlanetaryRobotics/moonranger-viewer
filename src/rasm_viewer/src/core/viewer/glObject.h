/* $Revision: 412 $

   $Date: 2012-01-27 22:30:34 -0500 (Fri, 27 Jan 2012) $

   $Author: mwagner $*/
/** @file glDataObject.h

    PROJECT: Sunsync
    AUTHOR: Chris Urmson (curmson@ri.cmu.edu)
    DESCRIPTION:  This is an object that can be drawn in a gl context and manipulated appropriately.

    */
/*REVISION HISTORY

  $Log: glObject.h,v $
  Revision 1.8  2008/10/02 21:09:24  dom
  Seperated reshape and split functions.  Removed unused clipping plane accessor.  Added animate().  Misc cleanup.

  Revision 1.7  2008/08/01 21:25:08  dom
  Moved field of view into camera.  Added optional split parameters to reshape() to support rendering image quadrants for widescreen.

  Revision 1.6  2007/10/05 14:33:13  dom
  Can now reload files with r, or use --autoReload (which uses an idle function to check for file modification).

  Revision 1.5  2006/11/16 19:10:56  luisa
  moved camera stuff out of glObject into camera.h, updated plotClouds
  trim down triangle stuff in plotClouds, single pass, minimum processing
  moved alittle bit of #define stuff in triangle around

  Revision 1.4  2006/11/09 16:48:57  dom
  Added ability to change clipping planes.

  Revision 1.3  2006/11/03 20:37:27  dom
  Added setView

  Revision 1.2  2006/10/19 19:24:34  dom
  Made field of view a variable

  Revision 1.1  2006/10/05 19:34:53  dom
  Initial checkin

  Revision 1.2  2006/01/31 15:52:59  dom
  Added explicit glu.h include

  Revision 1.1  2003/01/06 18:16:36  mwagner
  Initial revision in atacama repository.

  Revision 1.6  2001/05/30 16:49:55  curmson
  Made globjects hideable

  Revision 1.5  2001/05/21 16:19:39  curmson
  Added the ability to output text to the screen

  Revision 1.4  2001/05/16 13:58:02  curmson
  Added a "hash" marker

  Revision 1.3  2001/05/08 21:07:42  curmson
  Tagged most things I've written with an AUTHOR line

  Revision 1.2  2001/04/06 16:50:42  curmson
  Added ability to change step distance

  Revision 1.1  2001/03/19 16:08:39  curmson
  These classes can be used to more easily interface with GL and glut.
  A lot of common code has been centralized in these classes.


  (c) Copyright 2000 CMU. All rights reserved.
*/


#ifndef GL_DATA_OBJECT
#define GL_DATA_OBJECT
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>
#include "camera.h"

typedef double SECONDS;
typedef double METERS;
struct POSITION_TYPE {
  METERS x;
  METERS y;
  METERS z;
};


/**
   this is a pure virtual base class that can be overridden to contain whatever
   3D data is required
*/
class GLObject {
    POSITION_TYPE location;
    int shouldDraw;
public:
    virtual ~GLObject(){}
    GLObject(void) {location.x =0;location.y=0;location.z=0; shouldDraw=1;};
    /**
       This moves the drawing frame to the location and then calls the draw
       member function*/
    void draw(void) {if (!shouldDraw) return;
    glPushMatrix();
    glTranslated(location.x,location.y,location.z);
    drawObject();
    glPopMatrix();}
    /**
       This actually draws the given object*/
    virtual void drawObject(void) {};
    /**
       This sets the location of the object*/
    void setPosition(POSITION_TYPE loc) {location= loc;};
    void offset(POSITION_TYPE offset) {
	location.x+=offset.x; location.y+=offset.y;location.z+=offset.z;};
    void offset(float dx, float dy, float dz) {
	location.x+=dx; location.y+=dy; location.z+=dz;};
    POSITION_TYPE getPosition(void) {return location;};
    void setPosition(float x, float y, float z) {
	location.x = x; location.y = y; location.z = z;};
    void hide(void) {shouldDraw=0;};
    void show(void) {shouldDraw=1;};
};


/**
   A simple RGB origin marker*/
class GLOrigin : public GLObject{
    float length;
public:
    /**
       creates a new drawable origin, the length of the axies will be
       length units*/
    GLOrigin(float length) {this->length= length;};
    virtual ~GLOrigin(){}
    void drawObject(void);
};

/**
   This class draws a hash mark of the given color*/
class GLHash: public GLObject {
    float size,s2;
    float R,G,B;
public:
    GLHash(float size, float R, float G, float B) {
        this->size =size; this->R=R;this->G=G;this->B=B;s2=size/2;};
    virtual ~GLHash(){}
    void drawObject(void);
};

class GLGroundPlane: public GLObject {
    float R,G,B;
    float lineSpacing;
    int numCells;
public:
    GLGroundPlane(float lineSpacing,int numCells,float R, float G, float B);
    void drawObject(void);
};

/**
   This class wraps alot of the basic glut functionality, and provides some
   basic setup behavior.*/
class GlutWindow {
    static GlutWindow * window;
    static Camera * camera;
protected:

    /** this is the width and height of the window*/
    int width, height;
    /** this is the number of milliseconds between timer calls*/
    int mSecsToTimer;
    void setInstance(GlutWindow * active) { window = active;};
    void setCamera(Camera * active){camera = active;};

public:

    static void _idle(void) {window->idle();};
    static void _display(void) {window->display();};
    static void _reshape(int w,int h) {window->reshape(w,h);};
    static void _mouse(int button, int state, int x, int y) {
		if(!window->mouse(button,state,x,y))
		  camera->mouse(button,state,x,y);
    };
    static void _key(unsigned char key,int x, int y) {
		if(!window->key(key,x,y))
		  camera->key(key,x,y);
    };
    static void _specialKey(int key,int x, int y) {
		if(!window->specialKey(key,x,y))
		  camera->specialKey(key,x,y);
    };
    static void _mouseMotion(int x, int y) {
		if(!window->mouseMotion(x,y))
		  camera->mouseMotion(x,y);
    };
    static void _timer(int value) {
        window->timer();
        glutTimerFunc(window->mSecsToTimer,&GlutWindow::_timer,value);
    }

    /**
       This creates a new window and sets up the handlers etc.
       if useTimer is set, a timer is also setup.*/
    GlutWindow(int argc, char *argv[], int width, int height,
	       int useTimer=0, SECONDS period=1.0);
    virtual ~GlutWindow(){}
    /**
       any process specific initialization can be performed here.
    */
    virtual void init(void) {};

    /**
       This returns the width of the glut window*/
    int getWidth(void)const {return width;};
    /**
       This returns the height of the glut window*/
    int getHeight(void)const {return height;};

    /**
       The program will not return from this function call*/
    void run(void) { glutMainLoop();};

    virtual void idle(void);
    /**
       This will setup the camera (call positionCamera()) and then call draw()
    */
    void display(void);
    /**
       This will draw the given string to the screen at the location given by
       x,y- (0,0 is the bottom left of the window, 1,1 is the top right).
       the color defaults to a gray
       font can be one of the GLUT_BITMAP_*
    */
    void drawText(char *txt,float x,float y, float r=0.5,float g=0.5, float b=0.5, void *font=GLUT_BITMAP_HELVETICA_10);
    virtual void draw(void) {};
    virtual void animate(void) const {};
    virtual void reshape(int w,int h);
    virtual void splitWindow(int splitX, int splitY) const;
    virtual int mouse(int button, int state, int x, int y);
    virtual int key(unsigned char key,int x, int y);
    virtual int specialKey(int key,int x, int y);
    virtual int mouseMotion(int x, int y);
    virtual void timer(void) {};
};


#endif





