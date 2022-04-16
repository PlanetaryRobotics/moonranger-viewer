/* $Revision: 1.10 $

   $Date: 2008/10/02 21:09:24 $

   $Author: dom $*/
/** @file glObject.cc

    PROJECT: Sunsync
    AUTHOR: Chris Urmson (curmson@ri.cmu.edu)
    DESCRIPTION: This implements alot of the basic glut functions we'll want to use.

    */
/*REVISION HISTORY

  $Log: glObject.cc,v $
  Revision 1.10  2008/10/02 21:09:24  dom
  Seperated reshape and split functions.  Removed unused clipping plane accessor.  Added animate().  Misc cleanup.

  Revision 1.9  2008/08/29 18:44:05  dom
  Preliminary shadow implementation, enable with --shadow

  Revision 1.8  2008/08/26 20:44:59  dom
  Added a few more gl options (normalize, perspective correction and polygon smooth).

  Revision 1.7  2008/08/01 21:25:08  dom
  Moved field of view into camera.  Added optional split parameters to reshape() to support rendering image quadrants for widescreen.

  Revision 1.6  2007/10/05 14:33:13  dom
  Can now reload files with r, or use --autoReload (which uses an idle function to check for file modification).

  Revision 1.5  2006/11/16 19:10:56  luisa
  moved camera stuff out of glObject into camera.h, updated plotClouds
  trim down triangle stuff in plotClouds, single pass, minimum processing
  moved alittle bit of #define stuff in triangle around

  Revision 1.4  2006/11/09 16:58:10  dom
  Made zooming (drag with right button) and stepping (arrow keys) sensitive to scale.

  Revision 1.3  2006/11/09 16:48:57  dom
  Added ability to change clipping planes.

  Revision 1.2  2006/10/19 19:24:34  dom
  Made field of view a variable

  Revision 1.1  2006/10/05 19:34:53  dom
  Initial checkin

  Revision 1.2  2003/06/05 17:22:00  curmson
  Changes to allow compilation with gcc 3.2.

  Revision 1.1  2003/01/06 18:16:36  mwagner
  Initial revision in atacama repository.

  Revision 1.10  2001/06/04 17:26:58  curmson
  Fixed typo

  Revision 1.9  2001/06/04 17:21:26  curmson
  Added hiding ability

  Revision 1.8  2001/05/30 16:49:55  curmson
  Made globjects hideable

  Revision 1.7  2001/05/21 16:19:39  curmson
  Added the ability to output text to the screen

  Revision 1.6  2001/05/16 13:58:02  curmson
  Added a "hash" marker

  Revision 1.5  2001/05/08 21:07:42  curmson
  Tagged most things I've written with an AUTHOR line

  Revision 1.4  2001/04/12 17:30:06  curmson
  Motion is now in the direction the user is looking

  Revision 1.3  2001/04/06 16:50:42  curmson
  Added ability to change step distance

  Revision 1.2  2001/04/01 16:10:08  curmson
  Minor fix

  Revision 1.1  2001/03/19 16:08:39  curmson
  These classes can be used to more easily interface with GL and glut.
  A lot of common code has been centralized in these classes.


  (c) Copyright 2000 CMU. All rights reserved.
*/
#include "glObject.h"

/*************************************************
 * Common Object functions                       *
 ************************************************/

void GLOrigin::drawObject(void) {
    glColor3f(1.0,0.0,0.0);

    glBegin(GL_LINES);
     glVertex3f(0,0,0);
     glVertex3f(length,0,0);
     glColor3f(0.0,1.0,0.0);
     glVertex3f(0,0,0);
     glVertex3f(0,length,0);
     glColor3f(0.0,0.0,1.0);
     glVertex3f(0,0,0);
     glVertex3f(0,0,length);
    glEnd();
}

void GLHash::drawObject(void) {
    glColor3f(R,G,B);
    glBegin(GL_LINES);
      glVertex3f(-s2,0,0);
      glVertex3f(s2,0,0);
      glVertex3f(0,-s2,0);
      glVertex3f(0,s2,0);
      glVertex3f(0,0,-s2);
      glVertex3f(0,0,s2);
    glEnd();

}


GLGroundPlane::GLGroundPlane(float lineSpacing,int numCells,
			     float R, float G, float B) {
    this->R = R;
    this->G = G;
    this->B = B;
    this->lineSpacing = lineSpacing;
    this->numCells = numCells;
}
void GLGroundPlane::drawObject(void) {
    int i;
    glColor3f(R,G,B);
    glBegin(GL_LINES);
    for (i=-numCells/2;i<numCells/2+1;i++) {
        glColor3f(R,G,B);
        if ((i+numCells)%4==0)
            glColor3f(R,G,1);
        glVertex3f(i*lineSpacing,-lineSpacing*numCells/2,0);
        glVertex3f(i*lineSpacing,lineSpacing*numCells/2,0);
        glVertex3f(-lineSpacing*numCells/2,i*lineSpacing,0);
        glVertex3f(lineSpacing*numCells/2,i*lineSpacing,0);
    }
    glEnd();

}

/*************************************************
 * GLUT window functions                         *
 ************************************************/



/*
  define the GlutWindow static member*/
GlutWindow * GlutWindow::window = NULL;

Camera * GlutWindow::camera = NULL;

/**
       This creates a new window and sets up the handlers etc.
       if useTimer is set, a timer is also setup.*/
GlutWindow::GlutWindow(int argc, char *argv[], int width, int height,
		       int useTimer, SECONDS period) {
    this->width = width;
    this->height= height;
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE| GLUT_DEPTH | GLUT_STENCIL | GLUT_RGB);
    glutInitWindowSize(width,height);
//    glutInitWindowPosition(100,100);
    glutCreateWindow(argv[0]);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    glClearColor(0, 0, 0, 0.5);//black background, just like the shadows
    glClearDepth(1.0);
    glClearStencil(0);
    glDepthFunc(GL_LEQUAL);

    setInstance(this);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    glutIdleFunc(_idle);
    glutDisplayFunc(_display);
    glutReshapeFunc(_reshape);
    glutMouseFunc(_mouse);
    glutKeyboardFunc(_key);
    glutSpecialFunc(_specialKey);
    glutMotionFunc(_mouseMotion);
    if (useTimer) {
	    mSecsToTimer = int(period*1000);
        glutTimerFunc(int(period*1000),_timer,1);
    }
    init();
}

void GlutWindow::idle(void) {}

void GlutWindow::display(void) {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    camera->setupCamera();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    draw();
    glFlush();
    glutSwapBuffers();
    animate();
}

/* set the height and width */
void GlutWindow::reshape(int w,int h) {
  this->width  = w;
  this->height = h;
  splitWindow(0, 0);
}

/* splits the view into a quadrant,
 * specify a quadrant with splitX and splitY = +/-1
 */
void GlutWindow::splitWindow(int splitX, int splitY) const {
  glViewport(0,0,(GLsizei)width,(GLsizei)height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  GLfloat aspect = ((GLfloat)width)/((GLfloat)height);
  GLfloat fovY = (GLfloat)camera->getFOV();
  GLfloat f = tan(DEG_TO_RAD(fovY));
  GLfloat zmin = camera->getNear();
  GLfloat zmax = camera->getFar();
  GLfloat ymax =  f * zmin;
  GLfloat ymin = -f * zmin;
  GLfloat xmax = ymax*aspect;
  GLfloat xmin = ymin*aspect;

  /* make sure both x and y are split or not split */
  if(!splitX ^ !splitY){
    printf("Error, splitX = %d, splitY = %d\n", splitX, splitY);
    abort();
  }

  if(0==splitX && 0==splitY){
    gluPerspective(fovY, aspect, zmin, zmax);
  }else{
    if(splitX>0)xmin=0;
    else xmax=0;
    if(splitY>0)ymin=0;
    else ymax=0;
    glFrustum(xmin, xmax, ymin, ymax, zmin, zmax);
  }
}

int GlutWindow::mouse(int button, int state, int x, int y) {
  return 0;
}

int GlutWindow::key(unsigned char key,int x, int y) {
  return 0;
}

int GlutWindow::specialKey(int key,int x, int y) {
  return 0;
}

int GlutWindow::mouseMotion(int x, int y) {
  return 0;
}

/**
   This will draw the given string to the screen at the location given by
   x,y- (0,0 is the bottom left of the window, 1,1 is the top right).
   the color defaults to a gray*/
void GlutWindow::drawText(char *txt,float x,float y,
                          float r,float g, float b,
                          void *font) {
    GLint matrixMode;
    GLboolean lightingOn;
    char *ch;
    GLenum error;
    lightingOn= glIsEnabled(GL_LIGHTING);        /* lighting on? */
    if (lightingOn) glDisable(GL_LIGHTING);

    glGetIntegerv(GL_MATRIX_MODE, &matrixMode);  /* matrix mode? */

    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
        glLoadIdentity();
        glPushAttrib(GL_COLOR_BUFFER_BIT);       /* save current colour */
        glColor3f(r, g, b);
        glRasterPos3f(x, y, 0);
        for(ch= txt; *ch; ch++) {
            glutBitmapCharacter(font, (int)*ch);
        }
        glPopAttrib();
      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(matrixMode);
    if (lightingOn) glEnable(GL_LIGHTING);
    error = glGetError();
    if (error !=GL_NO_ERROR)
      std::cerr << gluErrorString(error) << std::endl;
}
