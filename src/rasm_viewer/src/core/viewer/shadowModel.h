/* Helper class to render shadows from a 3d model 
 * Based on the shadow volume method described in Nehe lesson 27
 *
 * Dom@cmu.edu
 */
#ifndef SHADOW_MODEL_H
#define SHADOW_MODEL_H

#include <GL/gl.h>
#include <math.h>

class shadowModel{
public:
  /* helper classes */
  class vertex{
  public:
    float x, y, z;
    vertex():x(0),y(0),z(0){}
    vertex(float _x, float _y, float _z):x(_x),y(_y),z(_z){}
    void glVertex()const{glVertex3f(x, y, z);}
    void glNormal()const{glNormal3f(x, y, z);}
    float mag()const{return sqrt(x*x + y*y + z*z);}
    vertex  operator +  (const vertex &b)const{return vertex(b.x+x, b.y+y, b.z+z);}
    vertex &operator += (const vertex &b)     {x+=b.x;y+=b.y;z+=b.z;return *this;}
    float   operator *  (const vertex &b)const{return b.x*x + b.y*y + b.z*z;}
    vertex  operator *  (float f        )const{return vertex(f*x, f*y, f*z);}
    vertex &operator *= (float f        )     {x*=f;y*=f;z*=f;return *this;}
    vertex &operator /= (float f        )     {x/=f;y/=f;z/=f;return *this;}
    vertex  operator -  (const vertex &b)const{return vertex(x-b.x, y-b.y, z-b.z);}
    vertex  cross       (const vertex &b)const{return vertex(y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x);}
    float   dist        (const vertex &b)const{return (*this - b).mag();}
  };

  class face{
  public:
    int points[3];
    int neighbors[3];
    vertex normal;
    unsigned char lit;
  };

  int haveNormals, haveNeighbors, haveSource;

  int numVertices;
  int numFaces;
  vertex *v;
  face *f;

  vertex *sv;//projected away from the light source

  //list of edges (defined by indices of two points) that case shadows
  int numShadowEdges;
  int *shadowEdgeA;
  int *shadowEdgeB;

  //order in which to render the edges, so that neighbors are drawn after each other
  int *shadowEdgeNext;

  shadowModel(int nVertices, int nFaces);
  ~shadowModel();

private:
  /* helper function to draw shadows onto the stencil buffer 
     (must call litFaces() and projectSource() first!)
   */
  void doShadowPass()const;

  /* given this point source, determine which faces are facing the source */
  void litFaces(const vertex &source);

  /* draws a line from the source through each vertex */
  void projectSource(float shadowDist, const vertex &source);

  /* finds all edges between lit and unlit faces */
  void findShadowEdges();
public:

  void setSource(float shadowDist, const vertex &source);

  /* draw shadows (uses the stencil buffer for intermediate computation)
   * be sure to call glColor4f beforehand to set the color of the shadow
   */
  void shadow(float nearPlane) const;

  void computeNormals(int upward);
  void computeNeighbors();
}; /* end of class shadowModel */

#endif
