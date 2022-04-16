#ifndef HELPERS_H
#define HELPERS_H

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <vector>
#include <assert.h>

typedef double SECONDS;
typedef double METERS;
typedef double RADIANS;

SECONDS getCurTime();

class point{
 public:
  METERS x, y, z;
  unsigned int m_num_original_points;
  float m_confidence;
 point(METERS _x, METERS _y, METERS _z):x(_x),y(_y),z(_z),m_num_original_points(1),m_confidence(0.5){}
  point(const point &copy):x(copy.x),y(copy.y),z(copy.z),m_num_original_points(1),m_confidence(0.5){}

  inline void print(const char *pre, const char *post) const
    {printf("%s%g, %g, %g%s", pre, x, y, z, post);}

  void glVertex() const;
  void glNormal() const;

  inline point & square(){x*=x;y*=y;z*=z;return *this;}
  inline METERS sum() const{return x+y+z;}
  inline METERS normSq() const{return x*x + y*y + z*z;}
  inline METERS norm() const{return sqrt(normSq());}

  inline point operator * ( METERS m ) const{return point(x*m, y*m, z*m);}
  inline point operator / ( METERS m ) const{return point(x/m, y/m, z/m);}
  inline point & operator *= ( METERS m ){x*=m;y*=m;z*=m;return *this;}
  inline point & operator /= ( METERS m ){x/=m;y/=m;z/=m;return *this;}
  inline point & operator += ( const point &B ){x+=B.x;y+=B.y;z+=B.z;return *this;}
  inline point & operator -= ( const point &B ){x-=B.x;y-=B.y;z-=B.z;return *this;}

  void applyTransform(const float rpy[3][3], float x, float y, float z);
};

inline point operator + ( const point &A, const point &B ){return point(A)+=B;}
inline point operator - ( const point &A, const point &B ){return point(A)-=B;}

METERS inline distSqBetween(const point &a, const point &b)
{return (a - b).square().sum();}
METERS inline distBetween(const point &a, const point &b)
{return sqrt((a - b).square().sum());}

class triangle{
  point a, b, c, norm;
  int ind_A, ind_B, ind_C;
 public:
  triangle(point _a, point _b, point _c);
  void setInd(int a, int b, int c);
  void getInd(int &a, int &b, int &c)const{a=ind_A;b=ind_B;c=ind_C;}
  void print();
  void draw();
  void drawWire();
  point getNorm() const{return norm;}
  point getLow()  const;
  point getMid()  const;
  point getHigh() const;
  bool contains2D   (const point &p) const;
  bool containsRay  (const point &a, const point &b, point &intersect) const;
  bool containsLine (const point &a, const point &b, point &intersect) const;
  point nearestPoint(const point &p) const;
  METERS distTo(const point &p) const;
  void getHeight(point &p)const;
  void boundingBox(point &minPoint, point &maxPoint)const;
  const point &getA() const{return a;}
  const point &getB() const{return b;}
  const point &getC() const{return c;}

  void getLengthsSq(METERS l[3]) const;
  void getLengths(METERS l[3]) const;
  METERS getPerimeter() const;
};

class orient{
 public:
  RADIANS roll, pitch, yaw;
  orient(RADIANS _roll, RADIANS _pitch, RADIANS _yaw):roll(_roll),pitch(_pitch),yaw(_yaw){}
  void print(const char *pre, const char *post){
    printf("%s%g, %g, %g%s", pre, roll, pitch, yaw, post);
  }
  void printDegrees(const char *pre, const char *post){
    printf("%s%g %g %g%s", pre, roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI, post);
  }
};

class transformMatrix{
 public:
  double mat[4][4];
  transformMatrix();
  transformMatrix(const point &p);
  transformMatrix(const orient &o);
  transformMatrix(const transformMatrix *t){copyFrom(t);}
  transformMatrix(const transformMatrix &t){copyFrom(&t);}
  void copyFrom(const transformMatrix *t);
  inline void copyFrom(const transformMatrix &t){copyFrom(&t);}
  void setIdentity();
  void multiplyBy(const transformMatrix *t);
  transformMatrix multiply(const transformMatrix &t) const;
  void applyToPoint(point &p) const;
  void print(const char *prefix) const;

  point getCenter() const;
  orient getAngles() const;

  static transformMatrix rollMatrix(RADIANS angle); /* about Y */
  static transformMatrix pitchMatrix(RADIANS angle); /* about X */
  static transformMatrix yawMatrix(RADIANS angle); /* about Z */

  inline transformMatrix & operator *= ( const transformMatrix &M ){
    multiplyBy(&M);
    return *this;
  }
  inline transformMatrix & operator *= ( const transformMatrix *M ){
    multiplyBy(M);
    return *this;
  }
};

inline transformMatrix operator * ( const transformMatrix &A,
				    const transformMatrix &B)
{return A.multiply(B);}
inline transformMatrix operator * ( const transformMatrix *A,
				    const transformMatrix &B)
{return A->multiply(B);}
inline transformMatrix operator * ( const transformMatrix &A,
				    const transformMatrix *B)
{return A.multiply(*B);}

inline point operator * ( const transformMatrix &M, const point &P ){
  point p(P);
  M.applyToPoint(p);
  return p;
}

class transformation{
 public:
  transformMatrix mat;
  transformation(){}
  virtual ~transformation(){}
  virtual transformation *copy()=0;
  virtual void apply()const=0;
  virtual void undo()const=0;
  virtual void applyToPoint(point &p)const {mat.applyToPoint(p);}
  virtual void print(const char *prefix)const=0;
}; /* end of virtual base class tranformation */

class importedTransform : public transformation{
 public:
  importedTransform(const transformMatrix *t)
    :transformation()
    {mat.copyFrom(t);}
  importedTransform(const importedTransform &copy)
    :transformation()
    {mat.copyFrom(&(copy.mat));}
  virtual transformation *copy(){return new importedTransform(*this);}
  virtual void apply()const{}
  virtual void undo()const{}
  virtual void print(const char *prefix)const{}
}; /* end of virtual class importedTranform */

class rotation : public transformation{
 protected:
  RADIANS roll, pitch, yaw;
  void calcMatrix();
 public:
  rotation(const rotation &copy):transformation(),
    roll(copy.roll),pitch(copy.pitch),yaw(copy.yaw)
    {mat.copyFrom(&(copy.mat));}
  rotation(RADIANS _roll, RADIANS _pitch, RADIANS _yaw):transformation(),
    roll(_roll),pitch(_pitch),yaw(_yaw)
    {calcMatrix();}
  rotation(orient orientation):transformation(),
    roll(orientation.roll),pitch(orientation.pitch),yaw(orientation.yaw)
    {calcMatrix();}
  void adjust(rotation delta);
  virtual transformation *copy(){return new rotation(*this);}
  void apply()const;
  void undo()const;
  void print(const char *prefix)const;
  orient getOrientation()const{return orient(roll, pitch, yaw);}
};/* end of class rotation */

class translation : public transformation{
 protected:
  METERS x, y, z;
  void calcMatrix();
 public:
  translation(const translation &copy):transformation(),
    x(copy.x),y(copy.y),z(copy.z)
    {mat.copyFrom(&(copy.mat));}
  translation(METERS _x, METERS _y, METERS _z):transformation(),
    x(_x),y(_y),z(_z)
    {calcMatrix();}
  translation(point position):transformation(),
    x(position.x),y(position.y),z(position.z)
    {calcMatrix();}
  void adjust(translation delta);
  virtual transformation *copy(){return new translation(*this);}
  void apply()const;
  void undo()const;
  void print(const char *prefix)const;
  point getPosition()const{return point(x, y, z);}
};/* end of class translation */

class composite : public transformation{
 protected:
  transformation *a, *b;
  void calcMatrix();
  void copyFrom(const composite *copy);
 public:
  composite(transformation *_a, transformation *_b)
    :transformation(),a(_a),b(_b){calcMatrix();}
  composite(const composite &copy);
  composite(const composite *copy);
  ~composite(){assert(a);assert(b);delete a; delete b; a = b = NULL;}
  virtual transformation *copy(){return new composite(*this);}
  void apply()const;
  void undo()const;
  void print(const char *prefix)const;
};


template <typename T>
class mycontainer{
 public:
  std::vector<T> data;
  void add(T p){data.push_back(p);}
  void add(const mycontainer *c){
    for(unsigned int i=0;i<c->size();i++)
      data.push_back(c->get(i));
  }
  void add(const mycontainer &c){
    for(unsigned int i=0;i<c.size();i++)
      data.push_back(c.get(i));
  }
  unsigned int size() const{return data.size();}
  T get(unsigned int ind)const {return T(data[ind]);}
  T &getRef(unsigned int ind){return data[ind];}
  bool empty() const{return data.empty();}
};

class points : public mycontainer<point>{
 public:
  /* use NULL for stdin */
  points(const char *file, unsigned int subsample=1);
  points(){}
  void applyTransform(const transformation *t);
  points copy() const;
  point mean() const;
  /* use NULL for stdout */
  void writeToFile(const char *filename) const;
  points subsample(unsigned int factor) const;
  void boundingBox(point &minPoint, point &maxPoint);
};

/* returns the points from the scene that are in the model */
points findInsidePoints(const points &model, const points &scene,
			METERS nearDist);

/* gives a measurement of how far about the two sets of points are */
double getError(points a, points b);


class triangles : public mycontainer<triangle>{
 public:
  triangles(){}
  points findInteriorPoints(const points &p) const;
  points findExteriorPoints(const points &p) const;
  point getPointNear(const point &p) const;
  void getHeight(point &p)const;
};

class transformStack : public mycontainer<transformation *>{
 public:
  transformation *getTransform()const;
};

char *toBinName(const char *file);

#endif
