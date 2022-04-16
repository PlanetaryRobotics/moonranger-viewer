/* This class can read a .wrl file as saved by solidworks
 * It has no dependencies other than OpenGL to draw the model.
 *
 * Dom@cmu.edu
 */
#ifndef WRL_H
#define WRL_H

#include <math.h>
#include <stdio.h>

class wrl{
 public:

  /*** start of wrl private helper classes ***/

  /*** start of wrl tuple helper classes ***/
  struct tuple3f{float x, y, z;};
  struct tuple4f{float x, y, z, w;};
  struct tuple3i{int x, y, z;};
  /*** end of wrl tuple helper classes ***/
  
  /*** start of wrl materialData helper classes ***/
  class materialData{
  public:
    unsigned int numMaterials;
    tuple3f *ambientColor;
    tuple3f *diffuseColor;
    tuple3f *emissiveColor;
    tuple3f *specularColor;

    unsigned int numShine;
    float *shininess;
    float *transparency;
    
    materialData();
    ~materialData();
    int readFromFile(FILE *f, unsigned int &lineNum);
    bool sameAs(int ind, int i)const;
    void glColor(int ind)const;

    unsigned int marshallSize()const;
    void unmarshall(const void *data);
    void marshall(void *data)const;
  };
  /*** end of wrl materialData helper classes ***/

  /*** end of wrl private helper classes ***/

 private:
  int loadBin(const char *file, const char *binName=NULL);
  int saveBin(const char *file, const char *binName=NULL)const;

 private:
  /* internal data */
  const unsigned int dataIndex;

  bool visible;

  unsigned int numVertices;
  unsigned int numFaces;

  /* list of vertices and faces (indexes into vertices) */
  tuple3f *vertices;
  tuple3i *faces;

  /* per face indices
   * face i uses color data j=materialIndex[i]
   */
  int *materialIndex;

  /* per face material data */
  materialData *materials;

  /* indices of neighbors */
  tuple3i *neighbors;

  /* make default constructor private to avoid use */
  wrl():dataIndex(0){}

 public:

  wrl(unsigned int ind);

  ~wrl();
  int load(const char *file, const char *binName);
  void saveAsSMF(const char *file)const;
  void applyTransform(const float rpy[3][3], float x, float y, float z);
  void draw()const;

  unsigned int marshallSize()const;
  void unmarshall(const void *data);
  void marshall(void *data)const;

  const tuple3f &getVertex(unsigned int ind)const{return vertices[ind];}
  const tuple3i &getFace(unsigned int ind)const{return faces[ind];}
  unsigned int getNumVertices()const{return numVertices;}
  unsigned int getNumFaces()const{return numFaces;}
  unsigned int getDataIndex()const{return dataIndex;}

  void setVisible(bool show){visible = show;}

}; /* end of wrl class definition */

#endif
