/*
 * This is the tmap base class.
 * It contains the basic definition of a tmap, plus some file I/O
 *
 * Dominic Jonak
 */
#ifndef TMAP_BASE_H
#define TMAP_BASE_H

#include <rasm_common_types.h>
#include <rasm_pose.h>
#include <vector>
#include <inttypes.h>

#ifdef USE_MYSQL
#include <db.h>
#endif // USE_MYSQL

namespace TMAP {

  class tmap_base{
  private:
    unsigned int sizeVertices, capacityVertices;
    unsigned int sizeFaces, capacityFaces;

  protected:
    RASM::point3d *vertices;
    RASM::triangle *faces;
    std::vector <RASM::point3d> interestPoints;    
    
  public:
    tmap_base();

    /* an smf file */
    tmap_base(const char *filename);
    void readFromFile(const char *filename);
    void writeToFile(const char *filename) const;
    void writeToFile(const char *filename, const RASM::pose& pose) const;

#ifdef USE_MYSQL

    // read TMAP from database record
    bool getFromDB(const uint64_t record_ID); 

    // write TMAP into database
    bool sendToDB(const RASM::pose& pose) const;

#endif // USE_MYSQL

    virtual ~tmap_base();
    virtual void print() const;

    virtual void clearPointsAndTriangles();

    /* helper functions just add a new point or triangle to the list */
    void addPoint(const RASM::point3d &point);
    void addTriangle(const unsigned int tri[3], const int neighbors[3]);
    void addInterestPoint(const RASM::point3d &point);
    void resizeVertices(const uint64_t num_vertices) {sizeVertices = num_vertices;}
    void resizeVertices(const uint64_t num_vertices, const uint64_t capacity) {sizeVertices = num_vertices; capacityVertices = capacity;}
    void resizeTriangles(const uint64_t num_faces) {sizeFaces = num_faces;}
    void resizeTriangles(const uint64_t num_faces, const uint64_t capacity) {sizeFaces = num_faces; capacityFaces = capacity;}

    /* removes a point by moving the last point into this position */
    void removePoint(unsigned int ind);
    void removeInterestPoint(unsigned int ind);

    /* helper functions replace an existing point or triangle */
    void replacePoint(unsigned int ind, const RASM::point3d &point);
    void replaceTriangle(unsigned int ind, 
			 const unsigned int tri[3], const int neighbors[3]);
    void replaceInterestPoint(unsigned int ind, const RASM::point3d &point);

    void removeNeighbors();

    /* returns -1 if there is an error with any neighbor field */
    int checkNeighbors() const;

    /* simple accessors */
    inline unsigned int numPoints()const{return sizeVertices;}
    inline unsigned int capacityPoints()const{return capacityVertices;}
    inline unsigned int numTriangles()const{return sizeFaces;}
    inline unsigned int capacityTriangles()const{return capacityFaces;}
    inline RASM::point3d &getPointRef(unsigned int ind) const{return vertices[ind];}
    inline const RASM::point3d &getPoint(unsigned int ind) const{return vertices[ind];}
    inline const RASM::point3d *getPoints() const{return vertices;}
    inline const RASM::triangle *getTriangles() const{return faces;}
    inline RASM::triangle &getTriangle(unsigned int ind) const{return faces[ind];}
    inline void getTrianglePoints(unsigned int triangleIndex, RASM::point3d tri[3]) const{
      for(int i=0;i<3;i++)tri[i] = vertices[faces[triangleIndex].points[i]];
    }
    inline void getTriangleNeighbors(unsigned int triangleIndex, int tri[3]) const{
      for(int i=0;i<3;i++)tri[i] = faces[triangleIndex].neighbors[i];
    }

    virtual void errorCheck() const;
  };/* end of class tmap declaration */
}/* end of namespace TMAP */

#endif /* end of TMAP_BASE_H */
