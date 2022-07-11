/*
 * This class combines the functionality of all the other tmap classes.
 * It also checks for conflicts/preconditions.
 */

#ifndef TMAP_FULL_H
#define TMAP_FULL_H

/* whether or not to check for valid flags before executing functions */
#define CHECK_VALID_BITS 1

#include <tmap_shrinkable.h>
#include <tmap_cleanable.h>
#include <tmap_astar.h>
#include <tmap_icp.h>
#include <tmap_searchable.h>
#include <tmap_pasteable.h>
#include <tmap_group.h>
#include <tmap_arc.h>
#include <multiArc.h>

#ifdef USE_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#endif // USE_PCL

namespace TMAP {

  class tmap{
    //  private:
  public:
    class tmap_uber:
      public virtual tmap_astar,
      public virtual tmap_icp,
      public virtual tmap_cleanable,
      public virtual tmap_pasteable,
      public virtual tmap_group,
      public virtual tmap_shrinkable{};
    tmap_uber data;

    enum VALID_BITS{ triangulation=0,
		     centers,
		     associations, 
		     neighbors,
		     quadVertices,
		     quadCenters,
		     astarGoal,
		     numValid};
    bool valid[numValid];

    void checkNeighbors() const;

  public:
    tmap();
    virtual ~tmap();

    /* new functions */
    tmap(const tmap &copy);

#ifdef USE_PCL
    tmap(const pcl::PointCloud<pcl::PointXYZ> &cloud);
    // TBD: support reading in a triangulated PointCloud 
    operator pcl::PointCloud<pcl::PointXYZ> () const;
    operator pcl::PointCloud<pcl::PointXYZ>::Ptr () const;
#endif //USE_PCL

    /* void evaluateNear(multiPath &arc, obstacleMAP &obstacles, */
    /* 		      bool extraWide, FILE *savedEvaluation = NULL); */
    /* void evaluateFar(multiPath &arc, const obstacleMAP &obstacles, */
    /* 		     bool extraWide, FILE *savedEvaluation = NULL, */
    /* 		     bool savePaths = false); */

    /* like paste() but only removes duplicate points
     * and leaves the current model untouched,
     * puts the union into a new model
     */
    void combineInto(tmap &result, const tmap &b)const;

    void rebuild();
    void tearDown();
    void showValid() const;
    void setValidTriangulation();//mark that we manually triangulated
#if CHECK_VALID_BITS
    void assertValid(VALID_BITS n, bool validState) const;
#else
    inline void assertValid(VALID_BITS n, bool validState) const{}
#endif

    /* from tmap */
    void readFromFile(const char *file);
    void writeToFile(const char *file, const RASM::pose& pose) const{data.writeToFile(file, pose);}
    void writeToFile(const char *file) const{data.writeToFile(file);}

#ifdef USE_MYSQL

    // read TMAP from database record
    bool getFromDB(const uint64_t record_ID) { return data.getFromDB(record_ID); }

    // write TMAP into database
    bool sendToDB(const RASM::pose& pose) const { return data.sendToDB(pose); }

#endif // USE_MYSQL

    void errorCheck() const{data.tmap_cleanable::errorCheck();}
    void fillInCenters();
    void removeCenters();
    void fillInAssociations();
    void removeAssociations();
    void fillInNeighbors();
    void removeNeighbors();
    void addPoint(const RASM::point3d &point,
		  bool invalidate=true);
    void addInterestPoint(const RASM::point3d &point,
		  bool invalidate=true);
    void addTriangle(const unsigned int tri[3], 
		     const int neighbors[3],
		     bool invalidate=true);

    void removePoint(unsigned int ind, bool invalidate=true);
    void removeInterestPoint(unsigned int ind, bool invalidate=true);

    void replacePoint(unsigned int ind, 
		      const RASM::point3d &point,
		      bool invalidate=true);
    void replaceInterestPoint(unsigned int ind, 
		      const RASM::point3d &point,
		      bool invalidate=true);
    void replaceTriangle(unsigned int ind,
			 const unsigned int tri[3],
			 const int neighbors[3],
			 bool invalidate=true);
    inline unsigned int numPoints()const
      {return data.numPoints();}
    inline unsigned int numTriangles()const
      {return data.numTriangles();}
    const RASM::point3d &getPoint(unsigned int ind) const;
    inline const RASM::point3d *getPoints()const
      {return data.getPoints();}
    inline const RASM::triangle *getTriangles()const
      {return data.getTriangles();}
    const RASM::triangle &getTriangle(unsigned int ind) const;
    inline const RASM::point3d &getCenter(unsigned int ind) const
      {return data.tmap_cleanable::getCenter(ind);}
    void clearPointsAndTriangles();
    void importMesh(RASM::mesh& _mesh, bool do_rebuild=true) 
    { 
      clearPointsAndTriangles();

      for(unsigned int i=0; i < _mesh.m_num_vertices; i++)
	{
	  addPoint(_mesh.m_vertices[i]);
	}
      for(unsigned int i=0; i < _mesh.m_num_faces; i++)  
	{
	  addTriangle(_mesh.m_faces[i].points,
		      _mesh.m_faces[i].neighbors);
	}

      setValidTriangulation();

      if(do_rebuild) rebuild();
    }

    /* from cleanable tmap */
    void maskBadData(tmap_cleanable::cleanableTMAPfilters filter,
		     float threshold)
      {data.maskBadData(filter, threshold);}
    void maskBadData(tmap_cleanable::cleanableTMAPfilters filter)
      {data.maskBadData(filter);}
    void maskFarData(RASM_UNITS distance, RASM::point2d center)
      {data.maskFarData(distance, center);}
    void maskDataLine(RASM::point2d start, RASM::point2d end,
		      RASM_UNITS minDist, RASM_UNITS maxDist,
		      float allowableAngle)
      {data.maskDataLine(start, end, minDist, maxDist, allowableAngle);}
    void removeBadData();

    /* from shrinkable tmap */
    void shrink(bool decimate, unsigned int finalSize, bool retriangulate);
    void setDecimationMode(tmap_shrinkable::DECIMATE_MODE mode)
      {data.setDecimationMode(mode);}
    void setCustomDecimation(double nearCutoff, double nearWidth, 
			     double nearDepth,  double nearRes,
			     double farCutoff,  double farWidth, 
			     double farDepth,   double farRes,
			     double maxAlt,
			     double minAlt,
			     bool keepAllNear,  bool keepAllFar,
			     bool nearApplyMinPoints, bool farApplyMinPoints,
            		     bool nearApplyMeanWithin, bool farApplyMeanWithin,
		             bool nearApplyMeanAmongAll, bool farApplyMeanAmongAll,
	            	     bool nearApplyMeanNeighbors, bool farApplyMeanNeighbors,
            		     bool nearOutputFromMean, bool farOutputFromMean,
            		     int nearMinGridPoints, int farMinGridPoints){
      data.setCustomDecimation(nearCutoff, nearWidth, nearDepth, nearRes,
			       farCutoff,  farWidth,  farDepth,  farRes,
			       maxAlt, minAlt, keepAllNear, keepAllFar, nearApplyMinPoints,
			       farApplyMinPoints, nearApplyMeanWithin, farApplyMeanWithin,
			       nearApplyMeanAmongAll, farApplyMeanAmongAll, nearApplyMeanNeighbors,
			       farApplyMeanNeighbors, nearOutputFromMean, farOutputFromMean,
			       nearMinGridPoints, farMinGridPoints);
    }

    /* from triangulate tmap */
    void triangulateMinimal();
    void triangulate();

    /* from pasteable tmap */
    void paste(const tmap &newmap,
	       const float tooClose/* meters */);

    /* from searchable tmap */
    unsigned int findClosestVertex(const RASM::point2d &target)const;
    unsigned int findClosestCenter(const RASM::point2d &target)const;
    unsigned int findClosestVertex(const RASM::point3d &target)const;
    unsigned int findClosestCenter(const RASM::point3d &target)const;
    int findSurfaceAt(const RASM::point2d &target, RASM::point3d &result)const;
    int findSurfaceAt(const RASM::point2d &target, RASM::point3d &result, int &triangleIndex)const;
    RASM::point3d findClosestSurface(const RASM::point3d &target)const;
    int findTriangle2d(const RASM::point2d &target, bool verbose=false)const;
    void buildTreeVertices();
    void buildTreeCenters();
    void updateTreeVertices(unsigned int n);

    /* from astar tmap */
    int astar(int triangleIndex, int maxSteps, const obstacleMAP &obstacles);
    void evaluatePaths();
    float costToTriangleCenter(const RASM::point2d &p, unsigned int tri);

    void astarReset();
    int getContainingTriangle(const RASM::point2d &p, float &costToCenter) const;
    void writeEvaluationToFile(const char *filename);
    void astarPrint() const{data.astarPrint();}
    bool haveGoal()const;
    void setGoal(const RASM::point2d &goal);
    float readpathcost(int triangleIndex, const obstacleMAP &obstacles);
    float inline readcellcost(int triangleIndex)const
      {return data.readcellcost(triangleIndex);}
    int inline readnextcell(int triangleIndex)const
      {return data.readnextcell(triangleIndex);}

    /* from moveable tmap */
    void translate(const RASM::point2d &amount);
    void translate(const RASM::point3d &amount);
    void rotate(double roll, double pitch, double yaw);
    void translateAndRotate(const RASM::point3d &amount, 
			    double roll, double pitch, double yaw);
    void freeTransform(const float matrix[4][4]);

    /* from icp tmap */
    int icp(const tmap &world, unsigned int maxIterations,
	    RASM::point3d targetPoint, float maxDist,
	    bool shouldFindMean=true,
	    bool shouldRotate=ICP_ROTATION,
	    bool shouldTranslate=ICP_TRANSLATION,
	    bool shouldTranslateXY=ICP_TRANSLATION,
	    bool useSurface=ICP_USE_SURFACE);
    int getTransform(const tmap &world, 
		     float transform[4][4],
		     bool shouldFindMean=true) const;
    void getInteriorAndStrayPoints(const tmap &world,
				   unsigned int *interiorPoints,
				   unsigned int *strayPoints,
				   unsigned int &numInteriorPoints,
				   unsigned int &numStrayPoints,
				   float nearDist, float verticalDist,
				   bool shouldFindMean=1)const;

    /* from group tmap */
    unsigned int markPointGroups(unsigned int numInd,
				 const unsigned int *ind,
				 unsigned int *groups)const;
  }; /* end of class tmap declaration */

} /* end of namespace TMAP */

#endif
