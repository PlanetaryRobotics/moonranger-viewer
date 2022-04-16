#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <tmap_full.h>
#include <util.h>

TMAP::tmap::tmap(){
  printf("[%s:%d] TMAP plain constructor called!\n", __FILE__, __LINE__); fflush(stdout);
  for(unsigned int i=0;i<numValid;i++)
    valid[i]=false;
}

/* new functions */
TMAP::tmap::tmap(const tmap &copy){
  printf("[%s:%d] TMAP copy constructor called!\n", __FILE__, __LINE__); fflush(stdout);
  for(unsigned int i=0;i<numValid;i++)
    valid[i]=false;

  for(unsigned int i=0;i<copy.data.numPoints();i++)
    data.addPoint(copy.data.getPoint(i));

  for(unsigned int i=0;i<copy.data.numTriangles();i++){
    const RASM::triangle t = copy.data.getTriangle(i);
    data.addTriangle(t.points, t.neighbors);
  }

  valid[triangulation]=copy.valid[triangulation];
}

#ifdef USE_PCL

TMAP::tmap::tmap(const pcl::PointCloud<pcl::PointXYZ> &cloud) 
{
  printf("[%s:%d] TMAP PCL constructor called!\n", __FILE__, __LINE__); fflush(stdout);

  data.print();

  for(unsigned int i=0;i<numValid;i++)
    valid[i]=false;

  // TBD: there may be an optimized way to do this...
  for(unsigned int i=0; i < cloud.size(); i++)
    {
      data.addPoint(cloud[i]);
    }

  printf("[%s:%d] cloud has %lu points, tmap has %u points,",
  	 __FILE__, __LINE__,
  	 cloud.size(),
  	 data.numPoints()); fflush(stdout);
  if(data.numPoints() > 0)
    {
      printf(" first is %f, %f, %f\n",
	     RASM_TO_METERS(data.getPoint(0).X()),
	     RASM_TO_METERS(data.getPoint(0).Y()),
	     RASM_TO_METERS(data.getPoint(0).Z()));
    }
}

TMAP::tmap::operator pcl::PointCloud<pcl::PointXYZ> () const
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // TBD: there may be an optimized way to do this...
  for(unsigned int i=0; i < data.numPoints(); i++)
    {
      cloud.push_back(data.getPoint(i));
    }

  return cloud;
}

TMAP::tmap::operator pcl::PointCloud<pcl::PointXYZ>::Ptr () const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // TBD: there may be an optimized way to do this...
  for(unsigned int i=0; i < data.numPoints(); i++)
    {
      cloud->push_back(data.getPoint(i));
    }

  return cloud;
}

#endif // USE_PCL

TMAP::tmap::~tmap(){}

// void TMAP::tmap::evaluateNear(multiPath &arc, obstacleMAP &obstacles, 
// 			      bool extraWide, FILE *savedEvaluation){
//   assertValid(triangulation, true);
//   assertValid(centers,       true);
//   assertValid(neighbors,     true);
//   assertValid(quadVertices,  true);
//   assertValid(quadCenters,   true);

//   printf("[%s:%d %f] calling tmap::evaluateNear()\n",
// 	 __FILE__, __LINE__, now());

//   arc.evaluateNear(data, obstacles, extraWide, savedEvaluation);
// }

// void TMAP::tmap::evaluateFar(multiPath &arc, const obstacleMAP &obstacles, 
// 			     bool extraWide, FILE *savedEvaluation,
// 			     bool savePaths){
//   assertValid(triangulation, true);
//   assertValid(centers,       true);
//   assertValid(neighbors,     true);
//   assertValid(quadVertices,  true);
//   assertValid(quadCenters,   true);
//   assertValid(astarGoal,     true);
//   arc.evaluateFar(data, obstacles, extraWide, savedEvaluation, savePaths);
// }

/* like paste() but only removes duplicate points
 * and leaves the current model untouched,
 * puts the union into a new model
 */
void TMAP::tmap::combineInto(TMAP::tmap &combined_result,
			     const TMAP::tmap &new_model) const
{
  /*
   * Clear out combined result.
   */
  combined_result.clearPointsAndTriangles();

  /*
   * Are there any points in the current model? 
   */
  if(this->numPoints()>0)
    {
      /* 
       * If so, then copy all points from the current model into the 
       * combined result.
       */
      for(unsigned int i=0; i < this->numPoints(); i++)
	{
	  /*
	   * TBD: what is this check for?
	   */
	  if( i != this->findClosestVertex(RASM::point2d(getPoint(i)))) continue;

	  combined_result.addPoint(this->getPoint(i));
	}

      /* 
       * Now iterate through the points in new_model.
       * Skip points that are duplicated in either combined_result or the current model.
       */
      for(unsigned int i=0; i < new_model.numPoints(); i++)
	{
	  /*
	   * "new_model_pt" is the vertex coordinate we're checking in the new model.
	   * "new_model_pt_2D" is its projection in two dimensions.
	   */
	  RASM::point3d new_model_pt = new_model.getPoint(i);
	  RASM::point2d new_model_pt_2D(new_model_pt);

	  /*
	   * index_current_model is the index of the vertex in the current model that lies closest 
	   * to new_model_pt_2D.
	   *
	   * index_new_model is the index of the vertex in the new model that lies closest to 
	   * new_model_pt_2D.
	   *
	   * TBD: why aren't we calling findClosestVertex with new_model_pt_2D?
	   */
	  unsigned int index_current_model = this->findClosestVertex(RASM::point2d(new_model_pt));
	  unsigned int index_new_model = new_model.findClosestVertex(RASM::point2d(new_model_pt));

	  /*
	   * Check to see if vertex 'i' in the new model is a duplicate of another vertex in either the
	   * existing model or in the current model.
	   */
	  if((i != index_new_model) ||                                          /* matches some other vertex in new_model */
	     (new_model_pt_2D == RASM::point2d(getPoint(index_current_model)))) /* matches a vertex in current model */
	    {
	      /*
	       * If it is a duplicate, then drop it and print a warning.
	       */
	      //	      printf("Warning, when combining models dropping point %d ", i);
	      //	      new_model_pt.print();
	      //	      printf("\n");
	      //	      printf("It matches existing point A %d ", index_current_model);
	      //	      getPoint(index_current_model).print();
	      //	      printf(" or point B %d ", index_new_model);
	      //	      new_model.getPoint(index_new_model).print();
	      //	      printf("\n");
	      //	      writeToFile("combineA.smf");
	      //	      new_model.writeToFile("combineNewModel.smf");
	      //	      combined_result.writeToFile("combineResult.smf");
	    }
	  else
	    {
	      /* 
	       * Otherwise, this is a new unique point that should be added 
	       * to the combined result.
	       */
	      combined_result.addPoint(new_model_pt);
	    }
	}

      /* 
       * Now triangulate the combined result, and remove points that aren't used in the triangulated mesh.
       */
      combined_result.triangulateMinimal();
      combined_result.maskBadData(TMAP::tmap_cleanable::unusedPoints /* this is the 'clean-up' metric */);
      combined_result.removeBadData();

    } // there are vertices in the current model
  else
    {
      /*
       * If there are no points in the current model then simply copy the
       * new model into the "combined" result.
       */
      for(unsigned int i=0; i < new_model.numPoints(); i++) combined_result.addPoint(new_model.getPoint(i));
    }

  /*
   * Prepare the combined result for usage.
   */
  combined_result.buildTreeVertices();
  combined_result.errorCheck();
}


void TMAP::tmap::rebuild(){
  if(0 == numPoints())return;
  if(!valid[triangulation])  triangulate();
  if(!valid[centers])        fillInCenters();
  if(!valid[associations])   fillInAssociations();
  if(!valid[neighbors])      fillInNeighbors();
  if(!valid[quadVertices])   buildTreeVertices();
  if(!valid[quadCenters])    buildTreeCenters();
  assert(data.haveTreeOfVertices());
  assert(data.haveTreeOfCenters());
}

void TMAP::tmap::tearDown(){
  valid[triangulation]       =false;
  if(valid[centers])         removeCenters();
  if(valid[associations])    removeAssociations();
  if(valid[neighbors])       removeNeighbors();
  valid[quadVertices]        =false;
  valid[quadCenters]         =false;
  valid[astarGoal]           =false;
}

void TMAP::tmap::showValid() const{
#define SHOW_VALID(i) do{printf(" %d %s %c\n", i, #i, (valid[i]?'Y':'N'));}while(0)
  SHOW_VALID(triangulation);
  SHOW_VALID(centers);
  SHOW_VALID(associations);
  SHOW_VALID(neighbors);
  SHOW_VALID(quadVertices);
  SHOW_VALID(quadCenters);
  SHOW_VALID(astarGoal);
#undef SHOW_VALID
}

#if CHECK_VALID_BITS
void TMAP::tmap::assertValid(VALID_BITS n, bool validState) const{
  if(n<0 || n>=numValid){
    for(unsigned int i=0;i<numValid;i++){
      printf("Error, expected valid[%d] to be %d\n", n, validState);
      showValid();
      assert(0);
    }
  }else{
    if(validState == valid[n])
      return;
    printf("Error, expected valid[%d] to be %d\n", n, validState);
    showValid();
    assert(0);
  }
}
#endif

void TMAP::tmap::setValidTriangulation(){
  valid[triangulation]       =true;
}

/* from tmap */
void TMAP::tmap::readFromFile(const char *file){
  tearDown();
  data.readFromFile(file);
  if(strstr(file, ".obj") || strstr(file, ".smf"))
    valid[triangulation]     =true;
}

void TMAP::tmap::fillInCenters(){
  assertValid(centers,       false);
  valid[centers]             =true;
  data.tmap_cleanable::fillInCenters();
}

void TMAP::tmap::removeCenters(){
  assertValid(centers,       true);
  valid[centers]             =false;
  data.tmap_cleanable::removeCenters();
}

void TMAP::tmap::fillInAssociations(){
  assertValid(associations,  false);
  valid[associations]        =true;
  data.tmap_cleanable::fillInAssociations();
}

void TMAP::tmap::removeAssociations(){
  assertValid(associations,  true);
  valid[associations]        =false;
  data.tmap_cleanable::removeAssociations();
}

void TMAP::tmap::fillInNeighbors(){
  assertValid(neighbors,     false);
  valid[neighbors]           =true;
  data.tmap_cleanable::fillInNeighbors();
  checkNeighbors();
}

void TMAP::tmap::removeNeighbors(){
  assertValid(neighbors,     true);
  valid[neighbors]           =false;
  data.removeNeighbors();
}

void TMAP::tmap::addPoint(const RASM::point3d &point,
			  bool invalidate){
  if(invalidate)tearDown();
  data.addPoint(point);
}

void TMAP::tmap::addInterestPoint(const RASM::point3d &point,
			  bool invalidate){
  if(invalidate)tearDown();
  data.addInterestPoint(point);
}

void TMAP::tmap::addTriangle(const unsigned int tri[3],
			     const int neighbors[3],
			     bool invalidate){
  if(invalidate)tearDown();
  data.addTriangle(tri, neighbors);
}

void TMAP::tmap::removePoint(unsigned int ind,
			     bool invalidate){
  if(invalidate)tearDown();
  data.removePoint(ind);
}

void TMAP::tmap::removeInterestPoint(unsigned int ind,
			     bool invalidate){
  if(invalidate)tearDown();
  data.removeInterestPoint(ind);
}

void TMAP::tmap::replacePoint(unsigned int ind,
			      const RASM::point3d &point,
			      bool invalidate){
  if(invalidate)tearDown();
  data.replacePoint(ind, point);
}

void TMAP::tmap::replaceInterestPoint(unsigned int ind,
			      const RASM::point3d &point,
			      bool invalidate){
  if(invalidate)tearDown();
  data.replaceInterestPoint(ind, point);
}

void TMAP::tmap::replaceTriangle(unsigned int ind,
				 const unsigned int tri[3],
				 const int neighbors[3],
				 bool invalidate){
  if(invalidate)tearDown();
  data.replaceTriangle(ind, tri, neighbors);
}

const RASM::point3d &TMAP::tmap::getPoint(unsigned int ind) const{
  assert(ind<numPoints());
  return data.getPoint(ind);
}

const RASM::triangle &TMAP::tmap::getTriangle(unsigned int ind) const{
  assert(ind<numTriangles());
  return data.getTriangle(ind);
}

void TMAP::tmap::clearPointsAndTriangles(){
  tearDown();
  data.tmap_cleanable::clearPointsAndTriangles();
}

/* from cleanable tmap */
void TMAP::tmap::removeBadData(){
  unsigned int preVertices   = data.numPoints();
  unsigned int preTriangles  = data.numTriangles();
  data.removeBadData();
  unsigned int postVertices  = data.numPoints();
  unsigned int postTriangles = data.numTriangles();

  /* adjust the valid mask as necessary */
  //triangulation should be preserved
  //centers were recreated if needed
  valid[neighbors]           =true;//neighbors were filled in
  //associations were recreated if needed

  //if any vertices were removed, then the quad is not valid
  if(preVertices != postVertices)
    valid[quadVertices]      =false;

  //if any triangles were removed, then the quad is not valid
  if(preTriangles != postTriangles)
    valid[quadCenters]       =false;

  valid[astarGoal]           =false;//goal is invalid
}

/* from shrinkable tmap */
void TMAP::tmap::shrink(bool decimate, unsigned int finalSize,
			bool retriangulate){
  if(data.numPoints()<3)return;

  if(decimate){
    assert(data.numPoints()>=0);
  }else{
    assertValid(triangulation, true);
    assert(data.numTriangles()>0);
  }

  /* note: this marks the triangulation invalid but does not destroy it
   * shrink will still use the existing triangulation
   * if decimate was set, then the data need not be triangulated already
   * as the end of the decimatation step must retriangulate anyway
   */
  tearDown();

  data.shrink(decimate, finalSize, false/* retriangulate manually below */);

  /* note: shrink always ends with some sort of triangulation
   * unless it was explicitly retriangulated, 
   * that may not be quite good enough
   */
  if(retriangulate && (numPoints()>=3)){
    triangulate();
    checkNeighbors();

    fillInAssociations();
    //fillInCenters();//this is done in TMAP::tmap::triangulate()
  }
}


/* from triangulate tmap */
void TMAP::tmap::triangulateMinimal(){
  tearDown();
  data.triangulateMinimal();
  valid[triangulation]       =true;
  valid[neighbors]           =true;
  checkNeighbors();
}

void TMAP::tmap::triangulate(){
  tearDown();
  data.triangulate();
  valid[triangulation]       =true;
  valid[neighbors]           =true;
  checkNeighbors();
  fillInCenters();
}

void TMAP::tmap::checkNeighbors() const{
  assertValid(neighbors,     true);
  //assert(-1 != data.checkNeighbors());
}

/* from pasteable tmap */
void TMAP::tmap::paste(const TMAP::tmap &newmap,
		       const float tooClose){
  if(newmap.numPoints()==0){
    triangulateMinimal();
    fillInCenters();
    return;
  }

  if(numPoints()>0){
    newmap.assertValid(triangulation, true);
    newmap.assertValid(centers,       true);
    newmap.assertValid(associations,  true);
    newmap.assertValid(neighbors,     true);
    newmap.assertValid(quadVertices,  true);
    newmap.assertValid(quadCenters,   true);
    assert(newmap.data.haveTreeOfCenters());
  }

  newmap.checkNeighbors();

  if(valid[associations])
    removeAssociations();

  if(valid[centers])
    removeCenters();

  data.paste(newmap.data, tooClose);

  valid[triangulation]       =true;
  valid[neighbors]           =true;
  valid[quadVertices]        =false;
  valid[quadCenters]         =false;
  valid[astarGoal]           =false;
}

/* from searchable tmap */
unsigned int TMAP::tmap::findClosestVertex(const RASM::point2d &target)const{
  assertValid(quadVertices,  true);
  return data.findClosestVertex(target);
}

unsigned int TMAP::tmap::findClosestCenter(const RASM::point2d &target)const{
  assertValid(centers,       true);
  assertValid(quadCenters,   true);
  return data.findClosestCenter(target);
}

unsigned int TMAP::tmap::findClosestVertex(const RASM::point3d &target)const{
  assertValid(quadVertices,  true);
  return data.findClosestVertex(target);
}

unsigned int TMAP::tmap::findClosestCenter(const RASM::point3d &target)const{
  assertValid(centers,       true);
  assertValid(quadCenters,   true);
  return data.findClosestCenter(target);
}

int TMAP::tmap::findSurfaceAt(const RASM::point2d &target,
			      RASM::point3d &result)const{
  assertValid(triangulation, true);
  assertValid(centers,       true);
  assertValid(associations,  true);
  assertValid(neighbors,     true);
  assertValid(quadVertices,  true);
  assertValid(quadCenters,   true);
  return data.findSurfaceAt(target, result);
}

int TMAP::tmap::findSurfaceAt(const RASM::point2d &target,
			      RASM::point3d &result,
			      int &triangleIndex)const{
  assertValid(triangulation, true);
  assertValid(centers,       true);
  assertValid(associations,  true);
  assertValid(neighbors,     true);
  assertValid(quadVertices,  true);
  assertValid(quadCenters,   true);
  return data.findSurfaceAt(target, result, triangleIndex);
}

RASM::point3d TMAP::tmap::findClosestSurface(const RASM::point3d &target)const{
  assertValid(triangulation, true);
  assertValid(centers,       true);
  assertValid(associations,  true);
  assertValid(neighbors,     true);
  assertValid(quadVertices,  true);
  assertValid(quadCenters,   true);
  return data.findClosestSurface(target);
}

int TMAP::tmap::findTriangle2d(const RASM::point2d &target, bool verbose)const{
  assertValid(triangulation, true);
  assertValid(centers,       true);
  assertValid(associations,  true);
  assertValid(neighbors,     true);
  assertValid(quadVertices,  true);
  assertValid(quadCenters,   true);
  return data.findTriangle2d(target, verbose);
}

void TMAP::tmap::buildTreeVertices(){
  assertValid(quadVertices,  false);
  data.buildTreeVertices();
  assert(data.haveTreeOfVertices());
  valid[quadVertices]        =true;
}

void TMAP::tmap::buildTreeCenters(){
  assertValid(centers,       true);
  assertValid(quadCenters,   false);
  data.buildTreeCenters();
  assert(data.haveTreeOfCenters());
  valid[quadCenters]         =true;
}

void TMAP::tmap::updateTreeVertices(unsigned int n){
  assertValid(quadVertices,  true);
  data.updateTreeVertices(n);
}

/* from astar tmap */
int TMAP::tmap::astar(int triangleIndex, int maxSteps,
		      const obstacleMAP &obstacles){
  assertValid(numValid,      true);
  return data.astar(triangleIndex, maxSteps, obstacles);
}

void TMAP::tmap::evaluatePaths(){
  assertValid(numValid,      true);
  return data.evaluatePaths();
}

float TMAP::tmap::costToTriangleCenter(const RASM::point2d &p, unsigned int tri)
{
  return data.costToTriangleCenter(p, tri);
}

void TMAP::tmap::astarReset(){
  data.astarReset();
}

int TMAP::tmap::getContainingTriangle(const RASM::point2d &p,
				      float &costToCenter)const{
  assertValid(triangulation, true);
  assertValid(centers,       true);
  assertValid(associations,  true);
  assertValid(neighbors,     true);
  assertValid(quadVertices,  true);
  assertValid(quadCenters,   true);
  return data.getContainingTriangle(p, costToCenter);
}

void TMAP::tmap::writeEvaluationToFile(const char *filename){
  data.writeEvaluationToFile(filename);
}

bool TMAP::tmap::haveGoal()const{return valid[astarGoal];}
void TMAP::tmap::setGoal(const RASM::point2d &goal){
  assertValid(triangulation, true);
  assertValid(centers,       true);
  assertValid(associations,  true);
  assertValid(neighbors,     true);
  assertValid(quadVertices,  true);
  assertValid(quadCenters,   true);
  assertValid(astarGoal,     false);
  valid[astarGoal]           =true;
  data.setGoal(goal);
}

float TMAP::tmap::readpathcost(int triangleIndex,
			       const obstacleMAP &obstacles){
  assertValid(astarGoal,      true);
  return data.readpathcost(triangleIndex, obstacles);
}


/* from moveable tmap */
void TMAP::tmap::translate(const RASM::point2d &amount){
  if(valid[centers])removeCenters();
  valid[quadVertices]        =false;
  valid[quadCenters]         =false;
  valid[astarGoal]           =false;
  data.translate(amount);
}

void TMAP::tmap::translate(const RASM::point3d &amount){
  if(valid[centers])removeCenters();
  valid[quadVertices]        =false;
  valid[quadCenters]         =false;
  valid[astarGoal]           =false;
  data.translate(amount);
}

void TMAP::tmap::rotate(double roll, double pitch, double yaw){
  tearDown();
  data.rotate(roll, pitch, yaw);
}

void TMAP::tmap::translateAndRotate(const RASM::point3d &amount, 
				    double roll, double pitch, double yaw){
  tearDown();
  data.translateAndRotate(amount, roll, pitch, yaw);
}

void TMAP::tmap::freeTransform(const float matrix[4][4]){
  tearDown();
  data.freeTransform(matrix);
}

/* from icp tmap */
int TMAP::tmap::icp(const tmap &world, unsigned int maxIterations,
		    RASM::point3d targetPoint, float maxDist,
		    bool shouldFindMean, bool shouldRotate,
		    bool shouldTranslate, bool shouldTranslateXY,
		    bool useSurface){
  world.assertValid(quadVertices, true);
  if(useSurface){
    world.assertValid(triangulation, true);
    world.assertValid(centers,       true);
    world.assertValid(associations,  true);
    world.assertValid(neighbors,     true);
    world.assertValid(quadCenters,   true);
  }

  int ret = data.icp(world.data, maxIterations, targetPoint, maxDist,
                     shouldFindMean, shouldRotate, shouldTranslate,
		     shouldTranslateXY, NULL, useSurface);
  tearDown();
  return ret;
}

int TMAP::tmap::getTransform(const tmap &world,
			     float transform[4][4],
			     bool shouldFindMean) const{
  world.assertValid(triangulation, true);
  world.assertValid(centers,       true);
  world.assertValid(associations,  true);
  world.assertValid(neighbors,     true);
  world.assertValid(quadVertices,  true);
  world.assertValid(quadCenters,   true);

  return data.getTransform(world.data, transform, shouldFindMean);
}

void TMAP::tmap::getInteriorAndStrayPoints(const tmap &world,
					   unsigned int *interiorPoints,
					   unsigned int *strayPoints,
					   unsigned int &numInteriorPoints,
					   unsigned int &numStrayPoints,
					   float nearDist, float verticalDist,
					   bool shouldFindMean)const{

  assertValid(associations,  true);
  world.assertValid(triangulation, true);
  world.assertValid(centers,       true);
  world.assertValid(associations,  true);
  world.assertValid(neighbors,     true);
  world.assertValid(quadVertices,  true);
  world.assertValid(quadCenters,   true);

  return data.getInteriorAndStrayPoints(world.data,
					interiorPoints,    strayPoints,
					numInteriorPoints, numStrayPoints,
					nearDist, verticalDist,
					shouldFindMean);
}


/* from group tmap */
unsigned int TMAP::tmap::markPointGroups(unsigned int numInd,
					 const unsigned int *ind,
					 unsigned int *groups)const{

  assertValid(triangulation, true);
  assertValid(associations,  true);

  return data.markPointGroups(numInd, ind, groups);
}
