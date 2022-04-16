#include <tmap_shrinkable.h>
//#include <stdmix.h>
//#include <mixio.h>
//#include <MxQSlim.h>
#include <opencv2/opencv.hpp>
#include <assert.h>
#include <sys/time.h>
#include <math.h>
#include <decimationGrid.h>
#include <fstream>
#include <igl/decimate.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <util.h>

/* grid size and resolution in meters */
static const double tridarNearCutoff = 1.0;
static const double tridarNearWidth = 3.0;
static const double tridarNearDepth = 5.0;
static const double tridarNearRes = 0.1;

/* grid size and resolution in meters */
static const double zoeNavNearCutoff = 2.3;
static const double zoeNavNearWidth = 5.0;
static const double zoeNavNearDepth = 5.0;
static const double zoeNavNearRes = 0.1;

/* another grid is used for points beyond tridarNearDepth
 * those can either be decimated or blindly kept if the flag is set
 */
static const double tridarFarWidth = 4.0;
static const double tridarFarDepth = 30.0;
static const double tridarFarRes = 0.1;
static const int tridarKeepAllFarPoints=1;

static const double zoeNavFarWidth = 12.0;
static const double zoeNavFarDepth = 12.0;
static const double zoeNavFarRes = 0.1;
static const int zoeNavKeepAllFarPoints=0;

static const double zoeMaxAlt = 1.1;

void TMAP::tmap_shrinkable::tridarDecimate(){
  const RASM::point2d tmp1(METERS_TO_RASM(-tridarNearWidth),
			   METERS_TO_RASM(tridarNearCutoff));
  const RASM::point2d tmp2(METERS_TO_RASM(tridarNearWidth),
			   METERS_TO_RASM(tridarNearDepth));
  const RASM::bounds2d tridarNearBounds(tmp1, tmp2);

  const RASM_UNITS tridarNearSpacing(METERS_TO_RASM(tridarNearRes));

  const RASM::point2d tmp3(METERS_TO_RASM(-tridarFarWidth), 
			   METERS_TO_RASM(tridarNearDepth));
  const RASM::point2d tmp4(METERS_TO_RASM(tridarFarWidth),
			   METERS_TO_RASM(tridarFarDepth));
  const RASM::bounds2d tridarFarBounds(tmp3, tmp4);

  const RASM_UNITS tridarFarSpacing(METERS_TO_RASM(tridarFarRes));

  /* not const, but only need to initialize this once
   * (ie, only need to allocate buffer space once)
   */
  static TMAP::decimationGrid dNear(tridarNearBounds, tridarNearSpacing);
  static TMAP::decimationGrid dFar (tridarFarBounds,  tridarFarSpacing);

  unsigned int nearN = dNear.maxPoints();
  unsigned int farN  = dFar.maxPoints();
  if(tridarKeepAllFarPoints)
    farN=numPoints();

  RASM::point3d *nearPoints = (RASM::point3d *)malloc(nearN*sizeof(RASM::point3d));
  RASM::point3d *farPoints  = (RASM::point3d *)malloc(farN*sizeof(RASM::point3d));

  RASM::point3d *nearCopy   = (RASM::point3d *)malloc(numPoints()*sizeof(RASM::point3d));
  RASM::point3d *farCopy    = (RASM::point3d *)malloc(numPoints()*sizeof(RASM::point3d));

  unsigned int numNear=0, numFar=0;
  for(unsigned int i=0;i<numPoints();i++){
    if(tridarNearBounds.contains(vertices[i]))
      nearCopy[numNear++] = vertices[i];
    else if(tridarFarBounds.contains(vertices[i]))
      farCopy[numFar++] = vertices[i];
  }

  dNear.processPoints(nearCopy, numNear, nearPoints, nearN);
  if(tridarKeepAllFarPoints){
    farN=numFar;
    memcpy(farPoints, farCopy, numFar*sizeof(RASM::point3d));
  }else{
    dFar.processPoints(farCopy, numFar, farPoints, farN);
  }

  assert(nearN + farN < numPoints());

  /* move all the points */
  resizeVertices(0);
  //  sizeVertices=0;
  for(unsigned int i=0;i<nearN;i++)
    {
      addPoint(nearPoints[i]);
      //      vertices[sizeVertices++]=nearPoints[i];
    }

  for(unsigned int i=0;i<farN;i++)
    {
      addPoint(farPoints[i]);
      //      vertices[sizeVertices++]=farPoints[i];
    }

  /*
  printf("decimated to %d near + %d far = %d vertices\n",
	 nearN, farN, numPoints());
  */

  /* retriangulate the resulting points */
  triangulateMinimal();

  //printf("decimated to %d vertices and %d faces\n", numPoints(), numTriangles());

  free(nearPoints);
  free(farPoints);
  free(nearCopy);
  free(farCopy);
}

// TODO: Why is this referencing zoe?
void TMAP::tmap_shrinkable::zoeNavDecimate(){
  const RASM::point2d tmp1(METERS_TO_RASM(-zoeNavNearWidth), 
			   METERS_TO_RASM(zoeNavNearCutoff));
  const RASM::point2d tmp2(METERS_TO_RASM(zoeNavNearWidth),
			   METERS_TO_RASM(zoeNavNearDepth));
  const RASM::bounds2d zoeNavNearBounds(tmp1, tmp2);

  const RASM_UNITS zoeNavNearSpacing(METERS_TO_RASM(zoeNavNearRes)); 

  const RASM::point2d tmp3(METERS_TO_RASM(-zoeNavFarWidth), 
			  METERS_TO_RASM(zoeNavNearDepth));
  const RASM::point2d tmp4(METERS_TO_RASM(zoeNavFarWidth),
			   METERS_TO_RASM(zoeNavFarDepth));
  const RASM::bounds2d zoeNavFarBounds(tmp3, tmp4);

  const RASM_UNITS zoeNavFarSpacing(METERS_TO_RASM(zoeNavFarRes));

  /* not const, but only need to initialize this once
   * (ie, only need to allocate buffer space once)
   */
  static TMAP::decimationGrid dNear(zoeNavNearBounds, zoeNavNearSpacing);
  static TMAP::decimationGrid dFar (zoeNavFarBounds,  zoeNavFarSpacing);

  /* max number of near and far points */
  unsigned int nearN = dNear.maxPoints();
  unsigned int farN  = numPoints();

  /* buffer that will definitely hold all the points */
  RASM::point3d *nearPoints = (RASM::point3d *)malloc(nearN*sizeof(RASM::point3d));
  RASM::point3d *farPoints  = (RASM::point3d *)malloc(farN*sizeof(RASM::point3d));

  RASM::point3d *nearCopy   = (RASM::point3d *)malloc(numPoints()*sizeof(RASM::point3d));
  RASM::point3d *farCopy    = (RASM::point3d *)malloc(numPoints()*sizeof(RASM::point3d));

  unsigned int numNear=0, numFar=0;
  for(unsigned int i=0;i<numPoints();i++){
    if(zoeMaxAlt>0.0 && RASM_TO_METERS(vertices[i].Z()) > zoeMaxAlt)
      continue;/* skip points above the max altitude */

    if(zoeNavNearBounds.contains(vertices[i]))
      nearCopy[numNear++] = vertices[i];
    else if(zoeNavFarBounds.contains(vertices[i]))
      farCopy[numFar++] = vertices[i];
  }

  dNear.processPoints(nearCopy, numNear, nearPoints, nearN,
		      1/* do apply min points in cell */,
		      0/* don't apply mean altitude within cell */,
		      1/* do apply mean altitude among cells */,
		      0/* don't apply mean altitude among cell neighbors */,
		      1/* do take output from means */);

  if(zoeNavKeepAllFarPoints){
    farN=numFar;
    memcpy(farPoints, farCopy, numFar*sizeof(RASM::point3d));
  }else{
    dFar.processPoints(farCopy, numFar, farPoints, farN,
		       0/* don't apply min points in cell */,
		       1/* do apply mean altitude within cell */,
		       0/* do not apply mean altitude among cells */,
		       1/* do apply mean altitude among cell neighbors */,
		       1/* do take output from means */);
  }

  assert(nearN + farN <= numPoints());
  assert(nearN <= numNear);
  assert(farN <= numFar);

  /* move all the points */
  //  sizeVertices=0;
  resizeVertices(0);
  for(unsigned int i=0;i<nearN;i++)
    {
      addPoint(nearPoints[i]);
      //      vertices[sizeVertices++]=nearPoints[i];
    }

  for(unsigned int i=0;i<farN;i++)
    {
      addPoint(farPoints[i]);
      //      vertices[sizeVertices++]=farPoints[i];
    }

  /*
  printf("decimated from %d near + %d far to %d near + %d far = %d vertices\n",
	 numNear, numFar, nearN, farN, numPoints());
  */

  /* retriangulate the resulting points */
  if(numPoints() >= 3)
    triangulateMinimal();

  //printf("decimated to %d vertices and %d faces\n", numPoints(), numTriangles());

  free(nearPoints);
  free(farPoints);
  free(nearCopy);
  free(farCopy);
}

// Calculates the best way to split the data based on the covariance matrix
void TMAP::tmap_shrinkable::covarianceDecimate(unsigned int maxGroupSize, unsigned int maxGroupVariance) {
  cv::Mat allPoints = cv::Mat(numPoints(), 3, CV_32F);

  // Build the matrix of vertices
  for (unsigned int i = 0; i < numPoints(); i++) {
    allPoints.at<float>(i,0) = vertices[i].X();
    allPoints.at<float>(i,1) = vertices[i].Y();
    allPoints.at<float>(i,2) = vertices[i].Z();
  }

  // Split the matrix recursively by the plane of greatest variance
  covarianceSplitPoints(allPoints, maxGroupSize, maxGroupVariance);

  // Rebuild the tmap vertices array
  cv::Size size = allPoints.size();
  unsigned int originalVertices = numPoints();
  //  sizeVertices = size.height;
  resizeVertices(size.height);

  for (unsigned int i = 0; i < numPoints(); i++) {
    RASM::point3d temp(floor(allPoints.at<float>(i,0) + 0.5), floor(allPoints.at<float>(i,1) + 0.5), floor(allPoints.at<float>(i,2) + 0.5));
    vertices[i] = temp;
  }

  printf("decimated from %d to %d vertices\n", originalVertices, numPoints());

  triangulateMinimal();

  printf("decimated to %d vertices and %d faces\n", numPoints(), numTriangles());

  printf("done shrinking\n");
}

// Splits the given matrix of points by variance, recursing until each group of points is small enough
//  or has low enough variance. 
void TMAP::tmap_shrinkable::covarianceSplitPoints(cv::Mat& points, unsigned int maxGroupSize, unsigned int maxGroupVariance) {
  cv::Size size = points.size();
  
  // Split the points then find which group each points belongs to
  int numpts = numPoints();
  cv::PCA pca(points, cv::Mat(), 0 /*CV_PCA_DATA_AS_ROW*/, numpts);
  
  bool* pointGroup = (bool *)malloc(size.height*sizeof(bool));	
  
  for (int i = 0; i < size.height; i++) {
    cv::Mat projectedMat = cv::Mat(1, 3, CV_32F);
  
    pca.project(points.row(i), projectedMat);
    pointGroup[i] = (projectedMat.at<float>(0, 0) < 0);
  }
  
  unsigned int group1Count = 0;
  unsigned int group2Count = 0;
  
  for (int i = 0; i < size.height; i++) {
    if (pointGroup[i] == 0) {
      group1Count++;
    }
    else {
      group2Count++;
    }
  }
  
  cv::Mat group1(group1Count, 3, CV_32F);
  cv::Mat group2(group2Count, 3, CV_32F);
  
  unsigned int group1Index = 0;
  unsigned int group2Index = 0;
  float group1XMean = 0;
  float group1YMean = 0;
  float group1ZMean = 0;
  float group2XMean = 0;
  float group2YMean = 0;
  float group2ZMean = 0;

  // Split the data into the two most distinct groups based on PCA results
  for (int i = 0; i < size.height; i++) {
    if (pointGroup[i] == 0) {
      cv::Mat tmp = group1.row(group1Index);
      points.row(i).copyTo(tmp);
      group1XMean += points.at<float>(i, 0);
      group1YMean += points.at<float>(i, 1);
      group1ZMean += points.at<float>(i, 2);
      group1Index++;
    }
    else {
      cv::Mat tmp = group2.row(group2Index);
      points.row(i).copyTo(tmp);
      group2XMean += points.at<float>(i, 0);
      group2YMean += points.at<float>(i, 1);
      group2ZMean += points.at<float>(i, 2);
      group2Index++;
    }
  }

  // Compute the variance of each group to see if we have to split further
  group1XMean = group1XMean/group1Index;
  group1YMean = group1YMean/group1Index;
  group1ZMean = group1ZMean/group1Index;
  group2XMean = group2XMean/group2Index;
  group2YMean = group2YMean/group2Index;
  group2ZMean = group2ZMean/group2Index;

  float group1Variance = 0;
  float group2Variance = 0;

  for (unsigned int i = 0; i < group1Index; i++) {
    float temp = group1.at<float>(i,2) - group1ZMean;
    group1Variance += temp*temp;
  }
  group1Variance = group1Variance/(group1Index-1);

  for (unsigned int i = 0; i < group2Index; i++) {
    float temp = group2.at<float>(i,2) - group2ZMean;
    group2Variance += temp*temp;
  }
  group2Variance = group2Variance/(group2Index-1);

  // Recursively split this subset of points if there are too many in the group or the variance
  //  is too high.
  if (group1Count > maxGroupSize && group1Variance > maxGroupVariance) {
    covarianceSplitPoints(group1, maxGroupSize, maxGroupVariance);
  }
  else {
    group1 = cv::Mat(1, 3, CV_32F);
    group1.at<float>(0,0) = group1XMean;
    group1.at<float>(0,1) = group1YMean;
    group1.at<float>(0,2) = group1ZMean;
  }

  if (group2Count > maxGroupSize && group2Variance > maxGroupVariance) {
    covarianceSplitPoints(group2, maxGroupSize, maxGroupVariance);
  }
  else {
    group2 = cv::Mat(1, 3, CV_32F);
    group2.at<float>(0,0) = group2XMean;
    group2.at<float>(0,1) = group2YMean;
    group2.at<float>(0,2) = group2ZMean;
  }

  // Combine and return the two groups
  cv::Size size1 = group1.size();
  cv::Size size2 = group2.size();

  points = cv::Mat(size1.height + size2.height, 3, CV_32F);

  for (int i = 0; i < size1.height + size2.height; i++) {
    cv::Mat tmp = points.row(i);

    if (i < size1.height) {
      group1.row(i).copyTo(tmp);
    }
    else {
      int index = i - size1.height;
      group2.row(index).copyTo(tmp);
    }
  }

  free(pointGroup);
}

void TMAP::tmap_shrinkable::customDecimate(double nearCutoff, double nearWidth,
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
  const RASM::point2d tmp1(METERS_TO_RASM(-nearWidth), 
			   METERS_TO_RASM(nearCutoff));
  const RASM::point2d tmp2(METERS_TO_RASM(nearWidth),
			   METERS_TO_RASM(nearDepth));
  const RASM::bounds2d nearBounds(tmp1, tmp2);

  const RASM_UNITS nearSpacing(METERS_TO_RASM(nearRes));

  const RASM::point2d tmp3(METERS_TO_RASM(-farWidth), 
			   METERS_TO_RASM(nearDepth));
  const RASM::point2d tmp4(METERS_TO_RASM(farWidth),
			   METERS_TO_RASM(farDepth));
  const RASM::bounds2d farBounds(tmp3, tmp4);

  const RASM_UNITS farSpacing(METERS_TO_RASM(farRes));

  if(keepAllNear && keepAllFar)
    {
      /*
       * Here is a special case; just do nothing.
       */
      return;
    }

  /* not const, but only need to initialize this once
   * (ie, only need to allocate buffer space once)
   */
  /*static*/ TMAP::decimationGrid dNear(nearBounds, nearSpacing);
  /*static*/ TMAP::decimationGrid dFar (farBounds,  farSpacing);

  /* max number of near and far points */
  unsigned int nearN = dNear.maxPoints();
  //unsigned int farN  = numPoints();
  unsigned int farN = dFar.maxPoints();
  printf("[%s:%d %f] farN numpoints %d \n", __FILE__, __LINE__, now(),farN);
  /* buffer that will definitely hold all the points */
  RASM::point3d *nearCopy   = (RASM::point3d *)malloc(numPoints()*sizeof(RASM::point3d));
  RASM::point3d *farCopy    = (RASM::point3d *)malloc(numPoints()*sizeof(RASM::point3d));

  assert(NULL != nearCopy &&
	 NULL != farCopy);

  unsigned int numNear=0, numFar=0;
  unsigned int numAboveMaxAlt=0, numBelowMinAlt=0, numOutsideBounds=0;

  // std::ofstream f1,f2,f3;
  // f1.open("filtered_points.asc");
  // f2.open("max_height_points.asc");
  // f3.open("all_points.asc");

  for(unsigned int i=0;i<numPoints();i++){
    // f3 << vertices[i].X()  << ", " << vertices[i].Y() << ", " << vertices[i].Z() << std::endl;
    if(/*maxAlt>0.0 && */RASM_TO_METERS(vertices[i].Z()) > maxAlt) {
      // f2 << vertices[i].X() << ", " << vertices[i].Y() << ", " << vertices[i].Z() << std::endl;
      numAboveMaxAlt++;
      continue;/* skip points above the max altitude */
    }

    if(/*minAlt<0.0 && */RASM_TO_METERS(vertices[i].Z()) < minAlt) {
      numBelowMinAlt++;
      continue;/* skip points below the min altitude */
    }
    //printf("Considering vertex: x:%4.5f y:%4.5f z:%4.5f\n",vertices[i].X(),vertices[i].Y(),vertices[i].Z());
    if(nearBounds.contains(vertices[i]))
      nearCopy[numNear++] = vertices[i];
    else if(farBounds.contains(vertices[i]))
      farCopy[numFar++] = vertices[i];
    else {
      // f1 << vertices[i].X() << ", " << vertices[i].Y() << ", " << vertices[i].Z() << std::endl;
      numOutsideBounds++;
    }
  }

  // f1.close();
  // f2.close();
  // f3.close();
  printf("Debug: Total: %d Above max alt: %d Outside bounds: %d\n",numPoints(), numAboveMaxAlt, numOutsideBounds);
  printf("Debug: Total: %d Below min alt: %d Outside bounds: %d\n",numPoints(), numBelowMinAlt, numOutsideBounds);

  RASM::point3d *nearPoints = (RASM::point3d *)malloc(numNear*sizeof(RASM::point3d));
  RASM::point3d *farPoints  = (RASM::point3d *)malloc(numFar*sizeof(RASM::point3d));
  assert(NULL != nearPoints &&
	 NULL != farPoints);

  // printf("size of arrays is %u, %u, and %u\n",
  // 	 numNear*sizeof(RASM::point3d),
  // 	 numFar*sizeof(RASM::point3d),
  // 	 numPoints()*sizeof(RASM::point3d));

  if(keepAllNear){
    nearN=numNear;
    memcpy(nearPoints, nearCopy, numNear*sizeof(RASM::point3d));
  }else{
    dNear.processPoints(nearCopy, numNear, nearPoints, nearN,
      nearApplyMinPoints /* originally true */,
      nearApplyMeanWithin /* originally false */,
      nearApplyMeanAmongAll /* originally true */,
      nearApplyMeanNeighbors /* originally false */,
      nearOutputFromMean /* originally true */,
      nearMinGridPoints /* not originally included (defaults to 1) */);
    printf("went from %d to %d near points after near decimation\n", 
	   numNear, nearN);
  }

  if(keepAllFar){
    farN=numFar;
    memcpy(farPoints, farCopy, numFar*sizeof(RASM::point3d));
  }else{
    dFar.processPoints(farCopy, numFar, farPoints, farN,
      farApplyMinPoints /* originally false */,
      farApplyMeanWithin /* originally true */,
      farApplyMeanAmongAll /* originally false */,
      farApplyMeanNeighbors /* originally true */,
      farOutputFromMean /* originally true */,
      farMinGridPoints /* not originally included (defaults to 1) */);
    printf("went from %d to %d far points after far decimation\n", 
	     numFar, farN);
  }

  assert(nearN + farN <= numPoints());
  assert(nearN <= numNear);
  assert(farN <= numFar);

  /* move all the points */
  resizeVertices(0);
  //sizeVertices=0;
  for(unsigned int i=0;i<nearN;i++)
    {
      addPoint(nearPoints[i]);
      // vertices[sizeVertices++]=nearPoints[i];
    }

  for(unsigned int i=0;i<farN;i++)
    {
      addPoint(farPoints[i]);
      //      vertices[sizeVertices++]=farPoints[i];
    }

  printf("decimated from %d near + %d far to %d near + %d far = %d vertices\n",
	   numNear, numFar, nearN, farN, numPoints());
 
  /* retriangulate the resulting points */
  triangulateMinimal();

  printf("decimated to %d vertices and %d faces\n", numPoints(), numTriangles());

  free(nearPoints);
  free(farPoints);
  free(nearCopy);
  free(farCopy);
  
  printf("done shrinking\n");

}

void TMAP::tmap_shrinkable::iglSlim(unsigned int finalSize) {

  double stime = now();
  printf("Before mesh decimation: Num vertices:%d  Num faces:%d\n",numPoints(),numTriangles());
  assert(vertices);
  Eigen::MatrixXd V(numPoints(),3);
  Eigen::MatrixXi F(numTriangles(),3);
  for(unsigned int i=0;i<numPoints();i++) {
    V(i,0) = RASM_TO_METERS(vertices[i].X());
    V(i,1) = RASM_TO_METERS(vertices[i].Y());
    V(i,2) = RASM_TO_METERS(vertices[i].Z());
  }
  for(unsigned int i=0;i<numTriangles();i++) {
    F(i,0) = faces[i].points[0];
    F(i,1) = faces[i].points[1];
    F(i,2) = faces[i].points[2];
  }

  Eigen::MatrixXd out_V;
  Eigen::MatrixXi out_F;

  Eigen::VectorXi a;
  Eigen::VectorXi b;

  //RASM::point3d *vertices;
  //RASM::triangle *faces;
  //V - vertices of input mesh
  //F - face indices of input mesh
  //out_V - vertices of output mesh
  //out_F - face indices of output mesh
  //
  igl::decimate(V,F,finalSize,out_V,out_F,a,b);

  /* copy the vertices that remain */
  assert(numPoints()>=out_V.rows());
  resizeVertices(0);
  //  sizeVertices=0;
  for(uint i=0;i<out_V.rows(); i++)
    {
      RASM::point3d newpt(METERS_TO_RASM(out_V(i,0)),
			  METERS_TO_RASM(out_V(i,1)),
			  METERS_TO_RASM(out_V(i,2)));
      addPoint(newpt);
    }

  /* copy the faces that remain */
  resizeTriangles(0);
  //  sizeFaces=0;
  for(uint i=0; i<out_F.rows(); i++) {

	  unsigned int faces[3] = {out_F(i,0), out_F(i,1), out_F(i,2)};
	  int neighbors[3] = {-1, -1, -1};
	  
	  addTriangle(faces, neighbors);
	  
	  //	  sizeFaces++;
  }
  double etime = now();
  printf("After mesh decimation: Num vertices:%d  Num faces:%d\n",numPoints(),numTriangles());
  printf("Mesh decimation took %2.4fms\n",(etime-stime)*1e3);
}

// void TMAP::tmap_shrinkable::slim(unsigned int finalSize){
//   assert(vertices);
//   MxStdModel m(numPoints(), numTriangles());
//   for(unsigned int i=0;i<numPoints();i++)
//     m.add_vertex(RASM_TO_METERS(vertices[i].X()), 
// 		 RASM_TO_METERS(vertices[i].Y()),
// 		 RASM_TO_METERS(vertices[i].Z()));
//   for(unsigned int i=0;i<numTriangles();i++)
//     m.add_face(faces[i].points[0], faces[i].points[1], faces[i].points[2]);
 
//   /* slim init */
//   MxQSlim *slim = new MxEdgeQSlim(m);
//   slim->placement_policy = MX_PLACE_OPTIMAL;
//   slim->boundary_weight = 1000.0;
//   slim->weighting_policy = MX_WEIGHT_AREA;
//   slim->compactness_ratio = 0.0;
//   slim->meshing_penalty = 1.0;
//   slim->will_join_only = false;
//   slim->initialize();

//   /* do the reduction */
//   slim->decimate(finalSize);

//   for(uint i=0; i<m.vert_count(); i++)
//     if( m.vertex_is_valid(i) && m.neighbors(i).length() == 0 )
//       m.vertex_mark_invalid(i);
//   m.compact_vertices();

//   /* copy the vertices that remain */
//   assert(numPoints()>=m.vert_count());
//   resizeVertices(0);
//   //  sizeVertices=0;
//   for(uint i=0;i<m.vert_count(); i++)
//     {
//       RASM::point3d newpt(METERS_TO_RASM(m.vertex(i)[0]),
// 			  METERS_TO_RASM(m.vertex(i)[1]),
// 			  METERS_TO_RASM(m.vertex(i)[2]));
//       addPoint(newpt);
//       // for(int j=0;j<3;j++)
//       // 	{
//       // 	  vertices[sizeVertices].coord3d[j] = METERS_TO_RASM(m.vertex(i)[j]);
//       // 	}
//       // sizeVertices++;
//     }

//   /* copy the faces that remain */
//   resizeTriangles(0);
//   //  sizeFaces=0;
//   for(uint i=0; i<m.face_count(); i++)
//     {
//       if(m.face_is_valid(i))
// 	{
// 	  for(int j=0;j<3;j++)
// 	    {
// 	      assert(m.face(i)[j] >= 0 && m.face(i)[j] < m.vert_count());
// 	    }
	  
// 	  unsigned int faces[3] = {m.face(i)[0], m.face(i)[1], m.face(i)[2]};
// 	  int neighbors[3] = {-1, -1, -1};
// 	  //	  for(int j=0;j<3;j++)
// 	  //	    {
// 	  //	      faces[sizeFaces].points[j] = m.face(i)[j];
// 	  //	      faces[sizeFaces].neighbors[j] = -1;
// 	  //	    }
	  
// 	  addTriangle(faces, neighbors);
	  
// 	  //	  sizeFaces++;
// 	}
//     }

//   /* cleanup */
//   delete slim;
// }


/* shrinks the model and re-triangulates */
void TMAP::tmap_shrinkable::shrink(bool decimate, unsigned int finalSize,
				   bool retriangulate,
				   const char *debugFile){

  if(decimate){
    switch(decimateMode){
    case DECIMATE_UNINITIALIZED:
      fprintf(stderr, "Error, did not set decimation mode\n");
      abort();
    case DECIMATE_CUSTOM:
      customDecimate(customDecimation_nearCutoff,
		     customDecimation_nearWidth,
		     customDecimation_nearDepth,
		     customDecimation_nearRes,
		     customDecimation_farCutoff,
		     customDecimation_farWidth,
		     customDecimation_farDepth,
		     customDecimation_farRes,
		     customDecimation_maxAlt,
		     customDecimation_minAlt,
		     customDecimation_keepAllNear,
		     customDecimation_keepAllFar,
         customDecimation_nearApplyMinPoints,
         customDecimation_farApplyMinPoints,
         customDecimation_nearApplyMeanWithin,
         customDecimation_farApplyMeanWithin,
         customDecimation_nearApplyMeanAmongAll,
         customDecimation_farApplyMeanAmongAll,
         customDecimation_nearApplyMeanNeighbors,
         customDecimation_farApplyMeanNeighbors,
         customDecimation_nearOutputFromMean,
         customDecimation_farOutputFromMean,
         customDecimation_nearMinGridPoints,
         customDecimation_farMinGridPoints);
      break;
    case DECIMATE_TRIDAR:
      tridarDecimate();
      break;
    case DECIMATE_COVARIANCE:
      {
	// TODO: rework custom variable code so these aren't hard-coded
	int num_kept = 0;
	int num_removed = 0;
	for(unsigned int i=0; i < numPoints(); i++)
	  {
	    const float min_dist = 2.0;
	    const float max_dist = 20.0;
	    float d = sqrt(RASM_TO_METERS(vertices[i].X()) * RASM_TO_METERS(vertices[i].X()) +
			   RASM_TO_METERS(vertices[i].Y()) * RASM_TO_METERS(vertices[i].Y()));
	    if((d < min_dist) || (d > max_dist))
	      {
		num_removed++;
		removePoint(i);
	      }
	    else
	      {
		num_kept++;
	      }
	  }
	printf("kept %d, removed %d\n", num_kept, num_removed);
	covarianceDecimate(10, 60);
	break;
      }
    case DECIMATE_ZOE_NAV:
      zoeNavDecimate();
      break;
    default:
      fprintf(stderr, "Error, unknown decimation mode %d\n", decimateMode);
      abort();
    }
  }

  printf("shrinkableTMAP::shrink() done with decimation (Mode: %d), numPoints() %d, numTriangles() %d, finalSize: %d\n", decimateMode, numPoints(), numTriangles(), finalSize);

  if(debugFile){
    writeToFile(debugFile);
    printf("shrinkableTMAP::shrink() Saved %s\n", debugFile);
  }

  //HACK this is a call to qslim and it crashes the program
  //check why this is so and fix it
  //the responsibility of this function is to reduce the triangles.
  //if this step is ignored, the mapper still seems to work, but
  //it might be hogging too much memory
  //if((numPoints()>=3) && (numTriangles()>finalSize))
  //  slim(finalSize);
  if(numPoints()>=3 && (numTriangles()>finalSize)) {
    iglSlim(finalSize);
  }
  printf("shrinkableTMAP::shrink() finished with slim\n");

  // If our decimation mode was anything other than DECIMATE_REPULSION, add the points in after the fact
  if (decimateMode != DECIMATE_REPULSION) {
	for (unsigned int i = 0; i < interestPoints.size(); i++) {
        addPoint(interestPoints[i]);
    }
  }

  if(retriangulate){
    if(numPoints()>=3)
      triangulateMinimal();
  }

  printf("shrinkableTMAP::shrink() finished with retriangulation\n");

}

void TMAP::tmap_shrinkable::setDecimationMode(DECIMATE_MODE mode){
  decimateMode = mode;

  switch(decimateMode){
  case DECIMATE_UNINITIALIZED:
  case DECIMATE_TRIDAR:
  case DECIMATE_COVARIANCE:
    break;
  case DECIMATE_ZOE_NAV:
    break;
  case DECIMATE_CUSTOM:
    fprintf(stderr, "Error, use setCustomDecimation()\n");
    abort();
  default:
    fprintf(stderr, "Error, set invalid decimation mode %d\n", mode);
    abort();
  }
}

void TMAP::tmap_shrinkable::setCustomDecimation(double nearCutoff, double nearWidth, 
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

  customDecimation_nearCutoff=nearCutoff;
  customDecimation_nearWidth=nearWidth;
  customDecimation_nearDepth=nearDepth;
  customDecimation_nearRes=nearRes;
  customDecimation_farCutoff=farCutoff;
  customDecimation_farWidth=farWidth;
  customDecimation_farDepth=farDepth;
  customDecimation_farRes=farRes;
  customDecimation_maxAlt=maxAlt;
  customDecimation_minAlt=minAlt;
  customDecimation_keepAllNear=keepAllNear;
  customDecimation_keepAllFar=keepAllFar;
  
  customDecimation_nearApplyMinPoints=nearApplyMinPoints;
  customDecimation_nearApplyMeanWithin=nearApplyMeanWithin;
  customDecimation_nearApplyMeanAmongAll=nearApplyMeanAmongAll;
  customDecimation_nearApplyMeanNeighbors=nearApplyMeanNeighbors;
  customDecimation_nearOutputFromMean=nearOutputFromMean;
  customDecimation_nearMinGridPoints=nearMinGridPoints;

  customDecimation_farApplyMinPoints=farApplyMinPoints;
  customDecimation_farApplyMeanWithin=farApplyMeanWithin;
  customDecimation_farApplyMeanAmongAll=farApplyMeanAmongAll;
  customDecimation_farApplyMeanNeighbors=farApplyMeanNeighbors;
  customDecimation_farOutputFromMean=farOutputFromMean;
  customDecimation_farMinGridPoints=farMinGridPoints;
  
  decimateMode = DECIMATE_CUSTOM;
}
