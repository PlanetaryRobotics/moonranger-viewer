/*
 * Subclass of tmap_shrinkable to reduce a point cloud
 * This can be done by overlaying a decimationGrid and/or
 * using edge contraction.
 *
 * Dominic Jonak
 */
#ifndef TMAP_SHRINKABLE_H
#define TMAP_SHRINKABLE_H

#include <tmap_triangulate.h>
#include <opencv2/opencv.hpp>

namespace TMAP {

  class tmap_shrinkable : public virtual tmap_triangulate{
  private:
    void tridarDecimate();
    void zoeNavDecimate();
    void covarianceDecimate(unsigned int maxGroupSize, unsigned int maxGroupVariance);
    void covarianceSplitPoints(cv::Mat& points, unsigned int maxGroupSize, unsigned int maxGroupVariance);
    void customDecimate(double nearCutoff, double nearWidth, 
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
            		int nearMinGridPoints, int farMinGridPoints);
    //void slim(unsigned int finalSize);
    void iglSlim(unsigned int finalSize);

  public:

    tmap_shrinkable():tmap_triangulate(){
      decimateMode=DECIMATE_UNINITIALIZED;
    }
    tmap_shrinkable(const char *filename):tmap_triangulate(){
      readFromFile(filename);
      decimateMode=DECIMATE_UNINITIALIZED;
    }

    /* shrinks the model to the desired size,
       can optionally decimate the model before hand
       (0 = don't decimate, 1 = decimate)
       can optionally retriangulate the model afterwards
       can optionally save the mesh to debugFile, NULL disables this
       if decimating, be sure to call setDecimationMode() first
       if NOT retriangulating, then you'll need to handle the following:
       triangulation, centers, associations, neighbors
    */
    void shrink(bool decimate, unsigned int finalSize, bool retriangulate,
		const char *debugFile=NULL);

    enum DECIMATE_MODE{
      DECIMATE_UNINITIALIZED=0,
      DECIMATE_CUSTOM,
      DECIMATE_TRIDAR,
      DECIMATE_ZOE_NAV,
      DECIMATE_COVARIANCE,
      DECIMATE_REPULSION
    };
    void setDecimationMode(DECIMATE_MODE mode);
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
			     int nearMinGridPoints, int farMinGridPoints);

  private:
    DECIMATE_MODE decimateMode;

    double customDecimation_nearCutoff, customDecimation_nearWidth, 
      customDecimation_nearDepth, customDecimation_nearRes,
      customDecimation_farCutoff, customDecimation_farWidth, 
      customDecimation_farDepth,  customDecimation_farRes,
      customDecimation_maxAlt, customDecimation_minAlt;

    bool customDecimation_keepAllNear, customDecimation_keepAllFar, 
      customDecimation_nearApplyMinPoints, customDecimation_farApplyMinPoints,
      customDecimation_nearApplyMeanWithin, customDecimation_farApplyMeanWithin,
      customDecimation_nearApplyMeanAmongAll, customDecimation_farApplyMeanAmongAll,
      customDecimation_nearApplyMeanNeighbors, customDecimation_farApplyMeanNeighbors,
      customDecimation_nearOutputFromMean, customDecimation_farOutputFromMean;
    
    int customDecimation_nearMinGridPoints, customDecimation_farMinGridPoints;
  }; /* end of class tmap_shrinkable declaration */

} /* end of namespace TMAP */

#endif
