#ifndef DECIMATION_GRID_H
#define DECIMATION_GRID_H

#include <rasm_common_types.h>

namespace TMAP {

  class decimationGrid{
    /* grid size and spacing */
    RASM::bounds2d b;
    RASM_UNITS s;
    unsigned int sizeX, sizeY;

    /* given a point, return the cell index
     * returns -1 if the point is out of bounds
     * the contains() version assumes the point is in-bounds
     */
    unsigned int pointToIndexContains(const RASM::point3d &p) const;
    int pointToIndex(const RASM::point3d &p) const;
    void neighbors(int index, int N[8]) const;

    /* per-cell statistics */
    RASM::point3d *means;
    float *stds;
    float *maxes;
    unsigned int *counts;

    /* fills in the mean+count or std of points in each cell
     * the contains() versions assume all input points are in bounds
     */
    void fillInMeansContains(const RASM::point3d *inputPoints, unsigned int inputN);
    void fillInMeans(const RASM::point3d *inputPoints, unsigned int inputN);
    void fillInStdsContains(const RASM::point3d *inputPoints, unsigned int inputN);
    void fillInStds(const RASM::point3d *inputPoints, unsigned int inputN);
    void fillInMaxesContains(const RASM::point3d *inputPoints, unsigned int inputN);
    void fillInMaxes(const RASM::point3d *inputPoints, unsigned int inputN);

  private:
    /* make default and copy constructors private to hide them */
    decimationGrid(){}
    decimationGrid(const decimationGrid &copy){}
    
  public:
    decimationGrid(const RASM::bounds2d &bounds, RASM_UNITS spacing);
    ~decimationGrid();

    /* look at the input points and fill in the output points
       the output should be allocated to hold up to maxPoints()
       note that the input and output buffers CAN be the same
       this function assumes that all points fall within the bounds
       
       the default options are sensible for very dense clouds
       for sparser clouds, consider using 0,1,1,1,0
    */
    static const unsigned int DEFAULT_MIN_GRID_POINTS=1;
    void processPoints(RASM::point3d *inputPoints,
		       unsigned int inputN,
		       RASM::point3d *outputPoints, 
		       unsigned int &outputN,
		       bool applyMinPointsInCell=true,
		       bool applyMeanWithinCell=false,
		       bool applyMeanAmongCellMeans=true,
		       bool applyMeanAmongCellNeighbors=false,
		       bool takeOutputFromMeans=true,
		       unsigned int minGridPoints=DEFAULT_MIN_GRID_POINTS);
    unsigned int maxPoints() const{return sizeX*sizeY;}

  };/* end of decimateGrid declaration */

}/* end of namespace TMAP */

#endif /* end of DECIMATION_GRID_H */
