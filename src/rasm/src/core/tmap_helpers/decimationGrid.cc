#include <stdlib.h>
#include <assert.h>
#include <string.h> /* for memcpy */
#include <decimationGrid.h>
#include <rasm_common_types.h>

TMAP::decimationGrid::decimationGrid(const RASM::bounds2d &bounds, RASM_UNITS spacing)
  :b(bounds), s(spacing){
  assert(spacing>0);
  RASM::point2d d = b.maxPoint - b.minPoint;
  sizeX = (int)ceil(RASM_TO_METERS(d.X())/RASM_TO_METERS(spacing));
  sizeY = (int)ceil(RASM_TO_METERS(d.Y())/RASM_TO_METERS(spacing));
  
  means = (RASM::point3d *)malloc(sizeX*sizeY*sizeof(RASM::point3d));
  stds = (float *)malloc(sizeX*sizeY*sizeof(float));
  maxes = (float *)malloc(sizeX*sizeY*sizeof(float));
  counts = (unsigned int *)malloc(sizeX*sizeY*sizeof(unsigned int));
  assert(means && counts && stds && maxes);

  /*
  printf("Created grid %dx%d\n", sizeX, sizeY);
  printf("min: ");bounds.minPoint.print();printf("\n");
  printf("max: ");bounds.maxPoint.print();printf("\n");
  printf("Spacing: %0.3f\n", RASM_TO_METERS(spacing));
  */
}

TMAP::decimationGrid::~decimationGrid(){
  if(means){free(means);means=NULL;}
  if(stds){free(stds);stds=NULL;}
  if(maxes){free(maxes);maxes=NULL;}
  if(counts){free(counts);counts=NULL;}
}

unsigned int TMAP::decimationGrid::pointToIndexContains(const RASM::point3d &p) const{
  RASM::point2d d = p - b.minPoint;
  unsigned int dX = (unsigned int)(d.X()/s);
  unsigned int dY = (unsigned int)(d.Y()/s);
  return dX + dY*sizeX;
}

int TMAP::decimationGrid::pointToIndex(const RASM::point3d &p) const{
  if(!b.contains(p))return -1;
  RASM::point2d d = p - b.minPoint;
  int dX = (int)(d.X()/s);
  int dY = (int)(d.Y()/s);
  return dX + dY*sizeX;
}

void TMAP::decimationGrid::neighbors(int index, int N[8]) const{
  int dX = index%sizeX;
  int dY = index/sizeX;

  /* fill in neighbors */
  const int dxs[8]={-1,  0,  1, -1,   1, -1,  0,  1};
  const int dys[8]={-1, -1, -1,  0,   0,  1,  1,  1};
  for(int i=0;i<8;i++)
    N[i] = (dX + dxs[i]) + (dY + dys[i])*sizeX;

  /* clear out invalid neighbors */
  if(0==dX)N[0] = N[3] = N[5] = -1;
  else if((int)sizeX-1==dX)N[2] = N[4] = N[7] = -1;
  if(0==dY)N[0] = N[1] = N[2] = -1;
  else if((int)sizeY-1==dY)N[5] = N[6] = N[7] = -1;
}


void TMAP::decimationGrid::fillInMeansContains(const RASM::point3d *inputPoints,
					       unsigned int inputN){
  for(unsigned int i=0;i<maxPoints();i++){
    counts[i] = 0;
    means[i] = RASM::point3d(0,0,0);
  }

  /* get the sum of all the points in each grid cell */
  for(unsigned int i=0;i<inputN;i++){
    unsigned int ind = pointToIndexContains(inputPoints[i]);
    means[ind] += inputPoints[i];
    counts[ind]++;
  }

  /* divide by the number in each cell to get the mean 
   * (no need for cells with 0 or 1 point
   */
  for(unsigned int i=0;i<maxPoints();i++){
    if(counts[i] <= 1)continue;
    for(int j=0;j<3;j++)
      means[i].coord3d[j]/=(RASM_UNITS)(counts[i]);
  }
}

void TMAP::decimationGrid::fillInMeans(const RASM::point3d *inputPoints,
				       unsigned int inputN){
  for(unsigned int i=0;i<maxPoints();i++){
    counts[i] = 0;
    means[i] = RASM::point3d(0,0,0);
  }

  /* get the sum of all the points in each grid cell */
  for(unsigned int i=0;i<inputN;i++){
    int ind = pointToIndex(inputPoints[i]);
    if(-1 == ind)continue;/* skip points outside the grid */
    means[ind] += inputPoints[i];
    counts[ind]++;
  }

  /* divide by the number in each cell to get the mean 
   * (no need for cells with 0 or 1 point
   */
  for(unsigned int i=0;i<maxPoints();i++){
    if(counts[i] <= 1)continue;
    for(int j=0;j<3;j++)
      means[i].coord3d[j]/=(RASM_UNITS)(counts[i]);
  }
}

void TMAP::decimationGrid::fillInStdsContains(const RASM::point3d *inputPoints,
					      unsigned int inputN){
  for(unsigned int i=0;i<maxPoints();i++)
    stds[i] = 0.0;

  for(unsigned int i=0;i<inputN;i++){
    unsigned int ind = pointToIndexContains(inputPoints[i]);
    if(counts[ind] <= 1)continue;

    float dZ = RASM_TO_METERS(inputPoints[i].Z() - means[ind].Z());
    stds[ind] += dZ*dZ;
  }

  for(unsigned int i=0;i<maxPoints();i++){
    if(counts[i] <= 1)continue;/* skip lonely points */
    stds[i]/=(float)(counts[i]);
  }
}

void TMAP::decimationGrid::fillInStds(const RASM::point3d *inputPoints,
				      unsigned int inputN){
  for(unsigned int i=0;i<maxPoints();i++)
    stds[i] = 0.0;

  for(unsigned int i=0;i<inputN;i++){
    int ind = pointToIndex(inputPoints[i]);
    if(-1 == ind)continue;
    if(counts[ind] <= 1)continue;

    float dZ = RASM_TO_METERS(inputPoints[i].Z() - means[ind].Z());
    stds[ind] += dZ*dZ;
  }

  for(unsigned int i=0;i<maxPoints();i++){
    if(counts[i] <= 1)continue;/* skip lonely points */
    stds[i]/=(float)(counts[i]);
  }
}

void TMAP::decimationGrid::fillInMaxes(const RASM::point3d *inputPoints,
				       unsigned int inputN){
  for(unsigned int i=0;i<maxPoints();i++)
    maxes[i] = -9999.0;

  for(unsigned int i=0;i<inputN;i++){
    int ind = pointToIndex(inputPoints[i]);
    if(-1 == ind)continue;

    float Z = RASM_TO_METERS(inputPoints[i].Z());
    if(Z>maxes[ind])maxes[ind]=Z;
  }
}

void TMAP::decimationGrid::fillInMaxesContains(const RASM::point3d *inputPoints,
					       unsigned int inputN){
  for(unsigned int i=0;i<maxPoints();i++)
    maxes[i] = -9999.0;

  for(unsigned int i=0;i<inputN;i++){
    unsigned int ind = pointToIndexContains(inputPoints[i]);
    float Z = RASM_TO_METERS(inputPoints[i].Z());
    if(Z>maxes[ind])maxes[ind]=Z;
  }
}

void TMAP::decimationGrid::processPoints(RASM::point3d *inputPoints,
					 unsigned int inputN,
					 RASM::point3d *outputPoints, 
					 unsigned int &outputN,
					 bool applyMinPointsInCell,
					 bool applyMeanWithinCell,
					 bool applyMeanAmongCellMeans,
					 bool applyMeanAmongCellNeighbors,
					 bool takeOutputFromMeans,
					 unsigned int minGridPoints){

  /* minimum number of points in each grid cell */
  //  const unsigned int minGridPoints = 3;

  /* when filling out the output points, 
   * should cells that have a large deviation be checked?
   */
  bool checkStdOnOutput=true;
  if(!takeOutputFromMeans)checkStdOnOutput=false;

  if(1){
    for(unsigned int i=0;i<inputN;i++){
      if(!b.contains(inputPoints[i]))
	{
	  printf("[%s:%d] ERROR bounds (%f, %f --> %f, %f) do not contain input point %d (%f %f %f)\n",
		 __FILE__, __LINE__,
		 RASM_TO_METERS(b.minPoint.X()),
		 RASM_TO_METERS(b.minPoint.Y()),
		 RASM_TO_METERS(b.maxPoint.X()),
		 RASM_TO_METERS(b.maxPoint.Y()),
		 i, 
		 RASM_TO_METERS(inputPoints[i].X()),
		 RASM_TO_METERS(inputPoints[i].Y()),
		 RASM_TO_METERS(inputPoints[i].Z()));
	  fflush(stdout);
	  assert(0);
	}
    }
  }
  
  fillInMeansContains(inputPoints, inputN);
  
#define REMOVE_POINT(ind)                     \
do{                                           \
  --pointsToRemove;                           \
  inputPoints[ind--] = inputPoints[--inputN]; \
}while(0)

#define REMOVE_POINT_CHECK(pind, cind)         \
do{                                            \
  if(0 == counts[cind])                        \
    --pointsToRemove;                          \
  inputPoints[pind--] = inputPoints[--inputN]; \
}while(0)

#define REMOVE_POINTS()                                      \
do{                                                          \
  if(pointsToRemove>0){                                      \
    for(unsigned int i=0;i<inputN;i++){                      \
      if(0 == counts[pointToIndexContains(inputPoints[i])]){ \
	REMOVE_POINT(i);                                     \
	if(0 == pointsToRemove)                              \
	  break;                                             \
      }                                                      \
    }                                                        \
    assert(0 == pointsToRemove);                             \
  }                                                          \
}while(0)

#define MARK_CELL_FOR_REMOVAL(ind) \
do{                                 \
  pointsToRemove += counts[ind];    \
  counts[ind] = 0;                  \
}while(0)

  unsigned int pointsToRemove=0;
  if(applyMinPointsInCell){ /* apply min points in grid filter */

    for(unsigned int i=0;i<maxPoints();i++){
      if(counts[i] < minGridPoints){
	MARK_CELL_FOR_REMOVAL(i);
      }
    }

  } /* end of min points in grid filter */

  if(checkStdOnOutput || applyMeanWithinCell){ /* fill in the per-cell std if needed */
    fillInStdsContains(inputPoints, inputN);
    fillInMaxesContains(inputPoints, inputN);
  }

  if(applyMeanWithinCell){ /* apply mean altitude filter within each cell */

    /* redo the means, this time with std filtering */
    for(unsigned int i=0;i<inputN;i++){

      int ind = pointToIndex(inputPoints[i]);
      /* cells with 0,1 or 2 points can't be filtered with this method */
      if(counts[ind] <= 2)continue;

      /* see if this is a valid point */
      float dZ = RASM_TO_METERS(inputPoints[i].Z() - means[ind].Z());

      /* see if this point is over 5 deviations plus 10cm from the cell mean */
      if(dZ > (0.1 + 5.0*stds[ind])){
	/* not a valid point, remove it from the means calculation */
	//means[ind] = (means[ind]*counts[ind] - inputPoints[i])/(counts[ind]-1);
	for(int j=0;j<3;j++)
	  means[ind].coord3d[j] = (means[ind].coord3d[j]*(RASM_UNITS)counts[ind] - 
				   inputPoints[i].coord3d[j])/(RASM_UNITS)(counts[ind]-1);
	counts[ind]--;
	REMOVE_POINT_CHECK(i, ind);
      }
    }
    
  }/* end of mean altitude filter within each cell */


  float Zmean=0.0, Zstd=-1.0;
  if(applyMeanAmongCellMeans || applyMeanAmongCellNeighbors){
    /* get Zmean and Zstd, the mean and std of the cell altitudes if needed */
    float Zsum = 0.0;
    int Nvalid = 0;
    for(unsigned int i=0;i<maxPoints();i++){
      if(0 == counts[i])continue;
      Nvalid++;
      Zsum += RASM_TO_METERS(means[i].Z());
    }

    if(Nvalid>0){ /* there better be atleast one valid cell... */
      float ZstdsumSq = 0.0;
      Zmean = Zsum/(float)Nvalid;

      /* get the std */
      for(unsigned int i=0;i<maxPoints();i++){
	if(0 == counts[i])continue;
	float dZ = RASM_TO_METERS(means[i].Z()) - Zmean;
	ZstdsumSq += dZ*dZ;
      }
      Zstd = sqrt(ZstdsumSq/(float)Nvalid);
    }
  }/* end of Zmean and Zstd calculation */

  if(applyMeanAmongCellMeans && Zstd>0.0){ /* apply mean altitude filter over all cell means */
    //printf("applyMeanAmongCellMeans: Zmean: %0.3fm Zstd: %0.3fm\n", Zmean, Zstd);

    /* see if any cell is over 5 deviations plus 10cm from the mean altitude */
    for(unsigned int i=0;i<maxPoints();i++){
      if(0 == counts[i] || counts[i]>30)continue;
      float dZ = RASM_TO_METERS(means[i].Z()) - Zmean;
      if(fabs(dZ) > 0.1 + Zstd*5.0){
	/*
	  printf("Cell %d with %d points is at ", i, counts[i]);
	  means[i].print();
	  printf("\n");
	*/
	
	MARK_CELL_FOR_REMOVAL(i);
      }
    }
  }/* end of mean altitude filter over all cell means */

  if(applyMeanAmongCellNeighbors && Zstd>0.0){ /* apply mean altitude filter over all cell neighbors */
    //printf("applyMeanAmongCellNeighbors: Zmean: %0.3fm Zstd: %0.3fm\n", Zmean, Zstd);

    /* see if any cell is over 3 deviations plus 10cm from four neighbors' mean altitude */
    for(unsigned int ind=0;ind<maxPoints();ind++){
      if(0 == counts[ind])continue;
      int N[8];
      neighbors(ind, N);
      int numFailed=0;
      for(int j=0;j<8;j++){
	if(-1 == N[j])continue;/* no neighbor */

	/* get the comparison altitude, either the neighbor or mean */
	const float Zcomp = (0 == counts[ N[j] ])?Zmean:RASM_TO_METERS(means[ N[j] ].Z());

	float dZ = RASM_TO_METERS(means[ind].Z()) - Zcomp;
	if(fabs(dZ) > 0.1 + Zstd*3.0){
	  numFailed++;
	  /*
	  printf("Cell %d is at %0.3f (%0.3f from neighbor %d at %0.3f) %d\n",
		 ind, RASM_TO_METERS(means[ind].Z()), dZ, 
		 N[j], Zcomp,
		 numFailed);
	  */
	  if(numFailed>=4)break;
	}
      }
      if(numFailed>=4){
	MARK_CELL_FOR_REMOVAL(ind);
      }
    }/* done iterating over cells */

  }/* done apply mean over cell neighbors */

  /* remove points from cells that just got removed */
  REMOVE_POINTS();

  /* TODO more filtering */

  if(takeOutputFromMeans){/* take output points from cell means */

    /* copy all the valid cell means to the output */
    outputN=0;
    if(checkStdOnOutput){
      for(unsigned int i=0;i<maxPoints();i++){
	if(0 == counts[i])continue;
#define LARGE_CELL_STD 0.1 /* in meters */
	if(counts[i] > 10 && stds[i] > LARGE_CELL_STD){
	  RASM::point3d p = means[i];
	  RASM::point3d ll(-s/8, -s/8, METERS_TO_RASM(-stds[i]));
	  RASM::point3d lr( s/8, -s/8, METERS_TO_RASM(-stds[i]));
	  RASM::point3d ul(-s/8,  s/8, METERS_TO_RASM(maxes[i])-p.Z());
	  RASM::point3d ur( s/8,  s/8, METERS_TO_RASM(maxes[i])-p.Z());
	  outputPoints[outputN++] = p+ll;
	  outputPoints[outputN++] = p+lr;
	  outputPoints[outputN++] = p+ul;
	  outputPoints[outputN++] = p+ur;
	  
	}else{
	  outputPoints[outputN++] = means[i];
	}
      }
    }else{/* just copy the means */
      for(unsigned int i=0;i<maxPoints();i++){
	if(0 == counts[i])continue;
	outputPoints[outputN++] = means[i];
      }
    }

  }else{/* take output points from input points */

    /* copy all unmasked input points to the output */
    outputN=inputN;
    if(outputPoints != inputPoints)
      memcpy(outputPoints, inputPoints, inputN*sizeof(struct RASM::point3d));

  }/* done filling in output */

}
