#include <rasm_common_types.h>
#include <matrixClass.h>
#include <../common/rotation_matrix.h>
#include <tmapQuadTree.h>
#include <tmapOctTree.h>
#include <tmapKDTree.h>

#include <stdio.h>
#include <math.h>
#include <vector>

static void testMatrixClass(){
  int nr = 3; int nc = 3;
  matrix M( nr, nc);
  matrix U( nr, nr);
  matrix S( nr, nc);
  matrix V( nc, nc);

  sscanf("3289.8  354.6  89.1", "%lf %lf %lf", &M[0][0], &M[0][1], &M[0][2]);
  sscanf(" 376.4 7441.5 663.5", "%lf %lf %lf", &M[1][0], &M[1][1], &M[1][2]);
  sscanf("  91.8  663.8 101.7", "%lf %lf %lf", &M[2][0], &M[2][1], &M[2][2]);

  M.svd(U, S, V);
  matrix Ut(U);
  Ut.transpose();

  matrix R2 = V*Ut;

  M.print("M");
  S.print("S");
  V.print("V");
  U.print("U");
  R2.print("R2");

  M.svdRotation().print("R2");
}

static void testRotationMatrix(){
  float A3[3][3], B3[3][3], C3[3][3], M3[3][3], M4[4][4];

  double r=1.0*M_PI/180.0, p=5.0*M_PI/180.0, y=25.0*M_PI/180.0;
  rotationMatrix(r, p, y, M3);
  rotationMatrix(r, p, y, M4);

  for(int r=0;r<3;r++){
    for(int c=0;c<3;c++){
      assert(fabs(M3[r][c] - M4[r][c])<1e-6);
    }
  }

  rotationMatrix(M_PI, 0.0, 0.0, B3);
  rotationMatrix(0.0, M_PI, 0.0, C3);
  matrixMult(A3, B3, C3);
}

static RASM::point3d *testPoints=NULL;
static unsigned int numTestPoints=0;

static void initTestPoints(){
  if(numTestPoints>0)return;

  numTestPoints=5;
  testPoints = (RASM::point3d *)malloc(numTestPoints*sizeof(RASM::point3d));
  testPoints[0] = RASM::point3d(METERS_TO_RASM(1.0),
				METERS_TO_RASM(1.0),
				METERS_TO_RASM(0.1));
  testPoints[1] = RASM::point3d(METERS_TO_RASM(2.0),
				METERS_TO_RASM(2.0),
				METERS_TO_RASM(0.1));
  testPoints[2] = RASM::point3d(METERS_TO_RASM(1.0),
				METERS_TO_RASM(2.0),
				METERS_TO_RASM(0.1));
  testPoints[3] = RASM::point3d(METERS_TO_RASM(2.0),
				METERS_TO_RASM(1.0),
				METERS_TO_RASM(0.1));
  testPoints[4] = RASM::point3d(METERS_TO_RASM(1.5),
				METERS_TO_RASM(1.5),
				METERS_TO_RASM(0.1));
}

static void testQuadTree(){
  initTestPoints();

  printf("\n\n\nStarting testQuadTree()\n\n");
  TMAP::quadTree tree;

  for(unsigned int i=0;i<numTestPoints;i++)
    tree.insert(testPoints[i], i);

  tree.print();

  {
    const RASM::point2d target = testPoints[3];

    unsigned int *matchInd=NULL;
    int matchN = tree.lookup(target, &matchInd);
    if(matchN<0){
      printf("Error, no match!?!\n");
      abort();
    }
    printf("%d indices match:", matchN);
    for(int i=0;i<matchN;i++)printf(" %d", matchInd[i]);
    printf("\n");
  }

  {
    const RASM::point2d small(METERS_TO_RASM(0.1), METERS_TO_RASM(0.1));
    const RASM::point2d target = testPoints[3] - small;

    RASM::point2d closest;
    unsigned int *closeInd=NULL;
    double distSq=0.0;
    int closeN = tree.lookup(target, closest, distSq, &closeInd);
    if(closeN<0){
      printf("Error, no close points!?!\n");
      abort();
    }
    printf("%d indices close:", closeN);
    for(int i=0;i<closeN;i++)printf(" %d", closeInd[i]);
    printf("\n");

    printf("Closest: %0.3f, %0.3f (distSq=%0.5f)\n", 
	   RASM_TO_METERS(closest.X()), RASM_TO_METERS(closest.Y()), distSq);
  }
}

static void testOctTree(){
  initTestPoints();

  printf("\n\n\nStarting testOctTree()\n\n");
  TMAP::octTree tree;

  for(unsigned int i=0;i<numTestPoints;i++)
    tree.insert(testPoints[i], i);

  tree.print();

  {
    const RASM::point3d target = testPoints[3];

    unsigned int *matchInd=NULL;
    int matchN = tree.lookup(target, &matchInd);
    if(matchN<0){
      printf("Error, no match!?!\n");
      abort();
    }
    printf("%d indices match:", matchN);
    for(int i=0;i<matchN;i++)printf(" %d", matchInd[i]);
    printf("\n");
  }

  {
    const RASM::point3d small(METERS_TO_RASM(0.1),
			      METERS_TO_RASM(0.1),
			      METERS_TO_RASM(0.1));
    const RASM::point3d target = testPoints[3] - small;

    RASM::point3d closest;
    unsigned int *closeInd=NULL;
    double distSq=0.0;
    int closeN = tree.lookup(target, closest, distSq, &closeInd);
    if(closeN<0){
      printf("Error, no close points!?!\n");
      abort();
    }
    printf("%d indices close:", closeN);
    for(int i=0;i<closeN;i++)printf(" %d", closeInd[i]);
    printf("\n");

    printf("Closest: %0.3f, %0.3f, %0.3f (distSq=%0.5f)\n", 
	   RASM_TO_METERS(closest.X()), 
	   RASM_TO_METERS(closest.Y()),
	   RASM_TO_METERS(closest.Z()),
	   distSq);
  }
}


static void testKDTree(){
  initTestPoints();

  printf("\n\n\nStarting testKDTree()\n\n");
  TMAP::kdTree tree;

  tree.insert(testPoints, numTestPoints);

  tree.print(stdout);

  {
    const RASM::point3d target = testPoints[3];

    int matchN = tree.lookup(target);
    if(matchN<0){
      printf("Error, no match!?!\n");
      abort();
    }
    printf("Match found: %d\n", matchN);
  }

  {
    const RASM::point3d small(METERS_TO_RASM(0.1),
			      METERS_TO_RASM(0.1),
			      METERS_TO_RASM(0.1));
    const RASM::point3d target = testPoints[3] - small;

    double distSq=0.0;
    int closeN = tree.lookup(target, distSq);
    if(closeN<0){
      printf("Error, no close points!?!\n");
      abort();
    }
    printf("Close index %d (distSq=%0.5f)\n", closeN, distSq);
  }
}

int main(int argc, char **argv){

  if(1)testMatrixClass();
  if(1)testRotationMatrix();
  if(1)testQuadTree();
  if(1)testOctTree();
  if(1)testKDTree();

  return 0;
}
