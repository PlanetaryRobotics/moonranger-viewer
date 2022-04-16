#include <stdio.h>
#include <vector>
#include <rasm_common_types.h>
#include <tmap_full.h>
#include <util.h>

#ifdef USE_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#endif // USE_PCL

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


static void testSearchable(){
  initTestPoints();

  printf("\n\n\nStarting testSearchable()\n\n");

  TMAP::tmap t;
  for(unsigned int i=0; i<numTestPoints; i++)
    t.addPoint(testPoints[i]);

  t.buildTreeVertices();

  for(unsigned int i=0; i<numTestPoints; i++)
    printf("Triangle closest to %d is %d\n",
	   i, t.findClosestVertex(testPoints[i]));
}

int main(int argc, char **argv){

  if(1)testSearchable();

  // THIS FAILS!
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  TMAP::tmap  tmap1;
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  TMAP::tmap* tmap2;
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  tmap2 = new TMAP::tmap();
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  tmap2->readFromFile("test.obj");
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  RASM::point3d p2 = tmap2->getPoint(0);
  printf("[%s:%d %f] tmap1 has %u points, tmap2 has %u points (%0.1f %0.1f %0.1f)\n",
	 __FILE__, __LINE__, now(),
	 tmap1.numPoints(),
	 tmap2->numPoints(),
	 RASM_TO_METERS(p2.X()), RASM_TO_METERS(p2.Y()), RASM_TO_METERS(p2.Z()));

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  tmap1 = *tmap2;
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  RASM::point3d p1 = tmap1.getPoint(0);
  p2 = tmap2->getPoint(0);
  printf("[%s:%d %f] tmap1 has %u points (%0.1f %0.1f %0.1f), tmap2 has %u points (%0.1f %0.1f %0.1f)\n",
	 __FILE__, __LINE__, now(),
	 tmap1.numPoints(),
	 RASM_TO_METERS(p1.X()), RASM_TO_METERS(p1.Y()), RASM_TO_METERS(p1.Z()), 
	 tmap2->numPoints(),
	 RASM_TO_METERS(p2.X()), RASM_TO_METERS(p2.Y()), RASM_TO_METERS(p2.Z()));

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  //  delete tmap2;

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  p1 = tmap1.getPoint(0);
  //  p2 = tmap2->getPoint(0);
  printf("[%s:%d %f] tmap1 has %u points (%0.1f %0.1f %0.1f)\n",
	 __FILE__, __LINE__, now(),
	 tmap1.numPoints(),
	 RASM_TO_METERS(p1.X()), RASM_TO_METERS(p1.Y()), RASM_TO_METERS(p1.Z()));
	 //	 tmap2->numPoints(),
	 //	 RASM_TO_METERS(p2.X()), RASM_TO_METERS(p2.Y()), RASM_TO_METERS(p2.Z()));

  // THIS WORKS!
  //TMAP::tmap tmap1;
  //TMAP::tmap tmap2 = tmap1;

#ifdef USE_PCL

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  cloud1->resize(1);
  cloud1->points[0].x = 1.0;
  cloud1->points[0].y = 2.0;
  cloud1->points[0].z = 3.0;

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  fflush(stdout); printf("[%s:%d %f] cloud2 has %lu points\n",
			 __FILE__, __LINE__, now(),
			 cloud2->size()); fflush(stdout);

  TMAP::tmap tmap3 = *cloud1; 

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  cloud2 = tmap3;

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  fflush(stdout); printf("[%s:%d %f] cloud2 has %lu points\n",
			 __FILE__, __LINE__, now(),
			 cloud2->size()); fflush(stdout);

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  fflush(stdout); 

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  cloud1 = tmap1;
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  fflush(stdout); printf("[%s:%d %f] cloud1 has %lu points\n",
			 __FILE__, __LINE__, now(),
			 cloud1->size()); fflush(stdout);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud1);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud1_filtered);

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

  fflush(stdout); printf("[%s:%d %f] cloud1_filtered has %lu points\n",
			 __FILE__, __LINE__, now(),
			 cloud1_filtered->size()); fflush(stdout);

  TMAP::tmap tmap4 = *cloud1_filtered;

  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());
  tmap4.writeToFile("filtered.obj");
  printf("[%s:%d %f] \n", __FILE__, __LINE__, now());

#endif //USE_PCL
  return 0;
}
