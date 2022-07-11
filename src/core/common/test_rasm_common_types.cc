#include <stdio.h>
#include <assert.h>
#include <float.h>
#include "rasm_common_types.h"

#ifdef USE_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#endif // USE_PCL

int main(int argc, char* argv[])
{
  RASM::point2d p1(1.2, 3.4);
  p1.print(stdout); printf("\n");

  RASM::point2d p2 = p1 * 3;
  p2.print(stdout); printf("\n");

  assert(p2.X() == 3*p1.X());
  assert(p2.Y() == 3*p1.Y());

  RASM::point3d p3(METERS_TO_RASM(1), 0, 0);
  RASM::point3d translation(0, 0, 0);

  RASM::point3d p4 = p3;
  p4.transform(translation, 
	       0,   /* roll */
	       0,   /* pitch */
 	       M_PI /* yaw */);
  printf("%s:%d %f %f %f\n", __FILE__, __LINE__, RASM_TO_METERS(p4.X()), RASM_TO_METERS(p4.Y()), RASM_TO_METERS(p4.Z()));
  assert(p3.X() == -p4.X());

  translation.set(METERS_TO_RASM(1), 0, 0);
  p4 = p3;
  printf("%s:%d %f %f %f\n", __FILE__, __LINE__, RASM_TO_METERS(p4.X()), RASM_TO_METERS(p4.Y()), RASM_TO_METERS(p4.Z()));
  p4.transform(translation, 
	       0,   /* roll */
	       0,   /* pitch */
	       0);
  printf("%s:%d %f %f %f\n", __FILE__, __LINE__, RASM_TO_METERS(p4.X()), RASM_TO_METERS(p4.Y()), RASM_TO_METERS(p4.Z()));
  assert(fabs((METERS_TO_RASM(1) + p3.X()) - p4.X()) < FLT_EPSILON);

  translation.set(METERS_TO_RASM(1), 0, 0);
  p4 = p3;
  printf("%s:%d %f %f %f\n", __FILE__, __LINE__, RASM_TO_METERS(p4.X()), RASM_TO_METERS(p4.Y()), RASM_TO_METERS(p4.Z()));
  p4.transform(translation, 
	       0,   /* roll */
	       0,   /* pitch */
	       M_PI /* yaw */);
  printf("%s:%d %f %f %f\n", __FILE__, __LINE__, RASM_TO_METERS(p4.X()), RASM_TO_METERS(p4.Y()), RASM_TO_METERS(p4.Z()));
  assert(p3.X() == -p4.X() + translation.X());

#ifdef USE_PCL
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(2);
  cloud.width = 2;
  cloud.height = 1;
  cloud.points[0] = p3;
  cloud.points[1] = p4;
  pcl::io::savePCDFile("test_rasm_common_types.pcd", cloud);

  pcl::PointCloud<pcl::PointXYZ> cloud2;
  pcl::io::loadPCDFile("test_rasm_common_types.pcd", cloud2);
  RASM::point3d p5 = cloud2.points[0];  
  RASM::point3d p6 = cloud2.points[1];  

  assert(p5 == p3);
  assert(p6 == p4);

  system("rm test_rasm_common_types.pcd");
#endif // USE_PCL

  printf("tests successful\n");

  return 0;
}
