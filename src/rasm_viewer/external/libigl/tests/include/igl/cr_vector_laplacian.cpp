#include <test_common.h>
#include <igl/cr_vector_laplacian.h>
#include <igl/cr_vector_mass.h>
#include <igl/EPS.h>

#include <vector>


TEST_CASE("cr_vector_laplacian: cube", "[igl]")
{
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  //This is a cube of dimensions 1.0x1.0x1.0
  igl::read_triangle_mesh(test_common::data_path("cube.obj"), V, F);

  Eigen::SparseMatrix<double> Ls, Ms;
  Eigen::MatrixXi E(12,3), oE(12,3);
  //We need to provide these as unput, as orient_halfedges is
  // platform-dependent (because unique_simplices is)
  E.col(0) << 6,9,0,3,8,10,8,14,12,15,17,15;
  E.col(1) << 0,1,7,4,6,9,16,13,2,5,11,14;
  E.col(2) << 1,2,4,13,10,11,7,16,5,3,12,17;
  oE.col(0) << 1,-1,1,1,-1,1,1,1,-1,1,1,-1;
  oE.col(1) << -1,-1,1,-1,-1,1,-1,1,-1,-1,1,-1;
  oE.col(2) << 1,1,1,-1,-1,-1,-1,1,1,-1,1,-1;
  igl::cr_vector_laplacian(V, F, E, oE, Ls);
  igl::cr_vector_mass(V, F, E, oE, Ms);
  Eigen::MatrixXd L(Ls), M(Ms);

  std::vector<double> initvecL = {0x1p+2,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1p+3,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1p+2,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,-0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1p+3,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x1p+3,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1p+2,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x1p+3,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x1p+2,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x1p+3,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1p+2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1p+3,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,-0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1p+2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x1p+2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1p+2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1p+3,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1p+2,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,-0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1p+3,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x1p+3,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1p+2,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x1p+3,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x1p+2,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x1p+3,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1p+2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1p+2,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1p+3,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x1p+2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x1p+2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,-0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1.6a09e667f3be3p+0,0x0p+0,0x0p+0,0x1p+2};
  Eigen::MatrixXd Lexact = Eigen::Map<Eigen::MatrixXd>(&(initvecL[0]), 36, 36);
  std::vector<double> initvecM = {0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x0p+0,0x1.555555555554fp-2};
  Eigen::MatrixXd Mexact = Eigen::Map<Eigen::MatrixXd>(&(initvecM[0]), 36, 36);

  test_common::assert_near(L,Lexact,1e-12);
  test_common::assert_near(M,Mexact,1e-12);
}
