// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "repmat.h"

template <typename DerivedA, typename DerivedB>
IGL_INLINE void igl::repmat(
  const Eigen::MatrixBase<DerivedA> & A,
  const int r,
  const int c,
  Eigen::PlainObjectBase<DerivedB> & B)
{
  assert(r>0);
  assert(c>0);
  // Make room for output
  B.resize(r*A.rows(),c*A.cols());

  // copy tiled blocks
  for(int i = 0;i<r;i++)
  {
    for(int j = 0;j<c;j++)
    {
      B.block(i*A.rows(),j*A.cols(),A.rows(),A.cols()) = A;
    }
  }
}

template <typename T, int majorType>
IGL_INLINE void igl::repmat(
  const Eigen::SparseMatrix<T, majorType> & A,
  const int r,
  const int c,
  Eigen::SparseMatrix<T, majorType> & B)
{
  assert(r>0);
  assert(c>0);
  B.resize(r*A.rows(), c*A.cols());
  std::vector<Eigen::Triplet<T>> b;
  b.reserve(r*c*A.nonZeros());

  for(int i = 0; i < r; i++)
  {
    for(int j = 0; j < c; j++)
    {
      // loop outer level
      for (int k = 0; k < A.outerSize(); ++k)
      {
        // loop inner level
        for (typename Eigen::SparseMatrix<T, majorType>::InnerIterator 
          it(A,k); it; ++it)
        {
          Eigen::Triplet<T> triplet(i * A.rows() + it.row(), j * A.cols() 
            + it.col(), it.value());
          b.push_back(triplet);
        }
      }
    }
  }
    B.setFromTriplets(b.begin(), b.end());
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
// generated by autoexplicit.sh
template void igl::repmat<double, 0>(Eigen::SparseMatrix<double, 0, int> const&, int, int, Eigen::SparseMatrix<double, 0, int>&);
// generated by autoexplicit.sh
template void igl::repmat<Eigen::Matrix<double, -1, 1, 0, -1, 1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> > const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
template void igl::repmat<Eigen::Matrix<double, -1, 1, 0, -1, 1>, Eigen::Matrix<double, -1, 1, 0, -1, 1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> > const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> >&);
template void igl::repmat<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, int, int, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
template void igl::repmat<double, 1>(Eigen::SparseMatrix<double, 1, int> const&, int, int, Eigen::SparseMatrix<double, 1, int>&);
template void igl::repmat<int, 1>(Eigen::SparseMatrix<int, 1, int> const&, int, int, Eigen::SparseMatrix<int, 1, int>&);
template void igl::repmat<float, 1>(Eigen::SparseMatrix<float, 1, int> const&, int, int, Eigen::SparseMatrix<float, 1, int>&);
template void igl::repmat<int, 0>(Eigen::SparseMatrix<int, 0, int> const&, int, int, Eigen::SparseMatrix<int, 0, int>&);
template void igl::repmat<float, 0>(Eigen::SparseMatrix<float, 0, int> const&, int, int, Eigen::SparseMatrix<float, 0, int>&);
#endif