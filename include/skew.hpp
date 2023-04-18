
#ifndef __se3_skew_hpp__
#define __se3_skew_hpp__

#include <Eigen/Dense>

using namespace Eigen;

namespace se3
{

  template<class Derived>
  inline Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor> skew(const Eigen::MatrixBase<Derived> & vec)
  {
//      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
      return (Eigen::Matrix<typename Derived::Scalar, 3, 3, Eigen::RowMajor>() <<  0,      -vec[2],  vec[1],
                                                                                   vec[2],  0,      -vec[0],
                                                                                  -vec[1],  vec[0],  0).finished();
  }

//  template <typename Derived>
//  inline Eigen::Matrix<typename Derived::Scalar, 6, 6>
//  skew_spatial_motion(const MatrixBase<Derived> & v)
//  {
////    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);
//    Matrix3d temp_m3f = skew(v.head(3));
//    return (Eigen::Matrix<typename Derived::Scalar, 6, 6>() << temp_m3f,        Matrix3d::Zero(),
//                                                               skew(v.tail(3)), temp_m3f).finished();
//  }

//  template <typename Derived>
//  inline Eigen::Matrix<typename Derived::Scalar, 6, 6>
//  skew_spatial_force(const MatrixBase<Derived> & v)
//  {
////    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 6, 1);
//    Matrix3d temp_m3f = skew(v.head(3));
//    return (Eigen::Matrix<typename Derived::Scalar, 6, 6>() << temp_m3f,         skew(v.tail(3)),
//                                                               Matrix3d::Zero(), temp_m3f).finished();
//  }

} // namespace se3

#endif // ifndef __se3_skew_hpp__
