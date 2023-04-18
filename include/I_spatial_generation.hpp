
#ifndef __se3_I_spatial_generation_hpp__
#define __se3_I_spatial_generation_hpp__

#include <Eigen/Core>
#include <skew.hpp>

using namespace Eigen;

namespace se3
{

  template<typename Derived, typename Derived2>
  inline Eigen::Matrix<typename Derived::Scalar, 6, 6>
  I_spatial_generate(double m, const MatrixBase<Derived>& pm_local,const MatrixBase<Derived2>& I_local)
  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);
    Matrix3d temp_m3f = skew(pm_local);
    return (Eigen::Matrix<typename Derived::Scalar, 6, 6>() << I_local + m * temp_m3f * temp_m3f.transpose(), m * temp_m3f,
                                                               m * temp_m3f.transpose(), m * Matrix3d::Identity(3, 3)).finished();
                                                               
//    return (Eigen::Matrix<typename Derived::Scalar, 6, 6>() << m * Matrix3d::Identity(3, 3), m * temp_m3f.transpose(),
//                                                               m * temp_m3f, I_local + m * temp_m3f * temp_m3f.transpose()).finished();
  }


} // namespace se3

#endif // ifndef __se3_skew_hpp__
