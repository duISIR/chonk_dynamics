
#ifndef __se3_i_localgenerate_hpp__
#define __se3_i_localgenerate_hpp__

#include <Eigen/Dense>
using namespace Eigen;

namespace se3
{
  inline Matrix3d
  i_localgenerate(const double & i_xx, const double & i_xy, const double & i_xz,
                  const double & i_yy, const double & i_yz, const double & i_zz)
  {
    return (Eigen::Matrix3d() <<  i_xx, i_xy, i_xz,
                                  i_xy, i_yy, i_yz,
                                  i_xz, i_yz, i_zz).finished();
  }

} // namespace se3

#endif // ifndef __se3_skew_hpp__
