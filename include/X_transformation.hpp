#ifndef __se3_X_transformation_hpp__
#define __se3_X_transformation_hpp__

#include "skew.hpp"

using namespace Eigen;

namespace se3
{
  inline Eigen::Matrix<double, 6, 6>
  parent_X_local(const Vector3d & parent_p_local, const Matrix3d & parent_rot_local)
  {
    Eigen::Matrix<double, 6, 6> A;
    A <<  parent_rot_local, se3::skew(parent_p_local) * parent_rot_local,
          Eigen::Matrix3d::Zero(), parent_rot_local;
    return A;
  }

  inline Eigen::Matrix<double, 6, 6>
  parent_X_local_conventional(const Vector3d & parent_p_local, const Matrix3d & parent_rot_local)
  {
    Eigen::Matrix<double, 6, 6> A;
    A <<  parent_rot_local, Eigen::Matrix3d::Zero(),
          se3::skew(parent_p_local) * parent_rot_local, parent_rot_local;
    return A;
  }


} // namespace se3

#endif // ifndef __se3_X_transformation_hpp__
