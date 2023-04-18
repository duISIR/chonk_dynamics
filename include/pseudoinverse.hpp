#ifndef __se3_pseudoinverse_hpp__
#define __se3_pseudoinverse_hpp__

#include <Eigen/Dense>

namespace se3
{
  template <class MatT>
  inline Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
  pinv(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-9}) // choose appropriately
  {
      typedef typename MatT::Scalar Scalar;
      auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
      const auto &singularValues = svd.singularValues();
      Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
      singularValuesInv.setZero();
      for (unsigned int i = 0; i < singularValues.size(); ++i) {
          if (singularValues(i) > tolerance)
          {
              singularValuesInv(i, i) = Scalar{1} / singularValues(i);
          }
          else
          {
              singularValuesInv(i, i) = Scalar{0};
          }
      }
      return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
  }


} // namespace se3

#endif // ifndef __se3_pseudoinverse_hpp__
