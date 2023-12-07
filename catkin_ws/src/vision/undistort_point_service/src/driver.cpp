//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: driver.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "driver.h"
#include "anonymous_function.h"
#include "anonymous_function1.h"
#include "anonymous_function11.h"
#include "checkStoppingCriteria.h"
#include "computeFiniteDifferences.h"
#include "linearLeastSquares.h"
#include "lsqcurvefit.h"
#include "rt_nonfinite.h"
#include "undistort_point_service_internal_types.h"
#include "undistort_point_service_internal_types1.h"
#include "undistort_point_service_internal_types11.h"
#include "xgemv.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : b_anonymous_function fun
//                double x0[2]
//                ::coder::array<double, 2U> &fCurrent
//                char output_algorithm[19]
//                double lambda_lower[2]
//                double lambda_upper[2]
//                ::coder::array<double, 2U> &jacobian
//                double &exitflag
//                double &output_iterations
//                double &output_funcCount
//                double &output_stepsize
//                double &output_firstorderopt
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
double driver(b_anonymous_function fun, double x0[2],
              ::coder::array<double, 2U> &fCurrent, char output_algorithm[19],
              double lambda_lower[2], double lambda_upper[2],
              ::coder::array<double, 2U> &jacobian, double &exitflag,
              double &output_iterations, double &output_funcCount,
              double &output_stepsize, double &output_firstorderopt)
{
  static const char cv[19]{'l', 'e', 'v', 'e', 'n', 'b', 'e', 'r', 'g', '-',
                           'm', 'a', 'r', 'q', 'u', 'a', 'r', 'd', 't'};
  c_anonymous_function b_this;
  array<double, 2U> JacCeqTrans;
  array<double, 2U> augJacobian;
  array<double, 2U> fNew;
  array<double, 2U> f_temp;
  array<double, 1U> rhs;
  d_struct_T FiniteDifferences;
  d_struct_T r;
  double a__3[2];
  double dx[2];
  double gradf[2];
  double absx;
  double b_gamma;
  double funDiff;
  double resnorm;
  double resnormNew;
  int aIdx;
  int bIdx;
  int funcCount;
  int iter;
  int iy0_tmp;
  int m_temp_tmp;
  int m_tmp;
  bool exitg1;
  bool stepSuccessful;
  for (bIdx = 0; bIdx < 19; bIdx++) {
    output_algorithm[bIdx] = cv[bIdx];
  }
  dx[0] = rtInf;
  dx[1] = rtInf;
  funDiff = rtInf;
  iter = 0;
  lsqcurvefit_anonFcn1(fun.workspace.fun, fun.workspace.ydata, x0, f_temp);
  m_temp_tmp = f_temp.size(0) << 1;
  jacobian.set_size(m_temp_tmp, 2);
  m_tmp = m_temp_tmp - 1;
  fCurrent.set_size(f_temp.size(0), 2);
  fNew.set_size(f_temp.size(0), 2);
  for (int i{0}; i <= m_tmp; i++) {
    fCurrent[i] = f_temp[i];
  }
  augJacobian.set_size(m_temp_tmp + 2, 2);
  rhs.set_size(m_temp_tmp + 2);
  for (int j{0}; j < 2; j++) {
    aIdx = j * m_temp_tmp;
    bIdx = j * (m_temp_tmp + 2);
    for (int i{0}; i <= m_tmp; i++) {
      augJacobian[bIdx + i] = jacobian[aIdx + i];
    }
  }
  resnorm = 0.0;
  if (m_temp_tmp >= 1) {
    for (int i{0}; i < m_temp_tmp; i++) {
      resnorm += fCurrent[i] * fCurrent[i];
    }
  }
  b_this.workspace.fun = fun;
  FiniteDifferences.nonlin = b_this;
  FiniteDifferences.f_1 = 0.0;
  FiniteDifferences.cEq_1.set_size(m_temp_tmp);
  FiniteDifferences.f_2 = 0.0;
  FiniteDifferences.cEq_2.set_size(m_temp_tmp);
  FiniteDifferences.nVar = 2;
  FiniteDifferences.mIneq = 0;
  FiniteDifferences.mEq = m_temp_tmp;
  FiniteDifferences.numEvals = 0;
  FiniteDifferences.SpecifyObjectiveGradient = false;
  FiniteDifferences.SpecifyConstraintGradient = false;
  FiniteDifferences.isEmptyNonlcon = (m_temp_tmp == 0);
  FiniteDifferences.FiniteDifferenceType = 0;
  FiniteDifferences.hasLB[0] = false;
  FiniteDifferences.hasLB[1] = false;
  FiniteDifferences.hasUB[0] = false;
  FiniteDifferences.hasUB[1] = false;
  FiniteDifferences.hasBounds = false;
  JacCeqTrans.set_size(2, m_temp_tmp);
  a__3[0] = x0[0];
  a__3[1] = x0[1];
  r = FiniteDifferences;
  utils::FiniteDifferences::computeFiniteDifferences(r, fCurrent, a__3,
                                                     JacCeqTrans);
  FiniteDifferences = r;
  aIdx = JacCeqTrans.size(1);
  for (bIdx = 0; bIdx < 2; bIdx++) {
    for (int i{0}; i < aIdx; i++) {
      augJacobian[i + augJacobian.size(0) * bIdx] = JacCeqTrans[bIdx + 2 * i];
    }
  }
  funcCount = FiniteDifferences.numEvals + 1;
  b_gamma = 0.01;
  augJacobian[m_temp_tmp] = 0.0;
  augJacobian[m_temp_tmp + 1] = 0.0;
  augJacobian[m_temp_tmp] = 0.1;
  iy0_tmp = (m_temp_tmp + 2) << 1;
  augJacobian[iy0_tmp - 2] = 0.0;
  augJacobian[iy0_tmp - 1] = 0.0;
  augJacobian[(m_temp_tmp + augJacobian.size(0)) + 1] = 0.1;
  for (int j{0}; j < 2; j++) {
    aIdx = j * (m_temp_tmp + 2);
    bIdx = j * m_temp_tmp;
    for (int i{0}; i <= m_tmp; i++) {
      jacobian[bIdx + i] = augJacobian[aIdx + i];
    }
  }
  internal::blas::xgemv(m_temp_tmp, jacobian, m_temp_tmp, fCurrent, gradf);
  resnormNew = 0.0;
  absx = std::abs(gradf[0]);
  if (std::isnan(absx) || (absx > 0.0)) {
    resnormNew = absx;
  }
  absx = std::abs(gradf[1]);
  if (std::isnan(absx) || (absx > resnormNew)) {
    resnormNew = absx;
  }
  absx = std::fmax(resnormNew, 1.0);
  stepSuccessful = true;
  bIdx = checkStoppingCriteria(gradf, absx, FiniteDifferences.numEvals + 1);
  exitg1 = false;
  while ((!exitg1) && (bIdx == -5)) {
    double xp[2];
    bool evalOK;
    bool guard1{false};
    f_temp.set_size(fCurrent.size(0), 2);
    for (bIdx = 0; bIdx < m_temp_tmp; bIdx++) {
      f_temp[bIdx] = -fCurrent[bIdx];
    }
    for (int i{0}; i <= m_tmp; i++) {
      rhs[i] = f_temp[i];
    }
    rhs[m_temp_tmp] = 0.0;
    rhs[m_temp_tmp + 1] = 0.0;
    linearLeastSquares(augJacobian, rhs, dx, m_temp_tmp + 2);
    xp[0] = x0[0] + dx[0];
    xp[1] = x0[1] + dx[1];
    lsqcurvefit_anonFcn1(fun.workspace.fun, fun.workspace.ydata, xp, f_temp);
    for (int i{0}; i <= m_tmp; i++) {
      fNew[i] = f_temp[i];
    }
    resnormNew = 0.0;
    if (m_temp_tmp >= 1) {
      for (int i{0}; i < m_temp_tmp; i++) {
        resnormNew += fNew[i] * fNew[i];
      }
    }
    evalOK = true;
    for (int i{0}; i < m_temp_tmp; i++) {
      if ((!evalOK) || (std::isinf(fNew[i]) || std::isnan(fNew[i]))) {
        evalOK = false;
      }
    }
    funcCount++;
    guard1 = false;
    if ((resnormNew < resnorm) && evalOK) {
      iter++;
      funDiff = std::abs(resnormNew - resnorm) / resnorm;
      resnorm = resnormNew;
      fCurrent.set_size(fNew.size(0), 2);
      for (bIdx = 0; bIdx < m_temp_tmp; bIdx++) {
        fCurrent[bIdx] = fNew[bIdx];
      }
      JacCeqTrans.set_size(2, m_temp_tmp);
      a__3[0] = xp[0];
      a__3[1] = xp[1];
      r = FiniteDifferences;
      evalOK = utils::FiniteDifferences::computeFiniteDifferences(r, fNew, a__3,
                                                                  JacCeqTrans);
      aIdx = r.numEvals;
      funcCount += aIdx;
      aIdx = JacCeqTrans.size(1);
      for (bIdx = 0; bIdx < 2; bIdx++) {
        for (int i{0}; i < aIdx; i++) {
          augJacobian[i + augJacobian.size(0) * bIdx] =
              JacCeqTrans[bIdx + 2 * i];
        }
      }
      for (int j{0}; j < 2; j++) {
        aIdx = j * (m_temp_tmp + 2);
        bIdx = j * m_temp_tmp;
        for (int i{0}; i <= m_tmp; i++) {
          jacobian[bIdx + i] = augJacobian[aIdx + i];
        }
      }
      if (evalOK) {
        x0[0] = xp[0];
        x0[1] = xp[1];
        if (stepSuccessful) {
          b_gamma *= 0.1;
        }
        stepSuccessful = true;
        guard1 = true;
      } else {
        bIdx = 2;
        aIdx = m_temp_tmp << 1;
        for (int i{0}; i < aIdx; i++) {
          jacobian[i] = rtNaN;
        }
        exitg1 = true;
      }
    } else {
      b_gamma *= 10.0;
      stepSuccessful = false;
      for (int j{0}; j < 2; j++) {
        aIdx = j * m_temp_tmp;
        bIdx = j * (m_temp_tmp + 2);
        for (int i{0}; i <= m_tmp; i++) {
          augJacobian[bIdx + i] = jacobian[aIdx + i];
        }
      }
      guard1 = true;
    }
    if (guard1) {
      resnormNew = std::sqrt(b_gamma);
      augJacobian[m_temp_tmp] = 0.0;
      augJacobian[m_temp_tmp + 1] = 0.0;
      augJacobian[m_temp_tmp] = resnormNew;
      augJacobian[iy0_tmp - 2] = 0.0;
      augJacobian[iy0_tmp - 1] = 0.0;
      augJacobian[(m_temp_tmp + augJacobian.size(0)) + 1] = resnormNew;
      internal::blas::xgemv(m_temp_tmp, jacobian, m_temp_tmp, fCurrent, gradf);
      bIdx = b_checkStoppingCriteria(gradf, absx, funDiff, x0, dx, funcCount,
                                     stepSuccessful, iter);
      if (bIdx != -5) {
        exitg1 = true;
      }
    }
  }
  output_firstorderopt = 0.0;
  funDiff = 3.3121686421112381E-170;
  absx = std::abs(gradf[0]);
  if (std::isnan(absx) || (absx > 0.0)) {
    output_firstorderopt = absx;
  }
  resnormNew = std::abs(dx[0]);
  if (resnormNew > 3.3121686421112381E-170) {
    b_gamma = 1.0;
    funDiff = resnormNew;
  } else {
    absx = resnormNew / 3.3121686421112381E-170;
    b_gamma = absx * absx;
  }
  lambda_lower[0] = 0.0;
  lambda_upper[0] = 0.0;
  absx = std::abs(gradf[1]);
  if (std::isnan(absx) || (absx > output_firstorderopt)) {
    output_firstorderopt = absx;
  }
  resnormNew = std::abs(dx[1]);
  if (resnormNew > funDiff) {
    absx = funDiff / resnormNew;
    b_gamma = b_gamma * absx * absx + 1.0;
    funDiff = resnormNew;
  } else {
    absx = resnormNew / funDiff;
    b_gamma += absx * absx;
  }
  lambda_lower[1] = 0.0;
  lambda_upper[1] = 0.0;
  exitflag = bIdx;
  output_iterations = iter;
  output_funcCount = funcCount;
  output_stepsize = funDiff * std::sqrt(b_gamma);
  return resnorm;
}

} // namespace levenbergMarquardt
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for driver.cpp
//
// [EOF]
//
