//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: linearLeastSquares.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "linearLeastSquares.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : ::coder::array<double, 2U> &lhs
//                ::coder::array<double, 1U> &rhs
//                double dx[2]
//                int m
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace levenbergMarquardt {
void linearLeastSquares(::coder::array<double, 2U> &lhs,
                        ::coder::array<double, 1U> &rhs, double dx[2], int m)
{
  array<double, 1U> tau;
  double jpvt[2];
  double work[2];
  double temp;
  int i;
  int j;
  int ma;
  int minmana;
  int minmn_tmp;
  int nfxd;
  jpvt[0] = 0.0;
  jpvt[1] = 0.0;
  ma = lhs.size(0);
  minmana = lhs.size(0);
  if (minmana > 2) {
    minmana = 2;
  }
  if (m <= 2) {
    minmn_tmp = m;
  } else {
    minmn_tmp = 2;
  }
  tau.set_size(minmana);
  for (i = 0; i < minmana; i++) {
    tau[i] = 0.0;
  }
  if ((lhs.size(0) == 0) || (minmn_tmp < 1)) {
    jpvt[0] = 1.0;
    jpvt[1] = 2.0;
  } else {
    double d;
    int ii;
    int ix;
    int mmi;
    int temp_tmp;
    nfxd = 0;
    for (j = 0; j < 2; j++) {
      if (jpvt[j] != 0.0) {
        nfxd++;
        if (j + 1 != nfxd) {
          ix = j * ma;
          minmana = (nfxd - 1) * ma;
          for (int k{0}; k < m; k++) {
            temp_tmp = ix + k;
            temp = lhs[temp_tmp];
            i = minmana + k;
            lhs[temp_tmp] = lhs[i];
            lhs[i] = temp;
          }
          jpvt[j] = jpvt[nfxd - 1];
          jpvt[nfxd - 1] = static_cast<double>(j) + 1.0;
        } else {
          jpvt[j] = static_cast<double>(j) + 1.0;
        }
      } else {
        jpvt[j] = static_cast<double>(j) + 1.0;
      }
    }
    if (nfxd > minmn_tmp) {
      nfxd = minmn_tmp;
    }
    minmana = lhs.size(0);
    work[0] = 0.0;
    work[1] = 0.0;
    for (int b_i{0}; b_i < nfxd; b_i++) {
      ii = b_i * minmana + b_i;
      mmi = m - b_i;
      if (b_i + 1 < m) {
        temp = lhs[ii];
        d = internal::reflapack::xzlarfg(mmi, temp, lhs, ii + 2);
        tau[b_i] = d;
        lhs[ii] = temp;
      } else {
        d = 0.0;
        tau[b_i] = 0.0;
      }
      if (b_i + 1 < 2) {
        temp = lhs[ii];
        lhs[ii] = 1.0;
        internal::reflapack::xzlarf(mmi, 1 - b_i, ii + 1, d, lhs,
                                    (ii + minmana) + 1, minmana, work);
        lhs[ii] = temp;
      }
    }
    if (nfxd < minmn_tmp) {
      double vn1[2];
      double vn2[2];
      ma = lhs.size(0);
      work[0] = 0.0;
      vn1[0] = 0.0;
      vn2[0] = 0.0;
      work[1] = 0.0;
      vn1[1] = 0.0;
      vn2[1] = 0.0;
      i = nfxd + 1;
      for (j = i; j < 3; j++) {
        d = internal::blas::xnrm2(m - nfxd, lhs, (nfxd + (j - 1) * ma) + 1);
        vn1[j - 1] = d;
        vn2[j - 1] = d;
      }
      for (int b_i{i}; b_i <= minmn_tmp; b_i++) {
        double s;
        int ip1;
        ip1 = b_i + 1;
        j = (b_i - 1) * ma;
        ii = (j + b_i) - 1;
        mmi = m - b_i;
        minmana = 3 - b_i;
        if (3 - b_i < 1) {
          nfxd = -2;
        } else {
          nfxd = -1;
          if (3 - b_i > 1) {
            temp = std::abs(vn1[b_i - 1]);
            for (int k{2}; k <= minmana; k++) {
              s = std::abs(vn1[(b_i + k) - 2]);
              if (s > temp) {
                nfxd = k - 2;
                temp = s;
              }
            }
          }
        }
        minmana = b_i + nfxd;
        if (minmana + 1 != b_i) {
          ix = minmana * ma;
          for (int k{0}; k < m; k++) {
            temp_tmp = ix + k;
            temp = lhs[temp_tmp];
            nfxd = j + k;
            lhs[temp_tmp] = lhs[nfxd];
            lhs[nfxd] = temp;
          }
          temp = jpvt[minmana];
          jpvt[minmana] = jpvt[b_i - 1];
          jpvt[b_i - 1] = temp;
          vn1[minmana] = vn1[b_i - 1];
          vn2[minmana] = vn2[b_i - 1];
        }
        if (b_i < m) {
          temp = lhs[ii];
          d = internal::reflapack::xzlarfg(mmi + 1, temp, lhs, ii + 2);
          tau[b_i - 1] = d;
          lhs[ii] = temp;
        } else {
          d = 0.0;
          tau[b_i - 1] = 0.0;
        }
        if (b_i < 2) {
          temp = lhs[ii];
          lhs[ii] = 1.0;
          internal::reflapack::xzlarf(mmi + 1, 2 - b_i, ii + 1, d, lhs,
                                      (ii + ma) + 1, ma, work);
          lhs[ii] = temp;
        }
        for (j = ip1; j < 3; j++) {
          minmana = b_i + (j - 1) * ma;
          d = vn1[j - 1];
          if (d != 0.0) {
            temp = std::abs(lhs[minmana - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }
            s = d / vn2[j - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < m) {
                d = internal::blas::xnrm2(mmi, lhs, minmana + 1);
                vn1[j - 1] = d;
                vn2[j - 1] = d;
              } else {
                vn1[j - 1] = 0.0;
                vn2[j - 1] = 0.0;
              }
            } else {
              vn1[j - 1] = d * std::sqrt(temp);
            }
          }
        }
      }
    }
  }
  minmana = lhs.size(0);
  nfxd = lhs.size(0);
  if (nfxd > 2) {
    nfxd = 2;
  }
  for (j = 0; j < nfxd; j++) {
    if (tau[j] != 0.0) {
      temp = rhs[j];
      i = j + 2;
      for (int b_i{i}; b_i <= minmana; b_i++) {
        temp += lhs[(b_i + lhs.size(0) * j) - 1] * rhs[b_i - 1];
      }
      temp *= tau[j];
      if (temp != 0.0) {
        rhs[j] = rhs[j] - temp;
        for (int b_i{i}; b_i <= minmana; b_i++) {
          rhs[b_i - 1] = rhs[b_i - 1] - lhs[(b_i + lhs.size(0) * j) - 1] * temp;
        }
      }
    }
  }
  dx[0] = rhs[0];
  dx[1] = rhs[1];
  if (lhs.size(0) != 0) {
    for (j = 1; j >= 0; j--) {
      minmana = j + j * m;
      dx[j] /= lhs[minmana];
      for (int b_i{0}; b_i < j; b_i++) {
        dx[j - 1] -= dx[j] * lhs[minmana - 1];
      }
    }
  }
  work[1] = dx[1];
  dx[static_cast<int>(jpvt[0]) - 1] = dx[0];
  dx[static_cast<int>(jpvt[1]) - 1] = work[1];
}

} // namespace levenbergMarquardt
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for linearLeastSquares.cpp
//
// [EOF]
//
