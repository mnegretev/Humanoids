//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "xgemv.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : int m
//                const ::coder::array<double, 2U> &A
//                int lda
//                const ::coder::array<double, 2U> &x
//                double y[2]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xgemv(int m, const ::coder::array<double, 2U> &A, int lda,
           const ::coder::array<double, 2U> &x, double y[2])
{
  if (m != 0) {
    int i;
    int iy;
    y[0] = 0.0;
    y[1] = 0.0;
    iy = 0;
    i = lda + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      y[iy] += c;
      iy++;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xgemv.cpp
//
// [EOF]
//
