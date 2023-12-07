//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzlarf.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 07-Dec-2023 09:29:35
//

// Include Files
#include "xzlarf.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                ::coder::array<double, 2U> &C
//                int ic0
//                int ldc
//                double work[2]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzlarf(int m, int n, int iv0, double tau, ::coder::array<double, 2U> &C,
            int ic0, int ldc, double work[2])
{
  int i;
  int ia;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    bool exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      int exitg1;
      i = ic0 + lastc * ldc;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }
  if (lastv > 0) {
    double c;
    int b_i;
    int j;
    if (lastc + 1 != 0) {
      if (lastc >= 0) {
        std::memset(&work[0], 0,
                    static_cast<unsigned int>(lastc + 1) * sizeof(double));
      }
      i = 0;
      b_i = ic0 + ldc * lastc;
      for (int iac{ic0}; ldc < 0 ? iac >= b_i : iac <= b_i; iac += ldc) {
        c = 0.0;
        j = (iac + lastv) - 1;
        for (ia = iac; ia <= j; ia++) {
          c += C[ia - 1] * C[((iv0 + ia) - iac) - 1];
        }
        work[i] += c;
        i++;
      }
    }
    if (!(-tau == 0.0)) {
      i = ic0;
      for (j = 0; j <= lastc; j++) {
        c = work[j];
        if (c != 0.0) {
          c *= -tau;
          b_i = lastv + i;
          for (int iac{i}; iac < b_i; iac++) {
            C[iac - 1] = C[iac - 1] + C[((iv0 + iac) - i) - 1] * c;
          }
        }
        i += ldc;
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzlarf.cpp
//
// [EOF]
//
