//
// File: InformationFilterUpdate.cu
//
// GPU Coder version                    : 1.5
// CUDA/C/C++ source code generated on  : 07-Sep-2020 09:07:32
//

// Include Files
#include "InformationFilterUpdate.h"
#include "MWCudaDimUtility.hpp"
#include <cmath>

// Function Declarations
static __global__ void InformationFilterUpdate_kernel1(double Rw[4]);
static __global__ void InformationFilterUpdate_kernel2(const double Rw[2],
  double b_Rw[4]);
static __global__ void InformationFilterUpdate_kernel3(const double r, const
  double t, const double Rw[4], double Rw_inv[4]);
static __global__ void InformationFilterUpdate_kernel4(const double r, const
  double t, const double Rw[4], double Rw_inv[4]);
static __global__ void InformationFilterUpdate_kernel5(double Fk_inv[16]);
static __global__ void InformationFilterUpdate_kernel6(const double T, double x
  [16]);
static __global__ void InformationFilterUpdate_kernel7(const signed char iv[4],
  const signed char iv1[4], signed char ipiv[4], double x[16]);
static __global__ void InformationFilterUpdate_kernel8(signed char p[4]);
static __global__ void InformationFilterUpdate_kernel9(const double T, double
  Gk[8]);
static __global__ void ab_InformationFilterUpdate_kern(const double r, const
  double delta, const double L_imuToRear, const double xk_m_out[4], double
  H_linear[52], double hk[13]);
static __global__ void b_InformationFilterUpdate_kerne(const double a[16], const
  double Fk_inv[16], const int i, double x[16]);
static __global__ void bb_InformationFilterUpdate_kern(const double H_linear[52],
  double A[52]);
static __global__ void c_InformationFilterUpdate_kerne(const double Fk_inv[16],
  const double x[16], const int i, double Ih[16]);
static __global__ void cb_InformationFilterUpdate_kern(const double xk_m_out[4],
  const double H_linear[52], const double hk[13], const double y_meas[13],
  double b_y_meas[13]);
static __global__ void d_InformationFilterUpdate_kerne(const double Ih[16],
  const double Gk[8], double b_Gk[8]);
static __global__ void db_InformationFilterUpdate_kern(const double y_meas[13],
  const double C[52], const double ik[4], double op[4]);
static __global__ void e_InformationFilterUpdate_kerne(const double Gk[8], const
  double b_Gk[8], const double Rw_inv[4], double x[4]);
static __global__ void f_InformationFilterUpdate_kerne(const double r, const
  double t, const double x[4], double Rw[4]);
static __global__ void g_InformationFilterUpdate_kerne(const double r, const
  double t, const double x[4], double Rw[4]);
static __global__ void h_InformationFilterUpdate_kerne(const double Gk[8], const
  double Ih[16], double b_Ih[8]);
static __global__ void i_InformationFilterUpdate_kerne(const double Rw[4], const
  double Ih[8], double b_Ih[8]);
static __global__ void j_InformationFilterUpdate_kerne(const double Gk[8], const
  double Ih[8], double calcEq[16]);
static __global__ void k_InformationFilterUpdate_kerne(const double
  initialization_vec[4], double ih[4]);
static __global__ void l_InformationFilterUpdate_kerne(const double ih[4], const
  double a[16], double ik[4]);
static __global__ void m_InformationFilterUpdate_kerne(const double ik[4], const
  double Fk_inv[16], double ih[4]);
static __global__ void n_InformationFilterUpdate_kerne(const double calcEq[16],
  const double ih[4], double ik[4]);
static __global__ void o_InformationFilterUpdate_kerne(double Fk_inv[16]);
static __global__ void p_InformationFilterUpdate_kerne(const double calcEq[16],
  const double Ih[16], double x[16]);
static __global__ void q_InformationFilterUpdate_kerne(signed char ipiv[4]);
static __global__ void r_InformationFilterUpdate_kerne(signed char p[4]);
static __global__ void s_InformationFilterUpdate_kerne(const double ik[4], const
  double Fk_inv[16], double xk_m_out[4]);
static __global__ void t_InformationFilterUpdate_kerne(const double
  B_usedMeas_vec[13], const double Re[13], double Re_inv[13]);
static __global__ void u_InformationFilterUpdate_kerne(double Re_inv[169]);
static __global__ void v_InformationFilterUpdate_kerne(const double Re_inv[13],
  double b_Re_inv[169]);
static __global__ void w_InformationFilterUpdate_kerne(double H_linear[52]);
static __global__ void x_InformationFilterUpdate_kerne(const signed char iv2[4],
  const signed char iv3[4], const signed char iv1[4], double H_linear[52]);
static __global__ void y_InformationFilterUpdate_kerne(const double xk_m_out[4],
  const double H_linear[52], double hk[13]);

// Function Definitions

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Rw[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel1
  (double Rw[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    // ---- Prediction step -------------------------
    Rw[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Rw[2]
//                double b_Rw[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel2(
  const double Rw[2], double b_Rw[4])
{
  unsigned int threadId;
  int j;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  j = static_cast<int>(threadId);
  if (j < 2) {
    b_Rw[j + (j << 1)] = Rw[j];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double r
//                const double t
//                const double Rw[4]
//                double Rw_inv[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel3(
  const double r, const double t, const double Rw[4], double Rw_inv[4])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Rw_inv[0] = Rw[3] / Rw[0] * t;
    Rw_inv[1] = -r * t;
    Rw_inv[2] = -Rw[2] / Rw[0] * t;
    Rw_inv[3] = t;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double r
//                const double t
//                const double Rw[4]
//                double Rw_inv[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel4(
  const double r, const double t, const double Rw[4], double Rw_inv[4])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Rw_inv[0] = Rw[3] / Rw[1] * t;
    Rw_inv[1] = -t;
    Rw_inv[2] = -Rw[2] / Rw[1] * t;
    Rw_inv[3] = r * t;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Fk_inv[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel5
  (double Fk_inv[16])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 16) {
    //  System matrix
    Fk_inv[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double T
//                double x[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel6(
  const double T, double x[16])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    x[0] = 1.0;
    x[4] = T;
    x[8] = 0.0;
    x[12] = 0.0;
    x[2] = 0.0;
    x[6] = 0.0;
    x[10] = 1.0;
    x[14] = T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv[4]
//                const signed char iv1[4]
//                signed char ipiv[4]
//                double x[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel7(
  const signed char iv[4], const signed char iv1[4], signed char ipiv[4], double
  x[16])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    x[(i << 2) + 1] = static_cast<double>(iv1[i]);
    x[(i << 2) + 3] = static_cast<double>(iv[i]);
    ipiv[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel8
  (signed char p[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    p[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double T
//                double Gk[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel9(
  const double T, double Gk[8])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    //  Noise matrix
    Gk[0] = T * T / 2.0;
    Gk[4] = 0.0;
    Gk[1] = T;
    Gk[5] = 0.0;
    Gk[2] = 0.0;
    Gk[6] = T * T / 2.0;
    Gk[3] = 0.0;
    Gk[7] = T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double r
//                const double delta
//                const double L_imuToRear
//                const double xk_m_out[4]
//                double H_linear[52]
//                double hk[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void ab_InformationFilterUpdate_kern(
  const double r, const double delta, const double L_imuToRear, const double
  xk_m_out[4], double H_linear[52], double hk[13])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    hk[0] -= xk_m_out[2] * xk_m_out[2] * L_imuToRear;
    hk[1] += xk_m_out[0] * xk_m_out[2];
    hk[10] = xk_m_out[0] * cos(delta) + r * xk_m_out[2] * sin(delta);
    H_linear[26] = -2.0 * xk_m_out[2] * L_imuToRear;
    H_linear[1] = xk_m_out[2];
    H_linear[27] = xk_m_out[0];
    H_linear[4] = cos(delta);
    H_linear[10] = cos(delta);
    H_linear[36] = r * sin(delta);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a[16]
//                const double Fk_inv[16]
//                const int i
//                double x[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void b_InformationFilterUpdate_kerne(
  const double a[16], const double Fk_inv[16], const int i, double x[16])
{
  unsigned int threadId;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId);
  if (i5 < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += Fk_inv[i4 + (i << 2)] * a[i4 + (i5 << 2)];
    }

    x[i + (i5 << 2)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double H_linear[52]
//                double A[52]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void bb_InformationFilterUpdate_kern(
  const double H_linear[52], double A[52])
{
  unsigned int threadId;
  int i;
  int i4;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i4 = static_cast<int>(threadId % 4U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i4)) / 4U);
  if (i < 13) {
    A[i4 + (i << 2)] = H_linear[i + 13 * i4];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Fk_inv[16]
//                const double x[16]
//                const int i
//                double Ih[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void c_InformationFilterUpdate_kerne(
  const double Fk_inv[16], const double x[16], const int i, double Ih[16])
{
  unsigned int threadId;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId);
  if (i5 < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += x[i + (i4 << 2)] * Fk_inv[i4 + (i5 << 2)];
    }

    Ih[i + (i5 << 2)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double xk_m_out[4]
//                const double H_linear[52]
//                const double hk[13]
//                const double y_meas[13]
//                double b_y_meas[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void cb_InformationFilterUpdate_kern(
  const double xk_m_out[4], const double H_linear[52], const double hk[13],
  const double y_meas[13], double b_y_meas[13])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 13) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += H_linear[i + 13 * i4] * xk_m_out[i4];
    }

    b_y_meas[i] = (y_meas[i] - hk[i]) + d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Ih[16]
//                const double Gk[8]
//                double b_Gk[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void d_InformationFilterUpdate_kerne(
  const double Ih[16], const double Gk[8], double b_Gk[8])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 4U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 4U);
  if (i < 2) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += Gk[i4 + (i << 2)] * Ih[i4 + (i5 << 2)];
    }

    b_Gk[i + (i5 << 1)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y_meas[13]
//                const double C[52]
//                const double ik[4]
//                double op[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void db_InformationFilterUpdate_kern(
  const double y_meas[13], const double C[52], const double ik[4], double op[4])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 13; i4++) {
      d += C[i + (i4 << 2)] * y_meas[i4];
    }

    op[i] = ik[i] + d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[8]
//                const double b_Gk[8]
//                const double Rw_inv[4]
//                double x[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void e_InformationFilterUpdate_kerne(
  const double Gk[8], const double b_Gk[8], const double Rw_inv[4], double x[4])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 2U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 2U);
  if (i < 2) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += b_Gk[i + (i4 << 1)] * Gk[i4 + (i5 << 2)];
    }

    x[i + (i5 << 1)] = d + Rw_inv[i + (i5 << 1)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double r
//                const double t
//                const double x[4]
//                double Rw[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void f_InformationFilterUpdate_kerne(
  const double r, const double t, const double x[4], double Rw[4])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Rw[0] = x[3] / x[0] * t;
    Rw[1] = -r * t;
    Rw[2] = -x[2] / x[0] * t;
    Rw[3] = t;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double r
//                const double t
//                const double x[4]
//                double Rw[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void g_InformationFilterUpdate_kerne(
  const double r, const double t, const double x[4], double Rw[4])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Rw[0] = x[3] / x[1] * t;
    Rw[1] = -t;
    Rw[2] = -x[2] / x[1] * t;
    Rw[3] = r * t;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[8]
//                const double Ih[16]
//                double b_Ih[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void h_InformationFilterUpdate_kerne(
  const double Gk[8], const double Ih[16], double b_Ih[8])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 2U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 2U);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += Ih[i + (i4 << 2)] * Gk[i4 + (i5 << 2)];
    }

    b_Ih[i + (i5 << 2)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Rw[4]
//                const double Ih[8]
//                double b_Ih[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void i_InformationFilterUpdate_kerne(
  const double Rw[4], const double Ih[8], double b_Ih[8])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 2U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 2U);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 2; i4++) {
      d += Ih[i + (i4 << 2)] * Rw[i4 + (i5 << 1)];
    }

    b_Ih[i + (i5 << 2)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[8]
//                const double Ih[8]
//                double calcEq[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void j_InformationFilterUpdate_kerne(
  const double Gk[8], const double Ih[8], double calcEq[16])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 4U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 4U);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 2; i4++) {
      d += Ih[i + (i4 << 2)] * Gk[i5 + (i4 << 2)];
    }

    calcEq[i + (i5 << 2)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double initialization_vec[4]
//                double ih[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void k_InformationFilterUpdate_kerne(
  const double initialization_vec[4], double ih[4])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    ih[0] = initialization_vec[2];
    ih[1] = 0.0;
    ih[2] = 0.0;
    ih[3] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ih[4]
//                const double a[16]
//                double ik[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void l_InformationFilterUpdate_kerne(
  const double ih[4], const double a[16], double ik[4])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += a[i + (i4 << 2)] * ih[i4];
    }

    ik[i] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ik[4]
//                const double Fk_inv[16]
//                double ih[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void m_InformationFilterUpdate_kerne(
  const double ik[4], const double Fk_inv[16], double ih[4])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += Fk_inv[i4 + (i << 2)] * ik[i4];
    }

    ih[i] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double calcEq[16]
//                const double ih[4]
//                double ik[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void n_InformationFilterUpdate_kerne(
  const double calcEq[16], const double ih[4], double ik[4])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += calcEq[i + (i4 << 2)] * ih[i4];
    }

    ik[i] = ih[i] - d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Fk_inv[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void o_InformationFilterUpdate_kerne
  (double Fk_inv[16])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 16) {
    Fk_inv[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double calcEq[16]
//                const double Ih[16]
//                double x[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void p_InformationFilterUpdate_kerne(
  const double calcEq[16], const double Ih[16], double x[16])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 4U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 4U);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += calcEq[i + (i4 << 2)] * Ih[i4 + (i5 << 2)];
    }

    x[i + (i5 << 2)] = Ih[i + (i5 << 2)] - d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void q_InformationFilterUpdate_kerne
  (signed char ipiv[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void r_InformationFilterUpdate_kerne
  (signed char p[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    p[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ik[4]
//                const double Fk_inv[16]
//                double xk_m_out[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void s_InformationFilterUpdate_kerne(
  const double ik[4], const double Fk_inv[16], double xk_m_out[4])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += Fk_inv[i + (i4 << 2)] * ik[i4];
    }

    xk_m_out[i] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double B_usedMeas_vec[13]
//                const double Re[13]
//                double Re_inv[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void t_InformationFilterUpdate_kerne(
  const double B_usedMeas_vec[13], const double Re[13], double Re_inv[13])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 13) {
    Re_inv[i] = 1.0 / Re[i] * B_usedMeas_vec[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Re_inv[169]
// Return Type  : void
//
static __global__ __launch_bounds__(192, 1) void u_InformationFilterUpdate_kerne
  (double Re_inv[169])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 169) {
    Re_inv[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Re_inv[13]
//                double b_Re_inv[169]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void v_InformationFilterUpdate_kerne(
  const double Re_inv[13], double b_Re_inv[169])
{
  unsigned int threadId;
  int j;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  j = static_cast<int>(threadId);
  if (j < 13) {
    b_Re_inv[j + 13 * j] = Re_inv[j];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double H_linear[52]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void w_InformationFilterUpdate_kerne
  (double H_linear[52])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    H_linear[48] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv2[4]
//                const signed char iv3[4]
//                const signed char iv1[4]
//                double H_linear[52]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void x_InformationFilterUpdate_kerne(
  const signed char iv2[4], const signed char iv3[4], const signed char iv1[4],
  double H_linear[52])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    H_linear[13 * i] = static_cast<double>(iv1[i]);
    H_linear[13 * i + 2] = static_cast<double>(iv3[i]);
    H_linear[13 * i + 3] = static_cast<double>(iv2[i]);
    H_linear[13 * i + 4] = 0.0;
    H_linear[13 * i + 5] = 0.0;
    H_linear[13 * i + 10] = 0.0;
    H_linear[13 * i + 11] = static_cast<double>(iv2[i]);
    H_linear[13 * i + 12] = static_cast<double>(iv2[i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double xk_m_out[4]
//                const double H_linear[52]
//                double hk[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void y_InformationFilterUpdate_kerne(
  const double xk_m_out[4], const double H_linear[52], double hk[13])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 13) {
    //  Nonlinear parts
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += H_linear[i + 13 * i4] * xk_m_out[i4];
    }

    hk[i] = d;
  }
}

//
// Arguments    : const double y_meas[13]
//                const double B_usedMeas_vec[13]
//                const double initialization_vec[4]
//                double delta
//                const double Rw[2]
//                const double Re[13]
//                double L_imuToRear
//                double L_geometricWheelbase
//                const double L_trackWidth[5]
//                const double L_axlePos[5]
//                double T
//                double xk_m_out[4]
//                double op[4]
// Return Type  : void
//
void InformationFilterUpdate(const double y_meas[13], const double
  B_usedMeas_vec[13], const double initialization_vec[4], double delta, const
  double Rw[2], const double Re[13], double L_imuToRear, double
  L_geometricWheelbase, const double L_trackWidth[5], const double L_axlePos[5],
  double T, double xk_m_out[4], double op[4])
{
  int j;
  double r;
  double t;
  static const signed char iv[4] = { 0, 0, 0, 1 };

  int c;
  static const signed char iv1[4] = { 0, 1, 0, 0 };

  int ar;
  int iy;
  int k;
  int ix;
  signed char i1;
  int i2;
  int ia;
  int jy;
  int i;
  int kAcol;
  int i3;
  static const double a[16] = { 100.0, 0.0, 0.0, 0.0, 0.0, 33.333333333333336,
    0.0, 0.0, 0.0, 0.0, 10000.0, 0.0, 0.0, 0.0, 0.0, 500.0 };

  static const signed char iv2[4] = { 1, 0, 0, 0 };

  static const signed char iv3[4] = { 0, 0, 1, 0 };

  double (*gpu_Rw)[4];
  double (*b_gpu_Rw)[2];
  double (*gpu_Rw_inv)[4];
  double (*gpu_Fk_inv)[16];
  double (*gpu_x)[16];
  signed char (*gpu_iv)[4];
  signed char (*gpu_iv1)[4];
  signed char (*gpu_ipiv)[4];
  signed char (*gpu_p)[4];
  double (*gpu_Gk)[8];
  double (*gpu_a)[16];
  double (*gpu_Ih)[16];
  double (*b_gpu_Gk)[8];
  double (*b_gpu_x)[4];
  double (*b_gpu_Ih)[8];
  double (*c_gpu_Ih)[8];
  double (*gpu_calcEq)[16];
  double (*gpu_initialization_vec)[4];
  double (*gpu_ih)[4];
  double (*gpu_ik)[4];
  double (*gpu_xk_m_out)[4];
  double (*gpu_B_usedMeas_vec)[13];
  double (*gpu_Re)[13];
  double (*gpu_Re_inv)[13];
  double (*b_gpu_Re_inv)[169];
  double (*gpu_H_linear)[52];
  signed char (*gpu_iv2)[4];
  signed char (*gpu_iv3)[4];
  double (*gpu_hk)[13];
  double (*gpu_A)[52];
  double (*gpu_y_meas)[13];
  double (*b_gpu_y_meas)[13];
  double (*gpu_C)[52];
  double (*gpu_op)[4];
  boolean_T syncIsDirty;
  cudaMallocManaged(&gpu_C, 416ULL);
  cudaMallocManaged(&b_gpu_y_meas, 104ULL);
  cudaMallocManaged(&gpu_A, 416ULL);
  cudaMallocManaged(&gpu_hk, 104ULL);
  cudaMallocManaged(&gpu_H_linear, 416ULL);
  cudaMallocManaged(&b_gpu_Re_inv, 1352ULL);
  cudaMallocManaged(&gpu_Re_inv, 104ULL);
  cudaMallocManaged(&gpu_ik, 32ULL);
  cudaMallocManaged(&gpu_ih, 32ULL);
  cudaMallocManaged(&gpu_calcEq, 128ULL);
  cudaMallocManaged(&c_gpu_Ih, 64ULL);
  cudaMallocManaged(&b_gpu_Ih, 64ULL);
  cudaMallocManaged(&b_gpu_x, 32ULL);
  cudaMallocManaged(&b_gpu_Gk, 64ULL);
  cudaMallocManaged(&gpu_Ih, 128ULL);
  cudaMallocManaged(&gpu_Gk, 64ULL);
  cudaMallocManaged(&gpu_p, 4ULL);
  cudaMallocManaged(&gpu_ipiv, 4ULL);
  cudaMallocManaged(&gpu_x, 128ULL);
  cudaMallocManaged(&gpu_Fk_inv, 128ULL);
  cudaMallocManaged(&gpu_Rw_inv, 32ULL);
  cudaMallocManaged(&gpu_Rw, 32ULL);
  cudaMallocManaged(&gpu_op, 32ULL);
  cudaMallocManaged(&gpu_xk_m_out, 32ULL);
  cudaMallocManaged(&b_gpu_Rw, 16ULL);
  cudaMallocManaged(&gpu_iv, 4ULL);
  cudaMallocManaged(&gpu_iv1, 4ULL);
  cudaMallocManaged(&gpu_a, 128ULL);
  cudaMallocManaged(&gpu_initialization_vec, 32ULL);
  cudaMallocManaged(&gpu_B_usedMeas_vec, 104ULL);
  cudaMallocManaged(&gpu_Re, 104ULL);
  cudaMallocManaged(&gpu_iv2, 4ULL);
  cudaMallocManaged(&gpu_iv3, 4ULL);
  cudaMallocManaged(&gpu_y_meas, 104ULL);
  cudaMemcpy(gpu_y_meas, (void *)&y_meas[0], 104ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv3, (void *)&iv3[0], 4ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv2, (void *)&iv2[0], 4ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_Re, (void *)&Re[0], 104ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_B_usedMeas_vec, (void *)&B_usedMeas_vec[0], 104ULL,
             cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_initialization_vec, (void *)&initialization_vec[0], 32ULL,
             cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_a, (void *)&a[0], 128ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv1, (void *)&iv1[0], 4ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv, (void *)&iv[0], 4ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(b_gpu_Rw, (void *)&Rw[0], 16ULL, cudaMemcpyHostToDevice);

  // ---- Prediction step -------------------------
  InformationFilterUpdate_kernel1<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Rw);
  InformationFilterUpdate_kernel2<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_Rw, *gpu_Rw);
  cudaDeviceSynchronize();
  if (std::abs((*gpu_Rw)[1]) > std::abs((*gpu_Rw)[0])) {
    r = (*gpu_Rw)[0] / (*gpu_Rw)[1];
    InformationFilterUpdate_kernel4<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(r,
      1.0 / (r * (*gpu_Rw)[3] - (*gpu_Rw)[2]), *gpu_Rw, *gpu_Rw_inv);
  } else {
    r = (*gpu_Rw)[1] / (*gpu_Rw)[0];
    InformationFilterUpdate_kernel3<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(r,
      1.0 / ((*gpu_Rw)[3] - r * (*gpu_Rw)[2]), *gpu_Rw, *gpu_Rw_inv);
  }

  //  System matrix
  InformationFilterUpdate_kernel5<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Fk_inv);
  InformationFilterUpdate_kernel6<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(T,
    *gpu_x);
  InformationFilterUpdate_kernel7<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_iv, *gpu_iv1, *gpu_ipiv, *gpu_x);
  syncIsDirty = true;
  for (j = 0; j < 3; j++) {
    c = j * 5;
    ar = 2 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    r = std::abs((*gpu_x)[c]);
    for (k = 0; k <= ar; k++) {
      ix++;
      t = std::abs((*gpu_x)[ix]);
      if (t > r) {
        iy = k + 1;
        r = t;
      }
    }

    if ((*gpu_x)[c + iy] != 0.0) {
      if (iy != 0) {
        (*gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        ar = iy;
        for (k = 0; k < 4; k++) {
          ix = j + k * 4;
          iy = ar + k * 4;
          r = (*gpu_x)[ix];
          (*gpu_x)[ix] = (*gpu_x)[iy];
          (*gpu_x)[iy] = r;
        }
      }

      i2 = (c - j) + 2;
      for (i = 0; i <= i2 - c; i++) {
        iy = (c + i) + 1;
        (*gpu_x)[iy] /= (*gpu_x)[c];
      }
    }

    ar = 2 - j;
    iy = c + 6;
    jy = c + 4;
    for (ia = 0; ia <= ar; ia++) {
      r = (*gpu_x)[jy];
      if ((*gpu_x)[jy] != 0.0) {
        ix = c;
        i2 = iy - 2;
        i3 = iy - j;
        for (kAcol = 0; kAcol <= i3 - i2; kAcol++) {
          i = (iy + kAcol) - 1;
          (*gpu_x)[i] += (*gpu_x)[ix + 1] * -r;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  InformationFilterUpdate_kernel8<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_p);
  syncIsDirty = true;
  for (k = 0; k < 3; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*gpu_ipiv)[k] > k + 1) {
      iy = (*gpu_p)[(*gpu_ipiv)[k] - 1];
      (*gpu_p)[(*gpu_ipiv)[k] - 1] = (*gpu_p)[k];
      (*gpu_p)[k] = static_cast<signed char>(iy);
    }
  }

  for (k = 0; k < 4; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    i1 = (*gpu_p)[k];
    (*gpu_Fk_inv)[k + (((*gpu_p)[k] - 1) << 2)] = 1.0;
    for (j = 0; j <= 3 - k; j++) {
      ia = k + j;
      if ((*gpu_Fk_inv)[ia + ((i1 - 1) << 2)] != 0.0) {
        for (i = 0; i <= 2 - ia; i++) {
          iy = (ia + i) + 1;
          (*gpu_Fk_inv)[iy + ((i1 - 1) << 2)] -= (*gpu_Fk_inv)[ia + ((i1 - 1) <<
            2)] * (*gpu_x)[iy + (ia << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    iy = (j << 2) - 1;
    for (k = 0; k < 4; k++) {
      jy = 4 - k;
      kAcol = (3 - k) << 2;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Fk_inv)[(iy - k) + 4] != 0.0) {
        (*gpu_Fk_inv)[(iy - k) + 4] /= (*gpu_x)[(kAcol - k) + 3];
        for (i = 0; i <= jy - 2; i++) {
          (*gpu_Fk_inv)[(i + iy) + 1] -= (*gpu_Fk_inv)[(iy - k) + 4] * (*gpu_x)
            [i + kAcol];
        }
      }
    }
  }

  //  Noise matrix
  InformationFilterUpdate_kernel9<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(T,
    *gpu_Gk);

  //  Prediction step alternative 2. Gives easier matrix to invert
  for (i = 0; i < 4; i++) {
    b_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*gpu_a, *gpu_Fk_inv, i, *gpu_x);
    c_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*gpu_Fk_inv, *gpu_x, i, *gpu_Ih);
  }

  d_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Ih, *gpu_Gk, *b_gpu_Gk);
  e_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Gk, *b_gpu_Gk, *gpu_Rw_inv, *b_gpu_x);
  cudaDeviceSynchronize();
  if (std::abs((*b_gpu_x)[1]) > std::abs((*b_gpu_x)[0])) {
    r = (*b_gpu_x)[0] / (*b_gpu_x)[1];
    g_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(r,
      1.0 / (r * (*b_gpu_x)[3] - (*b_gpu_x)[2]), *b_gpu_x, *gpu_Rw);
  } else {
    r = (*b_gpu_x)[1] / (*b_gpu_x)[0];
    f_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(r,
      1.0 / ((*b_gpu_x)[3] - r * (*b_gpu_x)[2]), *b_gpu_x, *gpu_Rw);
  }

  h_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Gk, *gpu_Ih, *b_gpu_Ih);
  i_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Rw, *b_gpu_Ih, *c_gpu_Ih);
  j_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Gk, *c_gpu_Ih, *gpu_calcEq);
  k_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_initialization_vec, *gpu_ih);
  l_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ih, *gpu_a, *gpu_ik);
  m_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ik, *gpu_Fk_inv, *gpu_ih);
  n_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_calcEq, *gpu_ih, *gpu_ik);
  o_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Fk_inv);
  p_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_calcEq, *gpu_Ih, *gpu_x);
  q_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 3; j++) {
    c = j * 5;
    ar = 2 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    r = std::abs((*gpu_x)[c]);
    for (k = 0; k <= ar; k++) {
      ix++;
      t = std::abs((*gpu_x)[ix]);
      if (t > r) {
        iy = k + 1;
        r = t;
      }
    }

    if ((*gpu_x)[c + iy] != 0.0) {
      if (iy != 0) {
        (*gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        ar = iy;
        for (k = 0; k < 4; k++) {
          ix = j + k * 4;
          iy = ar + k * 4;
          r = (*gpu_x)[ix];
          (*gpu_x)[ix] = (*gpu_x)[iy];
          (*gpu_x)[iy] = r;
        }
      }

      i2 = (c - j) + 2;
      for (i = 0; i <= i2 - c; i++) {
        iy = (c + i) + 1;
        (*gpu_x)[iy] /= (*gpu_x)[c];
      }
    }

    ar = 2 - j;
    iy = c + 6;
    jy = c + 4;
    for (ia = 0; ia <= ar; ia++) {
      r = (*gpu_x)[jy];
      if ((*gpu_x)[jy] != 0.0) {
        ix = c;
        i2 = iy - 2;
        i3 = iy - j;
        for (kAcol = 0; kAcol <= i3 - i2; kAcol++) {
          i = (iy + kAcol) - 1;
          (*gpu_x)[i] += (*gpu_x)[ix + 1] * -r;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  r_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_p);
  syncIsDirty = true;
  for (k = 0; k < 3; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*gpu_ipiv)[k] > k + 1) {
      iy = (*gpu_p)[(*gpu_ipiv)[k] - 1];
      (*gpu_p)[(*gpu_ipiv)[k] - 1] = (*gpu_p)[k];
      (*gpu_p)[k] = static_cast<signed char>(iy);
    }
  }

  for (k = 0; k < 4; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    i1 = (*gpu_p)[k];
    (*gpu_Fk_inv)[k + (((*gpu_p)[k] - 1) << 2)] = 1.0;
    for (j = 0; j <= 3 - k; j++) {
      ia = k + j;
      if ((*gpu_Fk_inv)[ia + ((i1 - 1) << 2)] != 0.0) {
        for (i = 0; i <= 2 - ia; i++) {
          iy = (ia + i) + 1;
          (*gpu_Fk_inv)[iy + ((i1 - 1) << 2)] -= (*gpu_Fk_inv)[ia + ((i1 - 1) <<
            2)] * (*gpu_x)[iy + (ia << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    iy = (j << 2) - 1;
    for (k = 0; k < 4; k++) {
      jy = 4 - k;
      kAcol = (3 - k) << 2;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Fk_inv)[(iy - k) + 4] != 0.0) {
        (*gpu_Fk_inv)[(iy - k) + 4] /= (*gpu_x)[(kAcol - k) + 3];
        for (i = 0; i <= jy - 2; i++) {
          (*gpu_Fk_inv)[(i + iy) + 1] -= (*gpu_Fk_inv)[(iy - k) + 4] * (*gpu_x)
            [i + kAcol];
        }
      }
    }
  }

  s_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ik, *gpu_Fk_inv, *gpu_xk_m_out);

  // ---- Measurement step --------------------------
  //  Extract front and rear track widths
  //  drive axle 1
  //  drive axle 2
  r = L_axlePos[0] + L_geometricWheelbase;
  t_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_B_usedMeas_vec, *gpu_Re, *gpu_Re_inv);
  u_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(192U, 1U, 1U)>>>
    (*b_gpu_Re_inv);
  v_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Re_inv, *b_gpu_Re_inv);

  //  Linear parts
  cudaDeviceSynchronize();
  (*gpu_H_linear)[1] = 0.0;
  (*gpu_H_linear)[14] = 0.0;
  (*gpu_H_linear)[27] = 0.0;
  (*gpu_H_linear)[40] = L_imuToRear;
  (*gpu_H_linear)[6] = 1.0;
  (*gpu_H_linear)[19] = 0.0;
  (*gpu_H_linear)[32] = -L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[45] = 0.0;
  (*gpu_H_linear)[7] = 1.0;
  (*gpu_H_linear)[20] = 0.0;
  (*gpu_H_linear)[33] = L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[46] = 0.0;
  (*gpu_H_linear)[8] = 1.0;
  (*gpu_H_linear)[21] = 0.0;
  (*gpu_H_linear)[34] = -L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[47] = 0.0;
  (*gpu_H_linear)[9] = 1.0;
  (*gpu_H_linear)[22] = 0.0;
  (*gpu_H_linear)[35] = L_trackWidth[3] / 2.0;
  w_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_H_linear);
  x_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_iv2, *gpu_iv3, *gpu_iv1, *gpu_H_linear);

  //  Nonlinear parts
  y_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_xk_m_out, *gpu_H_linear, *gpu_hk);
  cudaDeviceSynchronize();
  (*gpu_hk)[4] = ((*gpu_xk_m_out)[0] - L_trackWidth[0] * (*gpu_xk_m_out)[2] /
                  2.0) * std::cos(delta) + r * (*gpu_xk_m_out)[2] * std::sin
    (delta);
  (*gpu_hk)[5] = ((*gpu_xk_m_out)[0] + L_trackWidth[0] * (*gpu_xk_m_out)[2] /
                  2.0) * std::cos(delta) + r * (*gpu_xk_m_out)[2] * std::sin
    (delta);
  (*gpu_H_linear)[30] = -L_trackWidth[0] * std::cos(delta) / 2.0 + r * std::sin
    (delta);
  (*gpu_H_linear)[5] = std::cos(delta);
  (*gpu_H_linear)[31] = L_trackWidth[0] * std::cos(delta) / 2.0 + r * std::sin
    (delta);
  ab_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(r,
    delta, L_imuToRear, *gpu_xk_m_out, *gpu_H_linear, *gpu_hk);
  bb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_H_linear, *gpu_A);
  syncIsDirty = true;
  for (iy = 0; iy < 13; iy++) {
    kAcol = iy << 2;
    i2 = kAcol - 3;
    for (i = 0; i <= kAcol - i2; i++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*gpu_C)[kAcol + i] = 0.0;
    }
  }

  for (iy = 0; iy < 13; iy++) {
    jy = iy * 13 + 1;
    kAcol = iy << 2;
    ar = -1;
    i2 = jy - 12;
    for (ix = 0; ix <= jy - i2; ix++) {
      k = jy + ix;
      ia = ar;
      i3 = kAcol - 2;
      c = kAcol + 1;
      for (i = 0; i <= c - i3; i++) {
        j = kAcol + i;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*gpu_C)[j] += (*b_gpu_Re_inv)[k - 1] * (*gpu_A)[ia];
      }

      ar += 4;
    }
  }

  cb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_xk_m_out, *gpu_H_linear, *gpu_hk, *gpu_y_meas, *b_gpu_y_meas);
  db_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_y_meas, *gpu_C, *gpu_ik, *gpu_op);
  cudaDeviceSynchronize();
  cudaMemcpy(&xk_m_out[0], gpu_xk_m_out, 32ULL, cudaMemcpyDeviceToHost);
  cudaMemcpy(&op[0], gpu_op, 32ULL, cudaMemcpyDeviceToHost);
  cudaFree(*gpu_y_meas);
  cudaFree(*gpu_iv3);
  cudaFree(*gpu_iv2);
  cudaFree(*gpu_Re);
  cudaFree(*gpu_B_usedMeas_vec);
  cudaFree(*gpu_initialization_vec);
  cudaFree(*gpu_a);
  cudaFree(*gpu_iv1);
  cudaFree(*gpu_iv);
  cudaFree(*b_gpu_Rw);
  cudaFree(*gpu_xk_m_out);
  cudaFree(*gpu_op);
  cudaFree(*gpu_Rw);
  cudaFree(*gpu_Rw_inv);
  cudaFree(*gpu_Fk_inv);
  cudaFree(*gpu_x);
  cudaFree(*gpu_ipiv);
  cudaFree(*gpu_p);
  cudaFree(*gpu_Gk);
  cudaFree(*gpu_Ih);
  cudaFree(*b_gpu_Gk);
  cudaFree(*b_gpu_x);
  cudaFree(*b_gpu_Ih);
  cudaFree(*c_gpu_Ih);
  cudaFree(*gpu_calcEq);
  cudaFree(*gpu_ih);
  cudaFree(*gpu_ik);
  cudaFree(*gpu_Re_inv);
  cudaFree(*b_gpu_Re_inv);
  cudaFree(*gpu_H_linear);
  cudaFree(*gpu_hk);
  cudaFree(*gpu_A);
  cudaFree(*b_gpu_y_meas);
  cudaFree(*gpu_C);
}

//
// Arguments    : void
// Return Type  : void
//
void InformationFilterUpdate_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void InformationFilterUpdate_terminate()
{
  // (no terminate code required)
}

//
// File trailer for InformationFilterUpdate.cu
//
// [EOF]
//
