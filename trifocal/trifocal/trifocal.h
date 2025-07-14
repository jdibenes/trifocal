//*****************************************************************************
// Trifocal methods from
// https://github.com/LauraFJulia/TFT_vs_Fund
// in C++
//***************************************************************************** 

#pragma once

#include <Eigen/Eigen>
#include <unsupported/Eigen/src/KroneckerProduct/KroneckerTensorProduct.h>

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
compute_A
(
    T const* p1,
    T const* p2,
    T const* p3,
    int count,
    T* A // shape (4*n, 27) as RowMajor
)
{
    for (int i = 0; i < count; ++i)
    {
        T x1 = p1[i * 2 + 0];
        T y1 = p1[i * 2 + 1];
        T x2 = p2[i * 2 + 0];
        T y2 = p2[i * 2 + 1];
        T x3 = p3[i * 2 + 0];
        T y3 = p3[i * 2 + 1];

        A[(4 * i + 0) * 27 + 0] = x1;
        A[(4 * i + 0) * 27 + 1] = 0;
        A[(4 * i + 0) * 27 + 2] = -x1 * x2;
        A[(4 * i + 0) * 27 + 3] = 0;
        A[(4 * i + 0) * 27 + 4] = 0;
        A[(4 * i + 0) * 27 + 5] = 0;
        A[(4 * i + 0) * 27 + 6] = -x1 * x3;
        A[(4 * i + 0) * 27 + 7] = 0;
        A[(4 * i + 0) * 27 + 8] = x1 * x2 * x3;
        A[(4 * i + 0) * 27 + 9] = y1;
        A[(4 * i + 0) * 27 + 10] = 0;
        A[(4 * i + 0) * 27 + 11] = -x2 * y1;
        A[(4 * i + 0) * 27 + 12] = 0;
        A[(4 * i + 0) * 27 + 13] = 0;
        A[(4 * i + 0) * 27 + 14] = 0;
        A[(4 * i + 0) * 27 + 15] = -x3 * y1;
        A[(4 * i + 0) * 27 + 16] = 0;
        A[(4 * i + 0) * 27 + 17] = x2 * x3 * y1;
        A[(4 * i + 0) * 27 + 18] = 1;
        A[(4 * i + 0) * 27 + 19] = 0;
        A[(4 * i + 0) * 27 + 20] = -x2;
        A[(4 * i + 0) * 27 + 21] = 0;
        A[(4 * i + 0) * 27 + 22] = 0;
        A[(4 * i + 0) * 27 + 23] = 0;
        A[(4 * i + 0) * 27 + 24] = -x3;
        A[(4 * i + 0) * 27 + 25] = 0;
        A[(4 * i + 0) * 27 + 26] = x2 * x3;

        A[(4 * i + 1) * 27 + 0] = 0;
        A[(4 * i + 1) * 27 + 1] = x1;
        A[(4 * i + 1) * 27 + 2] = -x1 * y2;
        A[(4 * i + 1) * 27 + 3] = 0;
        A[(4 * i + 1) * 27 + 4] = 0;
        A[(4 * i + 1) * 27 + 5] = 0;
        A[(4 * i + 1) * 27 + 6] = 0;
        A[(4 * i + 1) * 27 + 7] = -x1 * x3;
        A[(4 * i + 1) * 27 + 8] = x1 * x3 * y2;
        A[(4 * i + 1) * 27 + 9] = 0;
        A[(4 * i + 1) * 27 + 10] = y1;
        A[(4 * i + 1) * 27 + 11] = -y1 * y2;
        A[(4 * i + 1) * 27 + 12] = 0;
        A[(4 * i + 1) * 27 + 13] = 0;
        A[(4 * i + 1) * 27 + 14] = 0;
        A[(4 * i + 1) * 27 + 15] = 0;
        A[(4 * i + 1) * 27 + 16] = -x3 * y1;
        A[(4 * i + 1) * 27 + 17] = x3 * y1 * y2;
        A[(4 * i + 1) * 27 + 18] = 0;
        A[(4 * i + 1) * 27 + 19] = 1;
        A[(4 * i + 1) * 27 + 20] = -y2;
        A[(4 * i + 1) * 27 + 21] = 0;
        A[(4 * i + 1) * 27 + 22] = 0;
        A[(4 * i + 1) * 27 + 23] = 0;
        A[(4 * i + 1) * 27 + 24] = 0;
        A[(4 * i + 1) * 27 + 25] = -x3;
        A[(4 * i + 1) * 27 + 26] = x3 * y2;

        A[(4 * i + 2) * 27 + 0] = 0;
        A[(4 * i + 2) * 27 + 1] = 0;
        A[(4 * i + 2) * 27 + 2] = 0;
        A[(4 * i + 2) * 27 + 3] = x1;
        A[(4 * i + 2) * 27 + 4] = 0;
        A[(4 * i + 2) * 27 + 5] = -x1 * x2;
        A[(4 * i + 2) * 27 + 6] = -x1 * y3;
        A[(4 * i + 2) * 27 + 7] = 0;
        A[(4 * i + 2) * 27 + 8] = x1 * x2 * y3;
        A[(4 * i + 2) * 27 + 9] = 0;
        A[(4 * i + 2) * 27 + 10] = 0;
        A[(4 * i + 2) * 27 + 11] = 0;
        A[(4 * i + 2) * 27 + 12] = y1;
        A[(4 * i + 2) * 27 + 13] = 0;
        A[(4 * i + 2) * 27 + 14] = -x2 * y1;
        A[(4 * i + 2) * 27 + 15] = -y1 * y3;
        A[(4 * i + 2) * 27 + 16] = 0;
        A[(4 * i + 2) * 27 + 17] = x2 * y1 * y3;
        A[(4 * i + 2) * 27 + 18] = 0;
        A[(4 * i + 2) * 27 + 19] = 0;
        A[(4 * i + 2) * 27 + 20] = 0;
        A[(4 * i + 2) * 27 + 21] = 1;
        A[(4 * i + 2) * 27 + 22] = 0;
        A[(4 * i + 2) * 27 + 23] = -x2;
        A[(4 * i + 2) * 27 + 24] = -y3;
        A[(4 * i + 2) * 27 + 25] = 0;
        A[(4 * i + 2) * 27 + 26] = x2 * y3;

        A[(4 * i + 3) * 27 + 0] = 0;
        A[(4 * i + 3) * 27 + 1] = 0;
        A[(4 * i + 3) * 27 + 2] = 0;
        A[(4 * i + 3) * 27 + 3] = 0;
        A[(4 * i + 3) * 27 + 4] = x1;
        A[(4 * i + 3) * 27 + 5] = -x1 * y2;
        A[(4 * i + 3) * 27 + 6] = 0;
        A[(4 * i + 3) * 27 + 7] = -x1 * y3;
        A[(4 * i + 3) * 27 + 8] = x1 * y2 * y3;
        A[(4 * i + 3) * 27 + 9] = 0;
        A[(4 * i + 3) * 27 + 10] = 0;
        A[(4 * i + 3) * 27 + 11] = 0;
        A[(4 * i + 3) * 27 + 12] = 0;
        A[(4 * i + 3) * 27 + 13] = y1;
        A[(4 * i + 3) * 27 + 14] = -y1 * y2;
        A[(4 * i + 3) * 27 + 15] = 0;
        A[(4 * i + 3) * 27 + 16] = -y1 * y3;
        A[(4 * i + 3) * 27 + 17] = y1 * y2 * y3;
        A[(4 * i + 3) * 27 + 18] = 0;
        A[(4 * i + 3) * 27 + 19] = 0;
        A[(4 * i + 3) * 27 + 20] = 0;
        A[(4 * i + 3) * 27 + 21] = 0;
        A[(4 * i + 3) * 27 + 22] = 1;
        A[(4 * i + 3) * 27 + 23] = -y2;
        A[(4 * i + 3) * 27 + 24] = 0;
        A[(4 * i + 3) * 27 + 25] = -y3;
        A[(4 * i + 3) * 27 + 26] = y2 * y3;
    }
}

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
linear_TFT
(
    T const* A,
    int rows,
    T* TFT
)
{
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>> A_m(A, rows, 27);

    Eigen::Matrix<T, 27, 1> t = (A_m.bdcSvd(Eigen::ComputeFullV).matrixV())(Eigen::all, Eigen::last);

    Eigen::Map<Eigen::Matrix<T, 3, 3>> T1(t.data() + 0);
    Eigen::Map<Eigen::Matrix<T, 3, 3>> T2(t.data() + 9);
    Eigen::Map<Eigen::Matrix<T, 3, 3>> T3(t.data() + 18);

    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_t1 = T1.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_t2 = T2.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_t3 = T3.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);

    Eigen::Matrix<T, 3, 1> v1 = (svd_t1.matrixV())(Eigen::all, Eigen::last);
    Eigen::Matrix<T, 3, 1> v2 = (svd_t2.matrixV())(Eigen::all, Eigen::last);
    Eigen::Matrix<T, 3, 1> v3 = (svd_t3.matrixV())(Eigen::all, Eigen::last);

    Eigen::Matrix<T, 3, 3> vx;
    vx << v1, v2, v3;
    Eigen::Matrix<T, 3, 1> epi31 = -((vx.jacobiSvd(Eigen::ComputeFullU).matrixU())(Eigen::all, Eigen::last));

    Eigen::Matrix<T, 3, 1> u1 = -((svd_t1.matrixU())(Eigen::all, Eigen::last));
    Eigen::Matrix<T, 3, 1> u2 = -((svd_t2.matrixU())(Eigen::all, Eigen::last));
    Eigen::Matrix<T, 3, 1> u3 = -((svd_t3.matrixU())(Eigen::all, Eigen::last));

    Eigen::Matrix<T, 3, 3> ux;
    ux << u1, u2, u3;
    Eigen::Matrix<T, 3, 1> epi21 = -((ux.jacobiSvd(Eigen::ComputeFullU).matrixU())(Eigen::all, Eigen::last));

    Eigen::Matrix<T, 3, 3> I3 = Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 9, 9> I9 = Eigen::Matrix<T, 9, 9>::Identity();

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> E(27, 18);
    E << Eigen::kroneckerProduct(I3, Eigen::kroneckerProduct(epi31, I3)), Eigen::kroneckerProduct(I9, -epi21);

    Eigen::BDCSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> E_svd = E.bdcSvd(Eigen::ComputeFullU);
    Eigen::Index E_rank = E_svd.rank();
    int64_t E_last = E_rank - 1;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Up = (E_svd.matrixU())(Eigen::all, Eigen::seq(0, E_last));

    Eigen::Map<Eigen::Matrix<T, 27, 1>> result(TFT);
    result = Up * ((A_m * Up).bdcSvd(Eigen::ComputeFullV).matrixV())(Eigen::all, Eigen::last);
}
