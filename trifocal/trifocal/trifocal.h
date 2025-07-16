//*****************************************************************************
// Trifocal methods from
// https://github.com/LauraFJulia/TFT_vs_Fund
// in C++
//***************************************************************************** 

#pragma once

#include <Eigen/Eigen>
#include <Eigen/QR>
#include <Eigen/Geometry>
#include <unsupported/Eigen/src/KroneckerProduct/KroneckerTensorProduct.h>

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
build_A
(
    T const* points_2D,
    int count,
    T* A // shape (4*n, 27) as RowMajor
)
{
    for (int i = 0; i < count; ++i)
    {
        T x1 = points_2D[(i * 6) + 0];
        T y1 = points_2D[(i * 6) + 1];
        T x2 = points_2D[(i * 6) + 2];
        T y2 = points_2D[(i * 6) + 3];
        T x3 = points_2D[(i * 6) + 4];
        T y3 = points_2D[(i * 6) + 5];

        A[(((4 * i) + 0) * 27) +  0] = x1;
        A[(((4 * i) + 0) * 27) +  1] = 0;
        A[(((4 * i) + 0) * 27) +  2] = -x1 * x2;
        A[(((4 * i) + 0) * 27) +  3] = 0;
        A[(((4 * i) + 0) * 27) +  4] = 0;
        A[(((4 * i) + 0) * 27) +  5] = 0;
        A[(((4 * i) + 0) * 27) +  6] = -x1 * x3;
        A[(((4 * i) + 0) * 27) +  7] = 0;
        A[(((4 * i) + 0) * 27) +  8] = x1 * x2 * x3;
        A[(((4 * i) + 0) * 27) +  9] = y1;
        A[(((4 * i) + 0) * 27) + 10] = 0;
        A[(((4 * i) + 0) * 27) + 11] = -x2 * y1;
        A[(((4 * i) + 0) * 27) + 12] = 0;
        A[(((4 * i) + 0) * 27) + 13] = 0;
        A[(((4 * i) + 0) * 27) + 14] = 0;
        A[(((4 * i) + 0) * 27) + 15] = -x3 * y1;
        A[(((4 * i) + 0) * 27) + 16] = 0;
        A[(((4 * i) + 0) * 27) + 17] = x2 * x3 * y1;
        A[(((4 * i) + 0) * 27) + 18] = 1;
        A[(((4 * i) + 0) * 27) + 19] = 0;
        A[(((4 * i) + 0) * 27) + 20] = -x2;
        A[(((4 * i) + 0) * 27) + 21] = 0;
        A[(((4 * i) + 0) * 27) + 22] = 0;
        A[(((4 * i) + 0) * 27) + 23] = 0;
        A[(((4 * i) + 0) * 27) + 24] = -x3;
        A[(((4 * i) + 0) * 27) + 25] = 0;
        A[(((4 * i) + 0) * 27) + 26] = x2 * x3;

        A[(((4 * i) + 1) * 27) +  0] = 0;
        A[(((4 * i) + 1) * 27) +  1] = x1;
        A[(((4 * i) + 1) * 27) +  2] = -x1 * y2;
        A[(((4 * i) + 1) * 27) +  3] = 0;
        A[(((4 * i) + 1) * 27) +  4] = 0;
        A[(((4 * i) + 1) * 27) +  5] = 0;
        A[(((4 * i) + 1) * 27) +  6] = 0;
        A[(((4 * i) + 1) * 27) +  7] = -x1 * x3;
        A[(((4 * i) + 1) * 27) +  8] = x1 * x3 * y2;
        A[(((4 * i) + 1) * 27) +  9] = 0;
        A[(((4 * i) + 1) * 27) + 10] = y1;
        A[(((4 * i) + 1) * 27) + 11] = -y1 * y2;
        A[(((4 * i) + 1) * 27) + 12] = 0;
        A[(((4 * i) + 1) * 27) + 13] = 0;
        A[(((4 * i) + 1) * 27) + 14] = 0;
        A[(((4 * i) + 1) * 27) + 15] = 0;
        A[(((4 * i) + 1) * 27) + 16] = -x3 * y1;
        A[(((4 * i) + 1) * 27) + 17] = x3 * y1 * y2;
        A[(((4 * i) + 1) * 27) + 18] = 0;
        A[(((4 * i) + 1) * 27) + 19] = 1;
        A[(((4 * i) + 1) * 27) + 20] = -y2;
        A[(((4 * i) + 1) * 27) + 21] = 0;
        A[(((4 * i) + 1) * 27) + 22] = 0;
        A[(((4 * i) + 1) * 27) + 23] = 0;
        A[(((4 * i) + 1) * 27) + 24] = 0;
        A[(((4 * i) + 1) * 27) + 25] = -x3;
        A[(((4 * i) + 1) * 27) + 26] = x3 * y2;

        A[(((4 * i) + 2) * 27) +  0] = 0;
        A[(((4 * i) + 2) * 27) +  1] = 0;
        A[(((4 * i) + 2) * 27) +  2] = 0;
        A[(((4 * i) + 2) * 27) +  3] = x1;
        A[(((4 * i) + 2) * 27) +  4] = 0;
        A[(((4 * i) + 2) * 27) +  5] = -x1 * x2;
        A[(((4 * i) + 2) * 27) +  6] = -x1 * y3;
        A[(((4 * i) + 2) * 27) +  7] = 0;
        A[(((4 * i) + 2) * 27) +  8] = x1 * x2 * y3;
        A[(((4 * i) + 2) * 27) +  9] = 0;
        A[(((4 * i) + 2) * 27) + 10] = 0;
        A[(((4 * i) + 2) * 27) + 11] = 0;
        A[(((4 * i) + 2) * 27) + 12] = y1;
        A[(((4 * i) + 2) * 27) + 13] = 0;
        A[(((4 * i) + 2) * 27) + 14] = -x2 * y1;
        A[(((4 * i) + 2) * 27) + 15] = -y1 * y3;
        A[(((4 * i) + 2) * 27) + 16] = 0;
        A[(((4 * i) + 2) * 27) + 17] = x2 * y1 * y3;
        A[(((4 * i) + 2) * 27) + 18] = 0;
        A[(((4 * i) + 2) * 27) + 19] = 0;
        A[(((4 * i) + 2) * 27) + 20] = 0;
        A[(((4 * i) + 2) * 27) + 21] = 1;
        A[(((4 * i) + 2) * 27) + 22] = 0;
        A[(((4 * i) + 2) * 27) + 23] = -x2;
        A[(((4 * i) + 2) * 27) + 24] = -y3;
        A[(((4 * i) + 2) * 27) + 25] = 0;
        A[(((4 * i) + 2) * 27) + 26] = x2 * y3;

        A[(((4 * i) + 3) * 27) +  0] = 0;
        A[(((4 * i) + 3) * 27) +  1] = 0;
        A[(((4 * i) + 3) * 27) +  2] = 0;
        A[(((4 * i) + 3) * 27) +  3] = 0;
        A[(((4 * i) + 3) * 27) +  4] = x1;
        A[(((4 * i) + 3) * 27) +  5] = -x1 * y2;
        A[(((4 * i) + 3) * 27) +  6] = 0;
        A[(((4 * i) + 3) * 27) +  7] = -x1 * y3;
        A[(((4 * i) + 3) * 27) +  8] = x1 * y2 * y3;
        A[(((4 * i) + 3) * 27) +  9] = 0;
        A[(((4 * i) + 3) * 27) + 10] = 0;
        A[(((4 * i) + 3) * 27) + 11] = 0;
        A[(((4 * i) + 3) * 27) + 12] = 0;
        A[(((4 * i) + 3) * 27) + 13] = y1;
        A[(((4 * i) + 3) * 27) + 14] = -y1 * y2;
        A[(((4 * i) + 3) * 27) + 15] = 0;
        A[(((4 * i) + 3) * 27) + 16] = -y1 * y3;
        A[(((4 * i) + 3) * 27) + 17] = y1 * y2 * y3;
        A[(((4 * i) + 3) * 27) + 18] = 0;
        A[(((4 * i) + 3) * 27) + 19] = 0;
        A[(((4 * i) + 3) * 27) + 20] = 0;
        A[(((4 * i) + 3) * 27) + 21] = 0;
        A[(((4 * i) + 3) * 27) + 22] = 1;
        A[(((4 * i) + 3) * 27) + 23] = -y2;
        A[(((4 * i) + 3) * 27) + 24] = 0;
        A[(((4 * i) + 3) * 27) + 25] = -y3;
        A[(((4 * i) + 3) * 27) + 26] = y2 * y3;
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

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
TFT_from_P
(
    T const* projection_1,
    T const* projection_2,
    T const* projection_3,
    T* TFT
)
{
    Eigen::Map<const Eigen::Matrix<T, 3, 4>> P1(projection_1);
    Eigen::Map<const Eigen::Matrix<T, 3, 4>> P2(projection_2);
    Eigen::Map<const Eigen::Matrix<T, 3, 4>> P3(projection_3);

    Eigen::Matrix<T, 2, 4> P1i;
    Eigen::Matrix<T, 1, 4> P2j;
    Eigen::Matrix<T, 1, 4> P3k;
    Eigen::Matrix<T, 4, 4> D;

    T norm = 0;

    for (int i = 0; i < 3; ++i)
    {
        switch (i)
        {
        case 0: P1i = P1(Eigen::seqN(1, 2), Eigen::all); break;
        case 1: P1i = P1(Eigen::seqN(0, 2, 2), Eigen::all); break;
        case 2: P1i = P1(Eigen::seqN(0, 2), Eigen::all); break;
        }
        for (int k = 0; k < 3; ++k)
        {
            P3k = P3(k, Eigen::all);
            for (int j = 0; j < 3; ++j)
            {
                P2j = P2(j, Eigen::all);
                D << P1i, P2j, P3k;
                T t = (1 - (2 * (i & 1))) * D.determinant();
                norm += t * t;
                TFT[i * 9 + k * 3 + j] = t;
            }
        }
    }

    norm = sqrt(norm);
    for (int i = 0; i < 27; ++i) { TFT[i] /= norm; }
}

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
crossM
(
    T const* v,
    T* M
)
{
    Eigen::Map<Eigen::Matrix<T, 3, 3>> M_m(M);
    M_m <<     0,  (-v[2]),   v[1], 
             v[2],     0,   (-v[0]),
           (-v[1]),  v[0],      0;
}

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
triangulate
(
    T const* cameras,
    int count_cameras,
    T const* points_2D,
    int count_points,
    T* points_3D
)
{
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> P_m(cameras, 3, 4 * count_cameras);
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> p2D_m(points_2D, 2 * count_cameras, count_points);
    Eigen::Map<      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> p3D_m(points_3D, 4, count_points);

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ls_matrix(2 * count_cameras, 4);
    Eigen::Matrix<T, 2, 3> L{
        {0, -1, 0.0},
        {1,  0, 0.0},
    };

    for (int n = 0; n < count_points; ++n)
    {
        for (int i = 0; i < count_cameras; ++i)
        {
            L(0, 2) =  p2D_m((i * 2) + 1, n);
            L(1, 2) = -p2D_m((i * 2) + 0, n);
            ls_matrix(Eigen::seq((i * 2) + 0, (i * 2) + 1), Eigen::all) = L * P_m(Eigen::all, Eigen::seq((i * 4) + 0, (i * 4) + 3));
        }
        p3D_m.col(n) = ls_matrix.bdcSvd(Eigen::ComputeFullV).matrixV().col(3);
    }
}

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
R_t_from_E
(
    T const* E,
    T const* points2D,
    int count_points,
    T* P
)
{
    Eigen::Matrix<T, 3, 3> W{
        {0, -1, 0},
        {1,  0, 0},
        {0,  0, 1},
    };

    Eigen::Matrix<T, 3, 3> Wt{
        { 0, 1, 0},
        {-1, 0, 0},
        { 0, 0, 1},
    };

    Eigen::Map<const Eigen::Matrix<T, 3, 3>> E_m(E);

    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> E_svd = E_m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<T, 3, 3> U  = E_svd.matrixU();
    Eigen::Matrix<T, 3, 3> Vt = E_svd.matrixV().transpose();

    Eigen::Matrix<T, 3, 3> R1 = U * W  * Vt;
    Eigen::Matrix<T, 3, 3> R2 = U * Wt * Vt;
    if (R1.determinant() < 0) { R1 = -R1; }
    if (R2.determinant() < 0) { R2 = -R2; }

    Eigen::Matrix<T, 3, 1> t  = U.col(2);
    Eigen::Matrix<T, 3, 1> nt = -t;

    Eigen::Matrix<T, 3, 4> P0 = Eigen::Matrix<T, 3, 4>::Identity();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cameras(3, 4 * 4 * 2);
    cameras << P0, R1, t, P0, R1, nt, P0, R2, t, P0, R2, nt;

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> XYZW1(4, count_points);
    //Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> XYZW2(4, count_points);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> XYZ1(3, count_points);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> XYZ2(3, count_points);
    
    int64_t max_count = 0;
    int64_t select = 3;

    for (int i = 0; i < 4; ++i)
    {
        triangulate(cameras.data() + (i * 3 * 4 * 2), 2, points2D, count_points, XYZW1.data());
        XYZ2 = cameras(Eigen::all, Eigen::seq((i * 8) + 4, (i * 8) + 7)) * XYZW1;
        XYZ1 = XYZW1.colwise().hnormalized();
        int64_t count = (XYZ1(2, Eigen::all).array() >= 0).count() + (XYZ2(2, Eigen::all).array() >= 0).count();
        if (count < max_count) { continue; }
        max_count = count;
        select = i;
    }

    Eigen::Map<Eigen::Matrix<T, 3, 4>> P_m(P);
    P_m = cameras(Eigen::all, Eigen::seq((select * 8) + 4, (select * 8) + 7));
}

//-----------------------------------------------------------------------------
// OK
//-----------------------------------------------------------------------------
template <typename T>
void
R_t_from_TFT
(
    T const* TFT,
    T const* points2D,
    int count_points,
    T* Rt01,
    T* Rt02
)
{
    Eigen::Map<const Eigen::Matrix<T, 3, 3>> T1(TFT + 0);
    Eigen::Map<const Eigen::Matrix<T, 3, 3>> T2(TFT + 9);
    Eigen::Map<const Eigen::Matrix<T, 3, 3>> T3(TFT + 18);

    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_t1 = T1.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_t2 = T2.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd_t3 = T3.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);

    Eigen::Matrix<T, 3, 1> v1 = (svd_t1.matrixV())(Eigen::all, Eigen::last);
    Eigen::Matrix<T, 3, 1> v2 = (svd_t2.matrixV())(Eigen::all, Eigen::last);
    Eigen::Matrix<T, 3, 1> v3 = (svd_t3.matrixV())(Eigen::all, Eigen::last);

    Eigen::Matrix<T, 3, 3> vx;
    vx << v1, v2, v3;
    Eigen::Matrix<T, 3, 1> epi31 = -((vx.jacobiSvd(Eigen::ComputeFullU).matrixU())(Eigen::all, Eigen::last));

    if (epi31(Eigen::last, Eigen::last) < 0) { epi31 = -epi31; }

    Eigen::Matrix<T, 3, 1> u1 = -((svd_t1.matrixU())(Eigen::all, Eigen::last));
    Eigen::Matrix<T, 3, 1> u2 = -((svd_t2.matrixU())(Eigen::all, Eigen::last));
    Eigen::Matrix<T, 3, 1> u3 = -((svd_t3.matrixU())(Eigen::all, Eigen::last));

    Eigen::Matrix<T, 3, 3> ux;
    ux << u1, u2, u3;
    Eigen::Matrix<T, 3, 1> epi21 = -((ux.jacobiSvd(Eigen::ComputeFullU).matrixU())(Eigen::all, Eigen::last));

    if (epi21(Eigen::last, Eigen::last) < 0) { epi21 = -epi21; }

    Eigen::Matrix<T, 3, 3> epi21_x;
    crossM(epi21.data(), epi21_x.data());

    Eigen::Matrix<T, 3, 3> epi31_x;
    crossM(epi31.data(), epi31_x.data());

    Eigen::Matrix<T, 3, 3> E21;
    E21 << (epi21_x * (T1 * epi31)), (epi21_x * (T2 * epi31)), (epi21_x * (T3 * epi31));

    Eigen::Matrix<T, 3, 3> E31;
    E31 << (epi31_x * (T1.transpose() * epi21)), (epi31_x * (T2.transpose() * epi21)), (epi31_x * (T3.transpose() * epi21));
    E31 = -E31;

    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> p2D(points2D, 6, count_points);

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> p21(4, count_points);
    p21 << p2D(Eigen::seq(0, 1), Eigen::all), p2D(Eigen::seq(2, 3), Eigen::all);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> p31(4, count_points);
    p31 << p2D(Eigen::seq(0, 1), Eigen::all), p2D(Eigen::seq(4, 5), Eigen::all);

    R_t_from_E(E21.data(), p21.data(), count_points, Rt01);
    R_t_from_E(E31.data(), p31.data(), count_points, Rt02);

    Eigen::Matrix<T, 3, 4> c0 = Eigen::Matrix<T, 3, 4>::Identity();
    Eigen::Map<Eigen::Matrix<T, 3, 4>> c1(Rt01);
    Eigen::Map<Eigen::Matrix<T, 3, 4>> c2(Rt02);
    Eigen::Matrix<T, 3, 4 * 2> cameras;
    cameras << c0, c1;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> p3D(4, count_points);

    triangulate(cameras.data(), 2, p21.data(), count_points, p3D.data());

    Eigen::Matrix<T, 3, Eigen::Dynamic> X(3, count_points);
    Eigen::Matrix<T, 3, Eigen::Dynamic> X3(3, count_points);
    X = p3D.colwise().hnormalized();
    X3 = c2(Eigen::all, Eigen::seq(0, 2)) * X;

    Eigen::Matrix<T, 3, Eigen::Dynamic> p3(3, count_points);
    p3 << p2D(Eigen::seq(4, 5), Eigen::all).colwise().homogeneous();

    Eigen::Matrix<T, 3, Eigen::Dynamic> p3_X3(3, count_points);
    Eigen::Matrix<T, 3, Eigen::Dynamic> p3_t3(3, count_points);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X3dt3(1, count_points);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> p3dt3(1, count_points);
    T numerator = 0;
    T denominator = 0;

    for (int i = 0; i < count_points; ++i)
    {
        p3_X3.col(i) = p3.col(i).cross(X3.col(i));
        p3_t3.col(i) = p3.col(i).cross(c2.col(3));
        X3dt3(0, i) = p3_X3.col(i).dot(p3_t3.col(i));
        p3dt3(0, i) = p3_t3.col(i).dot(p3_t3.col(i));
        numerator -= X3dt3(0, i);
        denominator += p3dt3(0, i);
    }

    c2.col(3) = (numerator / denominator) * c2.col(3);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
template <typename T>
void
compute_fixed_cost_A
(
    T const* A,
    int rows,
    T* R1tR1
)
{
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>> A_m(A, rows, 27);
    Eigen::HouseholderQR<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>> qr = A_m.householderQr();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R0 = qr.matrixQR().triangularView<Eigen::Upper>();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R = R0(Eigen::seq(0, 26), Eigen::seq(0, 26));
    Eigen::Map<Eigen::Matrix<T, 27, 27>> R1tR1_m(R1tR1);
    R1tR1_m = R.transpose() * R;  
}
