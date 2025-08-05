// trifocal_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#define GLOG_USE_GLOG_EXPORT
#define GLOG_NO_ABBREVIATED_SEVERITIES
#define CERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS
#define NOMINMAX
//#include <ceres/ceres.h>
//#include <ceres/rotation.h>
#include <iostream>
#include "loader.h"
#include "trifocal.h"


/*
struct TrifocalReprojectionError
{
    double observed_x;
    double observed_y;

    template <typename T>
    bool operator()
        (
        T const* const cameraI,
        T const* const cameraJ,
        T const* const cameraK,
        T* residuals
        ) const 
    {
    }
};
*/


int main()
{
    Eigen::Matrix<double, 4, 4> pose0 = load_pose("C:/Users/jcds/Desktop/trico/hl2_5/pose/000061.bin").cast<double>();
    Eigen::Matrix<double, 4, 4> pose1 = load_pose("C:/Users/jcds/Desktop/trico/hl2_5/pose/000065.bin").cast<double>();
    Eigen::Matrix<double, 4, 4> pose2 = load_pose("C:/Users/jcds/Desktop/trico/hl2_5/pose/000067.bin").cast<double>();

    pose0.transposeInPlace();
    pose1.transposeInPlace();
    pose2.transposeInPlace();

    Eigen::Matrix<double, 3, 4> pose00 = (pose0.inverse() * pose0)(Eigen::seq(0, 3), Eigen::all);
    Eigen::Matrix<double, 3, 4> pose01 = (pose0.inverse() * pose1)(Eigen::seq(0, 3), Eigen::all);
    Eigen::Matrix<double, 3, 4> pose02 = (pose0.inverse() * pose2)(Eigen::seq(0, 3), Eigen::all);

    Eigen::Matrix<double, 4, 7> p1{
        {1,   2, -3, -1.5, 4, -5, 1.5},
        {2,  -1, -2,  1.2, 3,  4,  -6},
        {10, 12, 15,    7, 9, 16,  19},
        {1,   1,  1,    1, 1,  1,   1},
    };
    
    Eigen::Matrix<double, 3, 7> p11 = pose00 * p1;
    Eigen::Matrix<double, 3, 7> p21 = pose01 * p1;
    Eigen::Matrix<double, 3, 7> p31 = pose02 * p1;

    std::cout << "IMAGE POINTS" << std::endl;
    std::cout << p11 << std::endl;
    std::cout << p21 << std::endl;
    std::cout << p31 << std::endl;

    Eigen::Matrix<double, 2, 7> x11 = p11.colwise().hnormalized();
    Eigen::Matrix<double, 2, 7> x21 = p21.colwise().hnormalized();
    Eigen::Matrix<double, 2, 7> x31 = p31.colwise().hnormalized();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor> A(4 * 7, 27);
    Eigen::Matrix<double, 27, 1> TFT;
    Eigen::Matrix<double, 3, 4> P2;
    Eigen::Matrix<double, 3, 4> P3;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> points2D(6, 7);
    points2D << x11, x21, x31;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Ac(27, 27);

    build_A(points2D.data(), 7, A.data());
    linear_TFT(A.data(), 4 * 7, TFT.data());
    R_t_from_TFT(TFT.data(), points2D.data(), 7, P2.data(), P3.data());
    fixed_cost_from_A(A.data(), 4 * 7, Ac.data());

    std::cout << "POSES" << std::endl;
    std::cout << pose01 << std::endl;
    std::cout << P2 << std::endl;
    std::cout << pose02 << std::endl;
    std::cout << P3 << std::endl;
    std::cout << pose01.col(3) / pose01(2, 3) << std::endl;
    std::cout << P2.col(3) / P2(2, 3) << std::endl;
    std::cout << pose02.col(3) / pose01(2, 3) << std::endl;
    std::cout << P3.col(3) / P2(2, 3) << std::endl;

    std::cout << "COST" << std::endl;
    std::cout << TFT.transpose() * Ac * TFT << std::endl;



    return 0;


    /*
    Eigen::Matrix<double, 6, 1> trip1;
    trip1 << x11.col(0), x21.col(0), x31.col(0);
    Eigen::Matrix<double, 6, 1> trip2;
    trip2 << x11.col(1), x21.col(1), x31.col(1);
    Eigen::Matrix<double, 6, 2> trip;
    trip << trip1, trip2;

    Eigen::Matrix<double, 4, 2> trip_3D;
    Eigen::Matrix<double, 3, 4 * 3> cim;
    cim << pose00, pose01, pose02;

    triangulate(cim.data(), 3, trip.data(), 2, trip_3D.data());

    std::cout << "Triangulation" << std::endl;
    std::cout << trip_3D.colwise().hnormalized() << std::endl;




    Eigen::Matrix<double, 3, 3> t_x;
    crossM(pose01.data() + 9, t_x.data());
    Eigen::Matrix<double, 3, 3> E01 = t_x * pose01(Eigen::seq(0, 2), Eigen::seq(0, 2));
    Eigen::Matrix<double, 4, 1> frt2D1;
    frt2D1 << x11.col(0), x21.col(0);
    Eigen::Matrix<double, 3, 4> P_est;

    R_t_from_E(E01.data(), frt2D1.data(), 1, P_est.data());

    std::cout << "POSE" << std::endl;
    std::cout << pose01 << std::endl;
    std::cout << P_est << std::endl;
    std::cout << pose01.col(3).colwise().hnormalized() << std::endl;
    std::cout << P_est.col(3).colwise().hnormalized() << std::endl;



    return 0;
    */

    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor> A(4 * 7, 27);
    
    Eigen::Matrix<double, 27, 1> TFT_P;
    Eigen::Matrix<double, 3, 4> P1;
    


    //std::cout << pose00(Eigen::all, 3) / pose01(2, 3) << std::endl;
    //std::cout << pose01(Eigen::all, 3) / pose01(2, 3) << std::endl;
    //std::cout << pose02(Eigen::all, 3) / pose02(2, 3) << std::endl;

    build_A(points2D.data(), 7, A.data());
    linear_TFT(A.data(), 4 * 7, TFT.data());
    TFT_from_P(pose00.data(), pose01.data(), pose02.data(), TFT_P.data());

    Eigen::HouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor>> qr(A.rows(), A.cols());
    qr.compute(A);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R = qr.matrixQR().triangularView<Eigen::Upper>();

    std::cout << "QR" << std::endl;
    std::cout << R << std::endl;
    std::cout << "SIZE: " << R.rows() << ", " << R.cols() << std::endl;


    std::cout << "TFT" << std::endl;
    std::cout << TFT << std::endl;
    std::cout << "TFT_error" << std::endl;
    std::cout << A * TFT << std::endl;
    std::cout << "TFT_P" << std::endl;
    std::cout << TFT_P << std::endl;
    std::cout << "TFT_P_error" << std::endl;
    std::cout << A * TFT_P << std::endl;
    /*
    * , P1, P2, P3
    std::cout << pose00 << std::endl;
    std::cout << P1 << std::endl;
    std::cout << pose01 / pose01(2, 3) << std::endl;
    std::cout << P2 / P2(2, 3) << std::endl;
    std::cout << pose02 / pose02(2, 3) << std::endl;
    std::cout << P3 / P3(2, 3) << std::endl;
    */
   

    return 0;

    /*
     std::cout << p11 << std::endl;
    std::cout << x11 << std::endl;
    std::cout << p21 << std::endl;
    std::cout << x21 << std::endl;
    std::cout << p31 << std::endl;
    std::cout << x31 << std::endl;
    

    
    */
    /*
    //std::cout << A << std::endl;
    
    std::cout << TFT << std::endl;
    
    */

    //std::cout << x11 << std::endl;
    //std::cout << x21 << std::endl;
    //std::cout << x31 << std::endl;
}


//auto flow0 = load_flow("C:/Users/jcds/Desktop/trico/hl2_5/flow_gt/000061.flo");
     //std::cout << flow0.rows() << ", " << flow0.cols() << std::endl;
 //std::cout << flow0(Eigen::seqN(0, 4), Eigen::seqN(0, 4)) << std::endl;



// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

/*
    double x[100];
    double y[100];
    //double coefs[2] = { 100, 400 };
    double coefs[2] = { 50, 20 };

    for (int i = 0; i < 100; i++)
    {
        x[i] = i;
        y[i] = 123 * i * i + 456 + ((i % 11) - 5);
    }

    ceres::Problem problem;
    for (int i = 0; i < 100; ++i)
    {
        problem.AddResidualBlock(simple_qudratic::Create(i, 123 * i * i + 456 + ((i % 11) - 5)), nullptr, coefs);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "out: " << coefs[0] << ", " << coefs[1] << std::endl;
    */

/*
struct SnavelyReprojectionError
{
    double observed_x;
    double observed_y;

    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y)
    {
    }

    template <typename T>
    bool operator()(const T* const camera,
        const T* const point,
        T* residuals) const {
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        T const& focal = camera[6];

        T predicted_x = focal * xp;
        T predicted_y = focal * yp;

        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;

        return true;
    }

    static ceres::CostFunction* Create(const double observed_x,
        const double observed_y)
    {
        return new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 7, 3>(observed_x, observed_y);
    }

};
*/
/*
struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {
    }

    template <typename T>
    bool operator()(const T* const camera,
        const T* const point,
        T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion.
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position.
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(double observed_x, double observed_y) {
        ceres::CostFunction* p = new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(new SnavelyReprojectionError(observed_x, observed_y));
        return p;
    }

    double observed_x;
    double observed_y;
};


struct simple_qudratic
{
    double x;
    double observed_y;

    simple_qudratic(double x, double y) : x(x), observed_y(y) {}

    template <typename T>
    bool operator()(const T* const coefs,
        T* residuals) const
    {
        T predicted_y = (coefs[0] * x * x + coefs[1]);
        residuals[0] = predicted_y - observed_y;
        return true;
    }

    static ceres::CostFunction* Create(double x, double y)
    {
        return new ceres::AutoDiffCostFunction<simple_qudratic, 1, 2>(new simple_qudratic(x, y));
    }
};
*/