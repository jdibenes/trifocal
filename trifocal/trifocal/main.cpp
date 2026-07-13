
#include <iostream>
#include "loader.h"
#include "trifocal.h"

bool solver_tft_linear(float const* p3d_1, float const* p2d_2, float const* p2d_3, int N, float* r_12, float* t_12, float* r_23, float* t_23, float threshold)
{
    return solver_tft_linear<double>(p3d_1, p2d_2, p2d_3, N, r_12, t_12, r_23, t_23, static_cast<double>(threshold));
}

int main()
{
    Eigen::Matrix<float, 4, 4> pose0 = load_pose("C:/Users/jcds/Documents/GitHub/xvoldor/demo/data/hl2_5/pose/000062.bin").transpose();
    Eigen::Matrix<float, 4, 4> pose1 = load_pose("C:/Users/jcds/Documents/GitHub/xvoldor/demo/data/hl2_5/pose/000072.bin").transpose();
    Eigen::Matrix<float, 4, 4> pose2 = load_pose("C:/Users/jcds/Documents/GitHub/xvoldor/demo/data/hl2_5/pose/000082.bin").transpose();

    Eigen::Matrix<float, 4, 4> pose00h = pose0.inverse() * pose0;
    Eigen::Matrix<float, 4, 4> pose01h = pose0.inverse() * pose1;
    Eigen::Matrix<float, 4, 4> pose02h = pose0.inverse() * pose2;

    Eigen::Matrix<float, 3, 4> pose00 = pose00h(Eigen::seqN(0, 3), Eigen::indexing::all);
    Eigen::Matrix<float, 3, 4> pose01 = pose01h(Eigen::seqN(0, 3), Eigen::indexing::all);
    Eigen::Matrix<float, 3, 4> pose02 = pose02h(Eigen::seqN(0, 3), Eigen::indexing::all);

    Eigen::Matrix<float, 4, 4> pose12h = pose02h * pose01h.inverse();
    Eigen::Matrix<float, 3, 4> pose12  = pose12h(Eigen::seqN(0, 3), Eigen::indexing::all);

    Eigen::Matrix<float, 4, 7> p1h{
        {1,   2, -3, -1.5, 4, -5, 1.5},
        {2,  -1, -2,  1.2, 3,  4,  -6},
        {10, 12, 15,    7, 9, 16,  19},
        {1,   1,  1,    1, 1,  1,   1},
    };

    Eigen::Matrix<float, 3, 7> p1n = p1h.colwise().hnormalized();
    
    Eigen::Matrix<float, 3, 7> p11 = pose00 * p1h;
    Eigen::Matrix<float, 3, 7> p21 = pose01 * p1h;
    Eigen::Matrix<float, 3, 7> p31 = pose02 * p1h;

    Eigen::Matrix<float, 2, 7> x11 = p11.colwise().hnormalized();
    Eigen::Matrix<float, 2, 7> x21 = p21.colwise().hnormalized();
    Eigen::Matrix<float, 2, 7> x31 = p31.colwise().hnormalized();

    Eigen::Matrix<float, 3, 1> r1;
    Eigen::Matrix<float, 3, 1> t1;
    Eigen::Matrix<float, 3, 1> r2;
    Eigen::Matrix<float, 3, 1> t2;

    bool ok = solver_tft_linear(p11.data(), x21.data(), x31.data(), 7, r1.data(), t1.data(), r2.data(), t2.data(), 0.0f);

    Eigen::AngleAxis<float> r12(r1.norm(), r1.normalized());
    Eigen::AngleAxis<float> r23(r2.norm(), r2.normalized());

    Eigen::Matrix<float, 3, 4> P2;
    Eigen::Matrix<float, 3, 4> P3;

    P2 << r12.toRotationMatrix(), t1;
    P3 << r23.toRotationMatrix(), t2;

    std::cout << "POSES" << std::endl;
    std::cout << pose01 << std::endl;
    std::cout << P2 << std::endl;
    std::cout << pose12 << std::endl;
    std::cout << P3 << std::endl;

    Eigen::Matrix<float, 4, 4> P2f;
    Eigen::Matrix<float, 4, 4> P3f;

    P2f << P2, Eigen::Matrix<float, 1, 4>{0, 0, 0, 1};
    P3f << P3, Eigen::Matrix<float, 1, 4>{0, 0, 0, 1};

    Eigen::Matrix<float, 4, 4> e01 = P2f.inverse() * pose01h;
    Eigen::Matrix<float, 4, 4> e12 = P3f.inverse() * pose12h;

    std::cout << "ERRORS" << std::endl;
    std::cout << e01 << std::endl;
    std::cout << e12 << std::endl;

    Eigen::Matrix<float, 3, 3> er1 = e01(Eigen::seqN(0, 3), Eigen::seqN(0, 3));
    Eigen::Matrix<float, 3, 3> er2 = e12(Eigen::seqN(0, 3), Eigen::seqN(0, 3));

    Eigen::AngleAxis<float> ea1(er1);
    Eigen::AngleAxis<float> ea2(er2);

    std::cout << "rotation errors: "    << (ea1.angle() * (180.0 / 3.141592653589793238463)) << " | " << (ea2.angle() * (180.0 / 3.141592653589793238463)) << std::endl;
    std::cout << "translation errors: " << (e01.col(3).hnormalized().norm())                 << " | " << (e12.col(3).hnormalized().norm())                 << std::endl;

    return 0;
}
