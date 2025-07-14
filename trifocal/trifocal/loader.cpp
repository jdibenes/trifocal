
#include <exception>
#include <Eigen/Eigen>
#include "lock.h"

Eigen::Matrix<float, 4, 4> load_pose(char const* filename)
{
    FILE* f = fopen(filename, "rb");
    if (f == NULL) { throw std::runtime_error(""); }
    Cleaner file_close([=]() { fclose(f); });

    Eigen::Matrix<float, 4, 4> pose;
    uint32_t total = 4 * 4;
    uint32_t count = fread(pose.data(), sizeof(float), total, f);
    if (count != total) { throw std::runtime_error(""); }

    pose.transposeInPlace();

    return pose;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor> load_flow(char const* filename)
{
    FILE* f = fopen(filename, "rb");
    if (f == NULL) { throw std::runtime_error(""); }
    Cleaner file_close([=]() { fclose(f); });

    uint32_t header[3];
    size_t total_header = sizeof(header) / sizeof(uint32_t);
    size_t count_header = fread(header, sizeof(uint32_t), total_header, f);
    if (count_header != total_header) { throw std::runtime_error(""); }

    uint32_t width  = header[1] * 2;
    uint32_t height = header[2];
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor> flow(height, width);
    size_t total_data = width * height;
    size_t count_data = fread(flow.data(), sizeof(float), total_data, f);
    if (count_data != total_data) { throw std::runtime_error(""); }

    return flow;
}
