
#pragma once

#include <Eigen/Eigen>

Eigen::Matrix<float, 4, 4> load_pose(char const* filename);
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign | Eigen::RowMajor> load_flow(char const* filename);
