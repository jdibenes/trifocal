
#pragma once

bool
trifocal_R_t_linear
(
    float const* p2d_1,
    float const* p2d_2,
    float const* p2d_3,
    float const* p2d_s,
    float const* p3d_s,
    int N,
    bool use_prior,
    float* r1,
    float* t1,
    float* r2,
    float* t2
);

bool
trifocal_R_t_Ressl
(
    float const* p2d_1,
    float const* p2d_2,
    float const* p2d_3,
    float const* p2d_s,
    float const* p3d_s,
    int N,
    bool use_prior,
    float* r1,
    float* t1,
    float* r2,
    float* t2
);
