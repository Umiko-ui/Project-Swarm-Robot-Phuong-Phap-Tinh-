#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "Spatial_grid.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>   

namespace py = pybind11;


struct VelocityMemory {
    double vx;
    double vy;
};
// use static because want this to remain after everyrun
static std::vector<VelocityMemory> memory_velocities;

// =====================================================
// MAIN FUNCTION 
// =====================================================

py::array_t<double> update_positions_cpp(
    py::array_t<double> p_current,
    py::array_t<double> p_target,
    py::array_t<double> Delta,
    double kc,
    double kf,
    double Rs,
    double dt
)
{
    // =================================================
    // BUFFER ACCESS
    // the request() function is exstracting the raw buffer information from python
    // which mean it contains pointer to data, dimension, shape, type info
    // =================================================
    auto buf_current = p_current.request();
    auto buf_target  = p_target.request();
    auto buf_delta   = Delta.request();
    // =================================================
    // VALIDATION
    // =================================================
    if (buf_current.ndim != 2 || buf_current.shape[1] != 2)
    {
        throw std::runtime_error(
            "p_current must have shape (N,2)"
        );
    }

    if (buf_target.ndim != 2 || buf_target.shape[1] != 2)
    {
        throw std::runtime_error(
            "p_target must have shape (N,2)"
        );
    }

    if (buf_delta.ndim != 2)
    {
        throw std::runtime_error(
            "Delta must be 2D"
        );
    }

    int N = buf_current.shape[0];

    if (buf_target.shape[0] != N)
    {
        throw std::runtime_error(
            "p_target size mismatch"
        );
    }

    if (buf_delta.shape[0] != N || buf_delta.shape[1] != N)
    {
        throw std::runtime_error(
            "Delta must have shape (N,N)"
        );
    }

    // =================================================
    // POINTERS
    // use because the raw pointer access of the address of the first element
    // if use vector then will have to copy numpy to vector then convert back
    // =================================================

    double* ptr_current = static_cast<double*>(buf_current.ptr);
    double* ptr_target = static_cast<double*>(buf_target.ptr);
    double* ptr_delta = static_cast<double*>(buf_delta.ptr);

    // =================================================
    // ROBOT STORAGE
    // =================================================

    std::vector<Robot> robots(N);

    for (int i = 0; i < N; i++) {

        robots[i].x = ptr_current[2*i];
        robots[i].y = ptr_current[2*i + 1];

        if (!std::isfinite(robots[i].x) || !std::isfinite(robots[i].y))
        {
            throw std::runtime_error(
                "Non-finite robot position"
            );
        }
    }

    // =================================================
    // MEMORY INIT
    // =================================================

    if ((int)memory_velocities.size() != N) {

        memory_velocities.resize(N);

        for (int i = 0; i < N; i++) {

            memory_velocities[i].vx = 0.0;
            memory_velocities[i].vy = 0.0;
        }
    }

    // =================================================
    // SPATIAL GRID
    // =================================================

    double rc = 3.0 * Rs;

    SpatialGrid grid(rc);

    for (int i = 0; i < N; i++) {
        grid.insert(i, robots);
    }

    // =================================================
    // FORCE STORAGE
    // =================================================

    std::vector<std::vector<double>> F(N,std::vector<double>(2, 0.0));

    // =================================================
    // FORCE COMPUTATION
    // =================================================

    for (int i = 0; i < N; i++) {
        double xi = robots[i].x;
        double yi = robots[i].y;
        // =============================================
        // TARGET FORCE
        // =============================================
        double tx = ptr_target[2*i];
        double ty = ptr_target[2*i + 1];
        if (std::isfinite(tx) && std::isfinite(ty))
        {
            F[i][0] += kc * (tx - xi);
            F[i][1] += kc * (ty - yi);
        }

        // =============================================
        // LOCAL CELLS
        // =============================================

        int cx = robots[i].cx;
        int cy = robots[i].cy;

        for (int dx_cell = -1;dx_cell <= 1;dx_cell++)
        {
            for (int dy_cell = -1;dy_cell <= 1;dy_cell++)
            {
                long long key = grid.hash(cx + dx_cell,cy + dy_cell);

                auto it = grid.grid.find(key);

                if (it == grid.grid.end()) continue;
                const std::vector<int>& cell = it->second;

                // =====================================
                // NEIGHBOR LOOP
                // =====================================
                for (int j : cell) {
                    if (i == j) continue;
                    double dx = robots[j].x - xi;
                    double dy = robots[j].y - yi;
                    double dist2 = dx*dx + dy*dy;

                    if (!std::isfinite(dist2)) continue;

                    if (dist2 < 1e-12) continue;

                    double dist = std::sqrt(dist2);

                    if (!std::isfinite(dist)) continue;

                    double dir_x = dx / dist;

                    double dir_y = dy / dist;

                    // =================================
                    // DELTA_ij
                    // =================================

                    double delta_ij = ptr_delta[i*N + j];

                    if (!std::isfinite(delta_ij)) continue;

                    if (delta_ij < 1e-6) continue;

                    if (delta_ij > 1000.0) continue;

                    // =================================
                    // DESIRED DISTANCE
                    // =================================

                    double desired = Rs * delta_ij;

                    if (!std::isfinite(desired)) continue;

                    desired = std::clamp(desired,0.05,1000.0);
    
    // if desired go beyond this range, it will assign an approriate number
    // to avoid dividing with 0
                    // =================================
                    // DISTANCE ERROR
                    // =================================

                    double error = dist - desired;

                    if (!std::isfinite(error))
                        continue;

                    // =================================
                    // WEIGHT
                    // =================================

                    double ratio = dist / desired;

                    ratio = std::clamp(ratio,-50.0,50.0);

                    double weight = std::exp(-ratio);

                    if (!std::isfinite(weight)) continue;

                    // =================================
                    // FORCE MODEL
                    // =================================

                    double f_linear = kf * error;

                    double f_exp = kf * error * weight;

                    double raw_force = 0.7 * f_linear + 0.3 * f_exp;

                    // =================================
                    // SATURATION
                    // =================================

                    double f_total =20.0 * std::tanh(raw_force / 20.0);

                    if (!std::isfinite(f_total)) continue;

                    // =================================
                    // APPLY FORCE
                    // =================================
                    F[i][0] += f_total * dir_x;
                    F[i][1] += f_total * dir_y;
                }
            }
        }
    }

    // =================================================
    // DIAGNOSTICS
    // =================================================

    double maxF = 0.0;
    double avgF = 0.0;

    for (int i = 0; i < N; i++) {

        if (!std::isfinite(F[i][0]) ||
            !std::isfinite(F[i][1]))
        {
            F[i][0] = 0.0;
            F[i][1] = 0.0;
        }

        double mag = std::sqrt(F[i][0]*F[i][0] + F[i][1]*F[i][1]);

        if (!std::isfinite(mag)) mag = 0.0;

        maxF = std::max(maxF, mag);

        avgF += mag;
    }

    avgF /= (double)N;

    std::cout
        << "MAX FORCE: "
        << maxF
        << " | AVG FORCE: "
        << avgF
        << std::endl;

    // =================================================
    // FORCE SCALING
    // =================================================

    double scale = 1.0;

    if (!std::isfinite(maxF)) {
        std::cout << "FORCE EXPLOSION DETECTED" << std::endl;
        scale = 0.0;
    }
    else if (maxF > 10.0) {
        scale = 10.0 / (maxF + 1e-8);
    }

    // =================================================
    // OUTPUT
    // =================================================

    auto result = py::array_t<double>({N, 2});

    auto buf_out = result.request();

    double* ptr_out = static_cast<double*>(buf_out.ptr);

    // =================================================
    // SECOND ORDER DYNAMICS
    // =================================================

    double damping = 0.92;

    for (int i = 0; i < N; i++) {

        memory_velocities[i].vx = damping *memory_velocities[i].vx + scale *F[i][0] *dt;

        memory_velocities[i].vy = damping * memory_velocities[i].vy + scale * F[i][1] *dt;

        // velocity sanitization
        if (!std::isfinite(memory_velocities[i].vx))
            memory_velocities[i].vx = 0.0;

        if (!std::isfinite(memory_velocities[i].vy))
            memory_velocities[i].vy = 0.0;

        ptr_out[2*i] = robots[i].x + memory_velocities[i].vx * dt;

        ptr_out[2*i + 1] = robots[i].y + memory_velocities[i].vy * dt;

        // output sanitization
        if (!std::isfinite(ptr_out[2*i]))
            ptr_out[2*i] = robots[i].x;

        if (!std::isfinite(ptr_out[2*i + 1]))
            ptr_out[2*i + 1] = robots[i].y;
    }

    return result;
}

