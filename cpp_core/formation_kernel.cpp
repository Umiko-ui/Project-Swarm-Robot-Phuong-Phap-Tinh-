#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include "Spatial_grid.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;



struct VelocityMemory
{
    double vx;
    double vy;
};

static std::vector<VelocityMemory> memory_velocities;
// POINT TO SEGMENT DISTANCE

double point_segment_distance(
    double px,
    double py,
    double x1,
    double y1,
    double x2,
    double y2,
    double& out_dx,
    double& out_dy
)
{
    double vx = x2 - x1;
    double vy = y2 - y1;

    double wx = px - x1;
    double wy = py - y1;

    double seg_len2 = vx*vx + vy*vy;

    if (seg_len2 < 1e-12)
    {
        out_dx = px - x1;
        out_dy = py - y1;

        return std::sqrt(out_dx*out_dx + out_dy*out_dy);
    }

    double t = (wx*vx + wy*vy) / seg_len2;

    t = std::clamp(t, 0.0, 1.0);

    double closest_x = x1 + t * vx;
    double closest_y = y1 + t * vy;

    out_dx = px - closest_x;
    out_dy = py - closest_y;

    return std::sqrt(out_dx*out_dx + out_dy*out_dy);
}

// MAIN FUNCTION

bool ccw(
    double ax, double ay,
    double bx, double by,
    double cx, double cy
)
{
    return (cy - ay)*(bx - ax)>(by - ay)*(cx - ax);
}

bool segments_intersect(
    double ax, double ay,
    double bx, double by,
    double cx, double cy,
    double dx, double dy
)
{
    return
        ccw(ax, ay, cx, cy, dx, dy)
        !=
        ccw(bx, by, cx, cy, dx, dy)

        &&

        ccw(ax, ay, bx, by, cx, cy)
        !=
        ccw(ax, ay, bx, by, dx, dy);
}



py::array_t<double> update_positions_cpp(
    py::array_t<double> p_current,
    py::array_t<double> p_target,
    py::array_t<double> Delta,

    // obstacle segments:
    // shape = (M,4)
    // [x1,y1,x2,y2]
    py::array_t<double> obstacle_segments,

    double kc,
    double kf,
    double Rs,
    double dt
)
{
    // BUFFER ACCESS

    auto buf_current = p_current.request();
    auto buf_target  = p_target.request();
    auto buf_delta   = Delta.request();
    auto buf_obs     = obstacle_segments.request();

    // VALIDATION

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

    if (buf_obs.ndim != 2 || buf_obs.shape[1] != 4)
    {
        throw std::runtime_error(
            "obstacle_segments must have shape (M,4)"
        );
    }

    int N = buf_current.shape[0];
    int M = buf_obs.shape[0];

    if (buf_target.shape[0] != N)
    {
        throw std::runtime_error(
            "p_target size mismatch"
        );
    }

    if (buf_delta.shape[0] != N ||
        buf_delta.shape[1] != N)
    {
        throw std::runtime_error(
            "Delta must have shape (N,N)"
        );
    }

    // POINTERS

    double* ptr_current = static_cast<double*>(buf_current.ptr);

    double* ptr_target = static_cast<double*>(buf_target.ptr);

    double* ptr_delta = static_cast<double*>(buf_delta.ptr);

    double* ptr_obs = static_cast<double*>(buf_obs.ptr);

    
    // ROBOT STORAGE
    std::vector<Robot> robots(N);

    for (int i = 0; i < N; i++)
    {
        robots[i].x = ptr_current[2*i];
        robots[i].y = ptr_current[2*i + 1];

        if (!std::isfinite(robots[i].x) ||
            !std::isfinite(robots[i].y))
        {
            throw std::runtime_error(
                "Non-finite robot position"
            );
        }
    }

    
    // MEMORY INIT

    if ((int)memory_velocities.size() != N)
    {
        memory_velocities.resize(N);

        for (int i = 0; i < N; i++)
        {
            memory_velocities[i].vx = 0.0;
            memory_velocities[i].vy = 0.0;
        }
    }

    // SPATIAL GRID

    double rc = 3.0 * Rs;

    SpatialGrid grid(rc);

    for (int i = 0; i < N; i++)
    {
        grid.insert(i, robots);
    }

    // FORCE STORAGE

    std::vector<std::vector<double>>
        F(N, std::vector<double>(2, 0.0));

    
    // FORCE COMPUTATION

    for (int i = 0; i < N; i++)
    {
        double xi = robots[i].x;
        double yi = robots[i].y;

        // TARGET FORCE

        double tx = ptr_target[2*i];
        double ty = ptr_target[2*i + 1];

        if (std::isfinite(tx) &&
            std::isfinite(ty))
        {
            F[i][0] += kc * (tx - xi);
            F[i][1] += kc * (ty - yi);
        }

        // OBSTACLE REPULSION

        double nearest_dx = 0.0;
        double nearest_dy = 0.0;

        double min_dist =
            std::numeric_limits<double>::infinity();

        for (int k = 0; k < M; k++)
        {
            double x1 = ptr_obs[4*k + 0];
            double y1 = ptr_obs[4*k + 1];
            double x2 = ptr_obs[4*k + 2];
            double y2 = ptr_obs[4*k + 3];

            double dx;
            double dy;

            double d = point_segment_distance(
                xi,
                yi,
                x1,
                y1,
                x2,
                y2,
                dx,
                dy
            );

            if (d < min_dist)
            {
                min_dist = d;

                nearest_dx = dx;
                nearest_dy = dy;
            }
        }

        
        // HARD REPULSION BARRIER

        double safe_distance = 1.5;

        if (min_dist < safe_distance)
        {
            double eps = 1e-6;

            double nx = nearest_dx / (min_dist + eps);

            double ny = nearest_dy / (min_dist + eps);

            // cubic explosion near wall
            double strength =
                40.0 *std::pow(
                    1.0 / (min_dist + eps),
                    3.0
                );

            // clamp to avoid numerical explosion
            strength = std::clamp(
                strength,
                0.0,
                200.0
            );

            F[i][0] += strength * nx;
            F[i][1] += strength * ny;
        }


        // LOCAL CELLS

        int cx = robots[i].cx;
        int cy = robots[i].cy;

        for (int dx_cell = -1;
             dx_cell <= 1;
             dx_cell++)
        {
            for (int dy_cell = -1;
                 dy_cell <= 1;
                 dy_cell++)
            {
                long long key =
                    grid.hash(
                        cx + dx_cell,
                        cy + dy_cell
                    );

                auto it =
                    grid.grid.find(key);

                if (it == grid.grid.end())
                    continue;

                const std::vector<int>& cell =
                    it->second;


                // NEIGHBOR LOOP
                for (int j : cell)
                {
                    if (i == j)
                        continue;

                    double dx = robots[j].x - xi;

                    double dy = robots[j].y - yi;

                    double dist2 = dx*dx + dy*dy;

                    if (!std::isfinite(dist2))
                        continue;

                    if (dist2 < 1e-12)
                        continue;

                    double dist = std::sqrt(dist2);

                    if (!std::isfinite(dist))
                        continue;

                    double dir_x = dx / dist;
                    double dir_y = dy / dist;

                    // DELTA_ij

                    double delta_ij = ptr_delta[i*N + j];

                    if (!std::isfinite(delta_ij))
                        continue;

                    if (delta_ij < 1e-6)
                        continue;

                    if (delta_ij > 1000.0)
                        continue;

                    // DESIRED DISTANCE

                    double desired = Rs * delta_ij;

                    desired = std::clamp(
                        desired,
                        0.05,
                        1000.0
                    );

                    // ERROR

                    double error = dist - desired;

                    double ratio = dist / desired;

                    ratio = std::clamp(
                        ratio,
                        -50.0,
                        50.0
                    );

                    double weight = std::exp(-ratio);

                    double f_linear = kf * error;

                    double f_exp = kf * error * weight;

                    double raw_force = 0.7 * f_linear + 0.3 * f_exp;

                    double f_total =20.0 *std::tanh(
                            raw_force / 20.0
                        );

                    F[i][0] += f_total * dir_x;

                    F[i][1] += f_total * dir_y;
                }
            }
        }
    }

    // DIAGNOSTICS

    double maxF = 0.0;
    double avgF = 0.0;

    for (int i = 0; i < N; i++)
    {
        double mag =
            std::sqrt(
                F[i][0]*F[i][0] +
                F[i][1]*F[i][1]
            );

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


    // FORCE SCALING

    double scale = 1.0;

    if (maxF > 50.0)
    {
        scale =
            50.0 / (maxF + 1e-8);
    }

    // OUTPUT

    auto result =
        py::array_t<double>({N, 2});

    auto buf_out = result.request();

    double* ptr_out = static_cast<double*>(buf_out.ptr);

  
    // SECOND ORDER DYNAMICS

    double damping = 0.92;



    int num_segments = buf_obs.shape[0];

    for (int i = 0; i < N; i++)
    {
        // VELOCITY UPDATE

        memory_velocities[i].vx = damping * memory_velocities[i].vx + scale * F[i][0] * dt;

        memory_velocities[i].vy = damping * memory_velocities[i].vy + scale * F[i][1] * dt;

        // SANITIZE VELOCITY

        if (!std::isfinite(memory_velocities[i].vx))
            memory_velocities[i].vx = 0.0;

        if (!std::isfinite(memory_velocities[i].vy))
            memory_velocities[i].vy = 0.0;

        // PREDICT NEW POSITION

        double old_x = robots[i].x;
        double old_y = robots[i].y;

        double new_x = old_x + memory_velocities[i].vx * dt;

        double new_y = old_y + memory_velocities[i].vy * dt;

        // COLLISION TEST

        bool collision = false;

        for (int s = 0; s < num_segments; s++)
        {
            double x1 = ptr_obs[4*s];
            double y1 = ptr_obs[4*s + 1];

            double x2 = ptr_obs[4*s + 2];
            double y2 = ptr_obs[4*s + 3];

            if (
                segments_intersect(
                    old_x,
                    old_y,
                    new_x,
                    new_y,
                    x1,
                    y1,
                    x2,
                    y2
                )
            )
            {
                collision = true;
                break;
            }
        }

        // COLLISION RESPONSE

        if (collision)
        {
            // keep only tangential motion
            memory_velocities[i].vx *= 0.3;
            memory_velocities[i].vy *= 0.3;

            new_x = old_x + memory_velocities[i].vx * dt;

            new_y = old_y + memory_velocities[i].vy * dt;
        }

    
        // WRITE OUTPUT

        ptr_out[2*i] = new_x;
        ptr_out[2*i + 1] = new_y;

        // FINAL SANITIZATION

        if (!std::isfinite(ptr_out[2*i]))
            ptr_out[2*i] = old_x;

        if (!std::isfinite(ptr_out[2*i + 1]))
            ptr_out[2*i + 1] = old_y;
    }

    return result;
}