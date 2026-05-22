#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <vector>

namespace py = pybind11;

using Matrix = std::vector<std::vector<double>>;
using AdjList = std::vector<std::vector<int>>;

// =====================================================
// UPDATE POSITIONS DECLARATION
// =====================================================

py::array_t<double> update_positions_cpp(
    py::array_t<double> p_current,
    py::array_t<double> p_target,
    py::array_t<double> Delta,

    // NEW ARGUMENT
    py::array_t<double> obstacle_segments,

    double kc,
    double kf,
    double Rs,
    double dt
);

// =====================================================
// TARGET COMPUTATION
// =====================================================

Matrix compute_targets_cpp(
    const std::vector<double>& center,
    int num_robots,
    bool is_ellipse,
    double R,
    double a,
    double b,
    double angle
);

// =====================================================
// PYBIND MODULE
// =====================================================

PYBIND11_MODULE(formation_cpp, m)
{
    m.def(
        "update_positions_cpp",
        &update_positions_cpp,

        py::arg("p_current"),
        py::arg("p_target"),
        py::arg("Delta"),
        py::arg("obstacle_segments"),
        py::arg("kc"),
        py::arg("kf"),
        py::arg("Rs"),
        py::arg("dt")
    );

    m.def(
        "compute_targets",
        &compute_targets_cpp
    );
}