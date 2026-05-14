#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>   
#include <pybind11/stl.h>
#include <vector>

namespace py = pybind11;

using Matrix = std::vector<std::vector<double>>;
using AdjList = std::vector<std::vector<int>>;

py::array_t<double> update_positions_cpp(
    py::array_t<double>,
    py::array_t<double>,
    py::array_t<double>,
    double,
    double,
    double,
    double
);


Matrix compute_targets_cpp(
    const std::vector<double>& center,
    int num_robots,
    bool is_ellipse,
    double R,
    double a,
    double b,
    double angle
);

PYBIND11_MODULE(formation_cpp, m) {
    m.def("update_positions", &update_positions_cpp);
    m.def("compute_targets", &compute_targets_cpp);
}