#include <vector>
#include <cmath>
#include <cassert>




using Matrix = std::vector<std::vector<double>>;

Matrix compute_targets_cpp(
    const std::vector<double>& center,
    int num_robots,
    bool is_ellipse,
    double R,
    double a,
    double b,
    double angle
) {
    assert(center.size() == 2);
    assert(num_robots > 0);

    Matrix targets;
    targets.reserve(num_robots);

    const double PI = 3.141592653589793;

    double cx = center[0];
    double cy = center[1];

    if (is_ellipse) {

        // =====================================================
        // TRUE ELLIPSE DISTRIBUTION
        // (x, y) = (a cosθ, b sinθ), then rotate
        // =====================================================

        double cosA = std::cos(angle);
        double sinA = std::sin(angle);

        for (int i = 0; i < num_robots; ++i) {

            double theta = 2.0 * PI * i / num_robots;

            // Local ellipse coordinates
            double x_local = a * std::cos(theta);
            double y_local = b * std::sin(theta);

            // Rotate + translate
            double tx = cx + x_local * cosA - y_local * sinA;
            double ty = cy + x_local * sinA + y_local * cosA;

            targets.push_back({tx, ty});
        }

    } else {

        // =====================================================
        // CIRCLE DISTRIBUTION
        // =====================================================

        double angle_step = 2.0 * PI / num_robots;
        double r = R * 0.8;

        for (int i = 0; i < num_robots; ++i) {

            double theta = i * angle_step;

            double tx = cx + r * std::cos(theta);
            double ty = cy + r * std::sin(theta);

            targets.push_back({tx, ty});
        }
    }

    return targets;
}




