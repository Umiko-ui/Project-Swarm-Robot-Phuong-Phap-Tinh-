import numpy as np
import formation_cpp


class VirtualRegionType:

    CIRCLE = "CIRCLE"
    ELLIPSE = "ELLIPSE"


class VirtualRegionController:

    def __init__(
        self,
        initial_radius,
        safety_margin=0.2
    ):

        self.base_radius = initial_radius

        self.safety_margin = safety_margin

        self.current_shape = (
            VirtualRegionType.CIRCLE
        )

        self.params = {

            "R": initial_radius,
            "a": initial_radius,
            "b": initial_radius,
            "angle": 0.0
        }

    # ---------------------------------------------------------
    # MAIN UPDATE
    # ---------------------------------------------------------

    def update(
        self,
        free_width,
        passage_angle,
        dt,
        max_force=0.0
    ):

        effective_width = (
            free_width - self.safety_margin
        )

        diameter = 2.0 * self.base_radius

        # -----------------------------------------------------
        # JAM DETECTION
        # -----------------------------------------------------

        jammed = max_force > 0.06

        # hysteresis thresholds
        ellipse_enter = diameter

        ellipse_exit = 1.3 * diameter

        # -----------------------------------------------------
        # STATE TRANSITION
        # -----------------------------------------------------

        if self.current_shape == (
            VirtualRegionType.CIRCLE
        ):

            if (
                effective_width < ellipse_enter
                or jammed
            ):

                self.current_shape = (
                    VirtualRegionType.ELLIPSE
                )

        else:

            if (
                effective_width > ellipse_exit
                and not jammed
            ):

                self.current_shape = (
                    VirtualRegionType.CIRCLE
                )

        # -----------------------------------------------------
        # PARAMETER UPDATE
        # -----------------------------------------------------

        if self.current_shape == (
            VirtualRegionType.CIRCLE
        ):

            self.params = {

                "R": self.base_radius,
                "a": self.base_radius,
                "b": self.base_radius,
                "angle": 0.0
            }

        else:

            b = max(
                effective_width / 2.0,
                0.1
            )

            # area-preserving deformation
            raw_a = (
                self.base_radius ** 2
            ) / b

            MAX_A = (
                self.base_radius * 4.0
            )

            a = min(raw_a, MAX_A)

            self.params = {

                "R": None,
                "a": a,
                "b": b,
                "angle": passage_angle
            }

    # ---------------------------------------------------------
    # TARGET GENERATION
    # ---------------------------------------------------------

    def get_robot_target_positions(
        self,
        current_center,
        num_robots
    ):

        is_ellipse = (

            self.current_shape
            == VirtualRegionType.ELLIPSE
        )

        R = (
            self.params.get("R", 0.0)
            or 0.0
        )

        a = self.params["a"]
        b = self.params["b"]
        angle = self.params["angle"]

        targets = formation_cpp.compute_targets(

            list(current_center),

            int(num_robots),

            bool(is_ellipse),

            float(R),

            float(a),

            float(b),

            float(angle)
        )

        return targets