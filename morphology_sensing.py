import math
import numpy as np
from sklearn.cluster import KMeans


class MorphologySensing:

    def __init__(self):
        pass

    # EXTRACT LINE SEGMENTS
    def extract_segments(self, obstacles):

        if obstacles is None:
            return []

        if isinstance(obstacles, list):
            return obstacles

        if isinstance(obstacles, np.ndarray):

            if obstacles.size == 0:
                return []

            return obstacles.tolist()

        # teacher obstacle object
        if hasattr(obstacles, "line_segments"):

            nested = obstacles.line_segments()

            flat = []

            for obs in nested:
                flat.extend(obs)

            return flat

        if hasattr(obstacles, "obstacles_line_segments"):

            nested = obstacles.obstacles_line_segments

            flat = []

            for obs in nested:
                flat.extend(obs)

            return flat

        return []

    # MINIMUM CLEARANCE
    def min_clearance(self, position, obstacles):

        pos = np.array(position, dtype=float)

        segments = self.extract_segments(obstacles)

        if len(segments) == 0:
            return 10.0

        min_dist = float("inf")

        for seg in segments:

            try:

                p1, p2 = seg

                p1 = np.array(p1, dtype=float)
                p2 = np.array(p2, dtype=float)

                midpoint = 0.5 * (p1 + p2)

                d = np.linalg.norm(pos - midpoint)

                min_dist = min(min_dist, d)

            except:
                continue

        return max(min_dist, 0.1)

    # PASSAGE ESTIMATION

    def estimate_passage(
        self,
        current_center,
        goal,
        open_sights,
        obstacles,
        robot_positions
    ):

        # DEFAULT GOAL DIRECTION

        dx_goal = goal[0] - current_center[0]
        dy_goal = goal[1] - current_center[1]

        passage_angle = math.atan2(
            dy_goal,
            dx_goal
        )

        # METRIC CLEARANCE

        clearances = [

            self.min_clearance(p, obstacles)

            for p in robot_positions
        ]

        min_clearance = min(clearances)

        free_width = 2.0 * min_clearance

        # OPEN-SIGHT GEOMETRIC INTERPRETATION

        if open_sights is not None and len(open_sights) > 0:

            obs_pts = []

            for sight in open_sights:

                if len(sight) >= 2:

                    obs_pts.append(sight[0])
                    obs_pts.append(sight[1])

            obs_pts = np.array(obs_pts)

            # KMEANS GAP ESTIMATION

            if len(obs_pts) >= 2:

                try:

                    kmeans = KMeans(
                        n_clusters=2,
                        random_state=0,
                        n_init=10
                    ).fit(obs_pts)

                    r_o1, r_o2 = (kmeans.cluster_centers_)

                    # corridor orientation
                    dx_obs = r_o2[0] - r_o1[0]
                    dy_obs = r_o2[1] - r_o1[1]

                    angle_gap1 = math.atan2(dx_obs,-dy_obs)

                    angle_gap2 = math.atan2(-dx_obs,dy_obs)

                    diff1 = abs(
                        math.atan2(math.sin(angle_gap1- passage_angle),
                            math.cos(angle_gap1- passage_angle))
                    )

                    diff2 = abs(
                        math.atan2(math.sin(angle_gap2 - passage_angle),
                            math.cos(angle_gap2 - passage_angle))
                    )

                    passage_angle = (
                        angle_gap1
                        if diff1 < diff2
                        else angle_gap2
                    )

                except:
                    pass

        return free_width, passage_angle