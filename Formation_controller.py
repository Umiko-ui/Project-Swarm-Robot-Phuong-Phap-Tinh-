import numpy as np
import formation_cpp


class FormationController:

    def __init__(self, kc=2.0, kf=0.2, Rs=2.0, dt=0.1):

        self.kc = kc
        self.kf = kf
        self.Rs = Rs
        self.dt = dt

    # =====================================================
    # SAFE NEIGHBOR CONSTRUCTION USING SPATIAL GRID
    # =====================================================

    def build_neighbors(self, p_current):

        N = len(p_current)

        Rs = self.Rs

        rc = 3.0 * Rs

        # grid cell size
        cell_size = rc

        grid = {}

        coords = np.asarray(p_current,dtype=np.float64)

        # -------------------------------------------------
        # ASSIGN TO GRID
        # -------------------------------------------------

        for i in range(N):

            cx = int(coords[i][0] // cell_size)
            cy = int(coords[i][1] // cell_size)

            key = (cx, cy)

            if key not in grid:
                grid[key] = []

            grid[key].append(i)

        neighbors = [[] for _ in range(N)]

        # -------------------------------------------------
        # SEARCH NEARBY CELLS
        # -------------------------------------------------

        for i in range(N):

            cx = int(coords[i][0] // cell_size)
            cy = int(coords[i][1] // cell_size)

            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    key = (cx + dx, cy + dy)
                    if key not in grid:
                        continue
                    for j in grid[key]:
                        if i == j:
                            continue
                        dist = np.linalg.norm(
                            coords[i] - coords[j]
                        )
                        if dist < rc:
                            neighbors[i].append(j)
        return neighbors

    # =====================================================
    # BUILD DELTA MATRIX
    # =====================================================

    def build_delta_matrix(self, p_current):

        p_current = np.asarray(p_current,dtype=np.float64)

        N = len(p_current)

        neighbors = self.build_neighbors(p_current)

        Delta = np.zeros((N, N),dtype=np.float64)

        # -------------------------------------------------
        # SYMMETRIC DISTANCE GRAPH
        # -------------------------------------------------

        for i in range(N):
            for j in neighbors[i]:
                dist = np.linalg.norm(p_current[i] - p_current[j])

                # avoid division problems
                dist = max(dist, 1e-6)

                # normalized desired distance
                # desired = Rs * Delta_ij
                delta_ij = dist / self.Rs

                Delta[i, j] = delta_ij
                Delta[j, i] = delta_ij

        # diagonal stays zero
        np.fill_diagonal(Delta, 0.0)

        return np.ascontiguousarray(Delta,dtype=np.float64)

    # =====================================================
    # MAIN UPDATE
    # =====================================================

    def update_positions(
        self,
        p_current,
        p_target,
        obstacles=None,
        Delta=None
    ):

        # -------------------------------------------------
        # INPUT SANITIZATION
        # -------------------------------------------------

        p_current_arr = np.ascontiguousarray(np.asarray(p_current,dtype=np.float64))

        p_target_arr = np.ascontiguousarray(np.asarray(p_target,dtype=np.float64))

        if p_current_arr.ndim != 2 \
           or p_current_arr.shape[1] != 2:
            raise ValueError(
                "p_current must have shape (N,2)"
            )

        if p_target_arr.ndim != 2 \
           or p_target_arr.shape[1] != 2:
            raise ValueError(
                "p_target must have shape (N,2)"
            )

        N = len(p_current_arr)

        if len(p_target_arr) != N:
            raise ValueError(
                "p_current and p_target size mismatch"
            )

        # -------------------------------------------------
        # DELTA MATRIX
        # -------------------------------------------------

        if Delta is None:

            Delta_arr = self.build_delta_matrix(p_current_arr)

        else:

            Delta_arr = np.ascontiguousarray(np.asarray(Delta,dtype=np.float64))

            if Delta_arr.shape != (N, N):

                raise ValueError(
                    "Delta must have shape ({N},{N})"
                )

        # -------------------------------------------------
        # SAFETY CHECKS
        # -------------------------------------------------

        if not np.all(np.isfinite(p_current_arr)):
            raise ValueError(
                "Non-finite values in p_current"
            )

        if not np.all(np.isfinite(p_target_arr)):
            raise ValueError(
                "Non-finite values in p_target"
            )

        if not np.all(np.isfinite(Delta_arr)):
            raise ValueError(
                "Non-finite values in Delta"
            )

        # -------------------------------------------------
        # CALL C++ CORE
        # -------------------------------------------------

        p_new = formation_cpp.update_positions(
            p_current_arr,
            p_target_arr,
            Delta_arr,
            float(self.kc),
            float(self.kf),
            float(self.Rs),
            float(self.dt)
        )

        p_new = np.asarray(p_new,dtype=np.float64)

        if not np.all(np.isfinite(p_new)):

            print("[WARNING] Non-finite output detected")

            # fallback to old positions
            return p_current_arr.copy()

        return p_new