import numpy as np


class LyapunovFSM:

    def __init__(self,
                 w_form=1.0,
                 w_goal=0.5,
                 w_vel=0.2,
                 eps_stable=0.05,
                 eps_dV=1e-4):

        self.w_form = w_form
        self.w_goal = w_goal
        self.w_vel = w_vel

        self.eps_stable = eps_stable
        self.eps_dV = eps_dV

        self.state = "TRACKING"

        self.V_prev = None
        self.bad_count = 0

    def compute_V(self, p_current, p_target, velocities, C_obs, goal):

        V_form = np.mean(np.linalg.norm(p_current - p_target, axis=1)**2)
        V_goal = np.linalg.norm(C_obs - goal)**2

        if velocities is None:
            V_vel = 0.0
        else:
            V_vel = np.mean(np.linalg.norm(velocities, axis=1)**2)

        V = (
            self.w_form * V_form +
            self.w_goal * V_goal +
            self.w_vel * V_vel
        )

        return V

    def update(self, p_current, p_target, velocities, C_obs, goal):

        V = self.compute_V(p_current, p_target, velocities, C_obs, goal)

        if self.V_prev is None:
            self.V_prev = V
            return self.state

        dV = V - self.V_prev

        # persistence to avoid noise-triggered switching
        if dV > self.eps_dV:
            self.bad_count += 1
        else:
            self.bad_count = 0

        if self.bad_count > 3:
            self.state = "REPLAN"

        elif V < self.eps_stable:
            self.state = "STABILIZE"

        else:
            self.state = "TRACKING"

        self.V_prev = V
        return self.state