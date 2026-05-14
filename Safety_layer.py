import numpy as np

class SafetyLayer:

    def __init__(self, obstacles, safety_radius=0.5, gain=1.0):
        self.obstacles = obstacles
        self.R = safety_radius
        self.k = gain

    def repulsion(self, p):
        p = np.array(p)
        force = np.zeros(2)

        for obs in self.obstacles:
            o = np.array(obs)

            diff = p - o
            dist = np.linalg.norm(diff) + 1e-8

            if dist < self.R:
                force += self.k * diff / (dist**2)

        return force

    def project(self, p):
        """Hard safety projection (prevents wall penetration)."""
        p = np.array(p)

        for obs in self.obstacles:
            o = np.array(obs)
            diff = p - o
            dist = np.linalg.norm(diff)

            if dist < self.R:
                p = o + (diff / dist) * self.R

        return p