import numpy as np
from Robot_class import Robot

class FollowerRobot(Robot):
    def __init__(self, center, radius, vision_range):
        
        angle = np.random.uniform(0, 2*np.pi)
        r = 0.5  # small offset

        start = (
            center.coordinate[0] + r*np.cos(angle),
            center.coordinate[1] + r*np.sin(angle)
        )

        goal = center.goal

        super().__init__(
            start=start,
            goal=goal,
            vision_range=vision_range, 
            robot_radius=radius
        )

        self.center = center

