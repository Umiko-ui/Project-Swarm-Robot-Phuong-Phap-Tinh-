from virtual_center_robot import VirtualCenterRobot
from Follower_robots import FollowerRobot
from Formation_controller import FormationController
from Virtual_region_controller import VirtualRegionController
import numpy as np

# WRONG MODEL 
class SwarmController1:
    def __init__(self, center_robot, follower_robots, formation_controller, virtual_region_controller):
        self.center = center_robot
        self.followers = follower_robots
        self.formation_controller = formation_controller
        self.virtual_region = virtual_region_controller
    


    def update(self, obstacles, goal, ranker, RRT_star,
            open_points_type, picking_strategy, result_log):

        # ------------------------------------------------
        # 0️⃣ SYNC CENTER FROM PREVIOUS STEP
        # ------------------------------------------------
        followers_pos = np.array([r.coordinate for r in self.followers])
        # ----- create a center from the follower_pos by taking the mean 
        alpha = 0.2

        measured = np.mean(followers_pos, axis=0)

        
        # ------------------------------------------------
        # 1️⃣ CENTER NAVIGATION
        # ------------------------------------------------
        
        self.center.run_navigation_step(
            obstacles=obstacles,
            goal=goal,
            ranker=ranker,
            RRT_star=RRT_star,
            open_points_type=open_points_type,
            picking_strategy=picking_strategy,
            result_log=result_log
        )
        self.center.coordinate = (
            (1 - alpha) * np.array(self.center.coordinate)
            + alpha * measured
        )
        print("CENTER (synced):", self.center.coordinate)

        # ------------------------------------------------
        # 2️⃣ PERCEPTION GET FROM THE CENTER ROBOT
        # ------------------------------------------------
        current_open_sights = self.center.visited_sights.get_open_sights(
            self.center.coordinate
        )

        # ------------------------------------------------
        # 3️⃣ REGION UPDATE
        # ------------------------------------------------
        shape, params = self.virtual_region.update_structural_parameters(
            current_center=self.center.coordinate,
            goal=goal,
            open_sights=current_open_sights
        )

        # ------------------------------------------------
        # 4️⃣ TARGET GENERATION
        # ------------------------------------------------
        targets = self.virtual_region.get_robot_target_positions(
            current_center=self.center.coordinate,
            num_robots=len(self.followers)
        )

        print("TARGETS:", targets)

        # ------------------------------------------------
        # 5️⃣ FORMATION DYNAMICS
        # ------------------------------------------------
        p_current = np.array([robot.coordinate for robot in self.followers])
        p_target = np.array(targets)

        p_new = self.formation_controller.update_positions(
            p_current=p_current,
            p_target=p_target
        )

        for robot, new_pos in zip(self.followers, p_new):
            robot.update_coordinate(tuple(new_pos))