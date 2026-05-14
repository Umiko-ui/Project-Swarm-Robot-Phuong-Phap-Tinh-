import numpy as np
from virtual_center_robot import VirtualCenterRobot
from Follower_robots import FollowerRobot
from Formation_controller import FormationController
from Virtual_region_controller import VirtualRegionController
from Lyapunov_lib import LyapunovFSM 


#SEPERATE DIFFERENT STATES 
class SwarmControllerStable:

    def __init__(self,
                 center_robot,
                 follower_robots,
                 formation_controller,
                 virtual_region_controller,
                 alpha=0.05,
                 dt=0.1):


        # MAKE THE OBJECTS 
        self.center = center_robot
        self.followers = follower_robots
        self.formation_controller = formation_controller
        self.virtual_region = virtual_region_controller



        # MAKE DIFFERENT STATES
        self.C_meas = np.zeros(2)
        self.C_obs = np.zeros(2)
        self.C_plan = np.zeros(2)
        self.V_plan = np.zeros(2)

        # PARAMETERS
        self.alpha = alpha
        self.dt = dt
        self.v_max = 0.25

        # DELAY BUFFER
        self.buffer = []
        self.buffer_size = 3


    # MEASUREMENT
    def compute_measurement(self):
        pos = np.array([r.coordinate for r in self.followers])
        self.C_meas = np.mean(pos, axis=0)

        self.buffer.append(self.C_meas.copy())
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

    # OBSERVER
    def update_observer(self):
        delayed = self.buffer[0] if len(self.buffer) > 0 else self.C_meas

        self.C_obs = (1 - self.alpha) * self.C_obs + self.alpha * delayed

    
    # PLANNER USE TEACHER CODE
    def compute_target(self, obstacles, goal, ranker,
                       RRT_star, open_points_type,
                       picking_strategy, result_log):

        return self.center.run_navigation_step(
            current_state=self.C_obs,
            obstacles=obstacles,
            goal=goal,
            ranker=ranker,
            RRT_star=RRT_star,
            open_points_type=open_points_type,
            picking_strategy=picking_strategy,
            result_log=result_log
        )

    
    # SMOOTH DYNAMICS
    def update_planner(self, target):

        error = target - self.C_plan

        acc = 2.0 * error - 2.0 * self.V_plan

        self.V_plan += acc * self.dt

        speed = np.linalg.norm(self.V_plan)
        if speed > self.v_max:
            self.V_plan = (self.V_plan / speed) * self.v_max

        self.C_plan += self.V_plan * self.dt
    # UPDATE CIRCLE/ELLIPSE BASED ON ENVIRONMENT

    def update_virtual_region(self, goal):

            # get sensing data from center robot
            open_sights = self.center.visited_sights.get_open_sights(
                self.center.coordinate
            )
            self.virtual_region.update_structural_parameters(
                current_center=self.C_obs,   
                goal=goal,
                open_sights=open_sights
            )

    # FOLLOWERS
    # CALL FORMATION_CONTROLLER.UPDATE_POSITIONS
    def update_followers(self):

        targets = self.virtual_region.get_robot_target_positions(
            current_center=self.C_plan,
            num_robots=len(self.followers)
        )

        current = np.array([r.coordinate for r in self.followers])

        new_pos = self.formation_controller.update_positions(
            p_current=current,
            p_target=np.array(targets)
        )

        for r, p in zip(self.followers, new_pos):
            r.update_coordinate(tuple(p))

    
    # MAIN LOOP
    def update(self, obstacles, goal, ranker,
               RRT_star, open_points_type,
               picking_strategy, result_log):

        self.compute_measurement()
        self.update_observer()

        target = self.compute_target(
            obstacles, goal, ranker,
            RRT_star, open_points_type,
            picking_strategy, result_log
        )

        self.update_planner(target)
        self.update_followers()

        
        print("SHAPE:", self.virtual_region.current_shape)
        print("PARAMS:", self.virtual_region.params)
        print("C_meas:", self.C_meas)
        print("C_obs :", self.C_obs)
        print("C_plan:", self.C_plan)

    
