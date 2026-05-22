import numpy as np
from virtual_center_robot import VirtualCenterRobot
from Follower_robots import FollowerRobot
from Formation_controller import FormationController
from Virtual_region_controller import VirtualRegionController
from Lyapunov_lib import LyapunovFSM 
from morphology_sensing import MorphologySensing


class SwarmControllerStable:

    def __init__(
        self,
        center_robot,
        follower_robots,
        formation_controller,
        virtual_region_controller,
        alpha=0.05,
        dt=0.1
    ):

        self.center = center_robot
        self.followers = follower_robots
        self.formation_controller = formation_controller
        self.virtual_region = virtual_region_controller

        self.sensing = MorphologySensing()

        self.C_meas = np.zeros(2)
        self.C_obs = np.zeros(2)
        self.C_plan = np.zeros(2)
        self.V_plan = np.zeros(2)

        self.alpha = alpha
        self.dt = dt
        self.v_max = 0.25

        self.buffer = []
        self.buffer_size = 3
        self.max_force = 0.0

    # -------------------------
    def compute_measurement(self):
        pos = np.array([r.coordinate for r in self.followers])
        self.C_meas = np.mean(pos, axis=0)

        self.buffer.append(self.C_meas.copy())
        if len(self.buffer) > self.buffer_size:
            self.buffer.pop(0)

    # -------------------------
    def update_observer(self):
        delayed = self.buffer[0] if self.buffer else self.C_meas
        self.C_obs = (1 - self.alpha) * self.C_obs + self.alpha * delayed

    # -------------------------
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

    # -------------------------
    def update_planner(self, target):

        error = target - self.C_plan
        acc = 2.0 * error - 2.0 * self.V_plan

        self.V_plan += acc * self.dt

        speed = np.linalg.norm(self.V_plan)
        if speed > self.v_max:
            self.V_plan = (self.V_plan / speed) * self.v_max

        self.C_plan += self.V_plan * self.dt

    # -------------------------
    def update_virtual_region(self, obstacles, goal):

        #free_width = self.sensing.free_width(self.C_obs, obstacles)
        robot_positions = [
            r.coordinate
            for r in self.followers
        ]

        clearances = [
            self.sensing.min_clearance(p, obstacles)
            for p in robot_positions
        ]

        min_clearance = min(clearances)

        free_width = 2.0 * min_clearance
        passage_angle = np.arctan2(
            goal[1] - self.C_obs[1],
            goal[0] - self.C_obs[0]
        )

        self.virtual_region.update(
            free_width=free_width,
            passage_angle=passage_angle,
            dt=self.dt,
            max_force=self.max_force
        )
        '''
        print("OBSTACLES TYPE:", type(obstacles))
        print("OBSTACLES ATTRS:", dir(obstacles))
        '''
        
    # -------------------------
    def update_followers(self):

        targets = self.virtual_region.get_robot_target_positions(
            self.C_plan,
            len(self.followers)
        )

        current = np.array([r.coordinate for r in self.followers])

        new_pos = self.formation_controller.update_positions(
            p_current=current,
            p_target=np.array(targets),
            obstacles=self.obstacles
        )
        vel = new_pos - current

        self.max_force = np.max(
            np.linalg.norm(vel, axis=1)
        )

        for r, p in zip(self.followers, new_pos):
            r.update_coordinate(tuple(p))

    # -------------------------
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

        self.obstacles = obstacles

        self.update_followers()
        self.update_planner(target)
        # morphology now clean & independent
        self.update_virtual_region(obstacles, goal)

        

        print("SHAPE:", self.virtual_region.current_shape)
        #print("BASE_RADIUS:", self.virtual_region.base_radius)
        #print("TARGET_RADIUS:", self.virtual_region.target_radius)


'''
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

                
            print("OPEN SIGHTS:", open_sights)

            self.virtual_region.update_structural_parameters(
                current_center=self.C_obs,   
                goal=goal,
                open_sights=open_sights,
                dt = self.dt
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

        self.update_virtual_region(goal)

        self.update_followers()

        
        print("SHAPE:", self.virtual_region.current_shape)
        print("PARAMS:", self.virtual_region.params)
        print("C_meas:", self.C_meas)
        print("C_obs :", self.C_obs)
        print("C_plan:", self.C_plan)
        print("BASE_RADIUS:", self.virtual_region.base_radius)
        print("TARGET_RADIUS:", self.virtual_region.target_radius)
    
'''