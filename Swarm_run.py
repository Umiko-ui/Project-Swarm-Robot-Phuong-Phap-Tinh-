import platform
from Plotter import Plotter
from RRTree_star import RRTree_star
from Result_log import Result_Log
from Robot_class import Robot, Robot_base
from Robot_paths_lib import *
from Robot_ranking import Ranker, Ranking_function
from Robot_sight_lib import *
from Robot_user_input import robot_user_input
from Tree import Node
from Swarm_formation1 import SwarmController1
from virtual_center_robot import VirtualCenterRobot
from Follower_robots import FollowerRobot
from Formation_controller import FormationController
from Virtual_region_controller import VirtualRegionController
from Swarm_Stabalize import SwarmControllerStable


def build_edges(swarm, R=2.5):
    edges = []
    robots = swarm.followers

    for i in range(len(robots)):
        p1 = robots[i].coordinate

        for j in range(i + 1, len(robots)):
            p2 = robots[j].coordinate

            dx = p1[0] - p2[0]
            dy = p1[1] - p2[1]

            if dx*dx + dy*dy < R*R:  # only nearby
                edges.append((p1, p2))

    return edges


# -------------------------------- MAIN FUNCTION 

def main(start=(0, 0), goal=(50, 50), map_name=None, num_iter=1,
         robot_vision=20, robot_type=Robot_base.RobotType.circle,
         robot_radius=0.5,
         open_points_type=Robot_base.Open_points_type.Open_Arcs,
         node_density=6,
         picking_strategy=Robot_base.Picking_strategy.neighbor_first,
         experiment=False, save_image=False, save_log=False,
         experiment_title=None):

    start = tuple(start)
    goal = tuple(goal)

    # ---------------- Ranking ----------------
    if open_points_type == Robot_base.Open_points_type.Open_Arcs:
        ranking_function = Ranking_function.Angular_similarity
    else:
        ranking_function = Ranking_function.RHS_RRT_base

    ranker = Ranker(alpha=0.9, beta=0.1, ranking_function=ranking_function)

    # ---------------- Plotter ----------------
    plotter = Plotter(title=f"Swarm Path Planning - {map_name}")

    # ---------------- Obstacles ----------------
    obstacles = Obstacles()
    if map_name is not None:
        obstacles.read(map_name=map_name)
        obstacles.line_segments()

    if not obstacles.valid_start_goal(start=start, goal=goal):
        print("Start hoặc Goal nằm trong vùng không hợp lệ!")
        return None

    # ---------------- RRT*  ----------------
    if open_points_type == Robot_base.Open_points_type.RRTstar:
        temp_robot = Robot(start=start, goal=goal, vision_range=robot_vision)
        boundary_area = temp_robot.find_working_space_boundaries(obstacles)
        sample_size = Robot.calculate_RRTnode_samplenumber(boundary=boundary_area, density=node_density)
        
        RRT_star = RRTree_star(root=Node(goal, cost=0), step_size=robot_vision,
                               radius=robot_vision, random_area=boundary_area, sample_size=sample_size)
        RRT_star.build(goal_coordinate=start, obstacles=obstacles, ignore_obstacles=True)
    else:
        RRT_star = None
   
    # ---------------- Result Log ----------------
    result_filename = Result_Log.prepare_name(
        start=start, goal=goal, pick=picking_strategy,
        range=robot_vision, open_points_type=open_points_type,
        map_name=map_name, experiment_title=experiment_title
    )
    result_log = Result_Log(header_csv=["asp_time", "asp_path_cost"])
    result_log.set_file_name(result_filename + ".csv")

    # ---------------- Swarm Setup ----------------
    center = VirtualCenterRobot(start=start, goal=goal, vision_range=robot_vision)
    followers = [
        FollowerRobot(center=center, radius=robot_radius, vision_range=robot_vision)
        for _ in range(4) 
    ]

    formation_controller = FormationController(kc=0.5, kf=2.5, Rs=2.0, dt=0.1)
    virtual_region_controller = VirtualRegionController(initial_radius=2.0)


    ''' OLD MODEL 
    swarm = SwarmController1(
        center_robot=center,
        follower_robots=followers,
        formation_controller=formation_controller,
        virtual_region_controller=virtual_region_controller
    )
    ''' 
    # DEFINE OBJECT SWARM 
    swarm = SwarmControllerStable(
        center_robot=center,
        follower_robots=followers,
        formation_controller=formation_controller,
        virtual_region_controller=virtual_region_controller
    )

    plotter.animation(
        Robot=swarm.center,
        iter_count=0,
        obstacles=obstacles,
        
        experiment=experiment
    )

    # 🔥 initialize scatter ONCE
    plotter.init_scene(obstacles, swarm)

    edges = build_edges(swarm)
    plotter.update_edges(edges)

    # ---------------- Main Loop ----------------
    iter_count = 0

    while True:
        iter_count += 1
        print(f"--- Iteration {iter_count} ---")
       
       
        swarm.update(
            obstacles=obstacles,
            goal=goal,
            ranker=ranker,
            RRT_star=RRT_star,
            open_points_type=open_points_type,
            picking_strategy=picking_strategy,
            result_log=result_log
        )

    # ---------------- DRAW REGION ----------------
        shape = swarm.virtual_region.current_shape
        params = swarm.virtual_region.params
        center = swarm.center.coordinate

        plotter.update_virtual_region(center, shape, params)
    # ---------------- DEBUG ----------------
        followers_pos = np.array([r.coordinate for r in swarm.followers])
        true_center = np.mean(followers_pos, axis=0)

        print("CENTER (stored):", swarm.center.coordinate)
        print("CENTER (true):  ", true_center)

        if hasattr(swarm, "_prev_positions"):
            prev = swarm._prev_positions
            curr = followers_pos
            max_step = np.max(np.linalg.norm(curr - prev, axis=1))
            print("MAX STEP:", max_step)

        swarm._prev_positions = followers_pos.copy()
        
    # ---------------- PLOT ----------------
        plotter.update_swarm(swarm)

        edges = build_edges(swarm)
        plotter.update_edges(edges)

        plotter.plt.pause(0.03)

        if (num_iter > 0 and iter_count >= num_iter) or swarm.center.finish():
            print("Done.")
            break

    plotter.show()

    return swarm

if __name__ == '__main__':
    menu_result = robot_user_input()
    open_pts_type = menu_result.open_pts_type

    if 'o' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.Open_Arcs
    elif 'r' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.RRTstar

    picking_strategy = menu_result.p
    if 'g' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.global_first
    elif 'n' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.neighbor_first

    main(
        start=menu_result.s,
        goal=menu_result.g,
        map_name=menu_result.m,
        num_iter=menu_result.n,
        robot_vision=menu_result.r,
        node_density=menu_result.d,
        open_points_type=open_pts_type,
        picking_strategy=picking_strategy,
        experiment=False,
        save_image=True,
        save_log=True
    )

    