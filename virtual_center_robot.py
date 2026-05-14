from RRTree_star import RRTree_star
from Result_log import Result_Log
from Robot_class import Robot
from Robot_sight_lib import scan_around
from Robot_paths_lib import *
from Robot_ranking import Ranker
from Robot_user_input import robot_user_input
from Tree import Node


class VirtualCenterRobot(Robot):

    def run_navigation_step(self,
                            current_state,
                            obstacles,
                            goal,
                            ranker,
                            RRT_star,
                            open_points_type,
                            picking_strategy,
                            result_log):

        
        # use external state instead of internal coordinate
        self.coordinate = tuple(current_state)

        # clean old data
        self.clear_local()

        # ---------------- SCAN ----------------
        closed_sights, open_sights = scan_around(self, obstacles, goal)

        # ---------------- GOAL CHECK ----------------
        self.check_goal(goal, closed_sights)

        # ---------------- LOCAL EXPLORATION ----------------
        if not self.saw_goal and not self.reach_goal:

            self.get_local_active_open_ranking_points(
                open_sights=open_sights,
                ranker=ranker,
                goal=goal,
                RRT_star=RRT_star,
                open_points_type=open_points_type
            )

            self.expand_global_open_ranking_points(
                self.local_active_open_rank_pts
            )

            self.visibility_graph.add_local_open_points(
                self.coordinate,
                self.local_active_open_pts
            )

        # ---------------- NEXT POINT SELECTION ----------------
        self.next_point = self.pick_next_point(
            goal,
            picking_strategy=picking_strategy
        )

        # ---------------- PATH GENERATION ----------------
        if self.next_point is not None:

            if tuple(self.next_point) == tuple(goal):
                skeleton_path = [self.coordinate, goal]
            else:
                skeleton_path = self.visibility_graph.BFS_skeleton_path(
                    self.coordinate,
                    tuple(self.next_point)
                )
        else:
            skeleton_path = []
            self.is_no_way_to_goal(True)

        # ---------------- PATH REFINEMENT ----------------
        asp, ls, l_time, a_time = approximately_shortest_path(
            skeleton_path,
            self.visited_sights,
            self.vision_range
        )

        # ---------------- LOGGING ----------------
        if len(skeleton_path) > 2:
            result_log.add_result([
                path_cost(asp),
                l_time + a_time
            ])

        self.expand_visited_path(asp)

        if not self.no_way_to_goal and self.next_point is not None:
            return np.array(self.next_point)

        return np.array(current_state)