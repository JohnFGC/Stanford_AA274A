#!/usr/bin/env python3

import numpy as np
import rclpy
from P1_astar import AStar, DetOccupancyGrid2D
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from scipy.interpolate import splev, splrep

class Navigator(BaseNavigator):
    def __init__(self, kpx = 4.5, kpy = 4.5, kdx = 2.5, kdy = 2.5) -> None:
        super().__init__()
        self.kp = 2.0
        
        self.V_PREV_THRES = 0.0001
        self.t_prev = 0.0
        self.V_prev = 0.0
        self.om_prev = 0.0
        
        self.kpx = kpx
        self.kpy = kpy
        self.kdx = kdx
        self.kdy = kdy
        
    def compute_heading_control(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        err = wrap_angle(goal.theta - state.theta)
        msg = TurtleBotControl()
        msg.omega = self.kp * err
        return msg 
    
    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        """
        Inputs:
            state: Current state
            t: Current time
        Outputs:
            V, om: Control actions
        """
        x, y, th = state.x, state.y, state.theta
        x_d, y_d = plan.desired_state(t).x, plan.desired_state(t).y
        xd_d, yd_d = splev(t, plan.path_x_spline, der = 1), splev(t, plan.path_y_spline, der = 1)
        xdd_d, ydd_d = splev(t, plan.path_x_spline, der = 2), splev(t, plan.path_y_spline, der = 2)
        #x_d, xd_d, xdd_d, y_d, yd_d, ydd_d = self.get_desired_state(t)
        
        dt = t - self.t_prev

        # avoid singularity
        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        xd = self.V_prev*np.cos(th)
        yd = self.V_prev*np.sin(th)

        # compute virtual controls
        u = np.array([xdd_d + self.kpx*(x_d-x) + self.kdx*(xd_d-xd),
                      ydd_d + self.kpy*(y_d-y) + self.kdy*(yd_d-yd)])

        # compute real controls
        J = np.array([[np.cos(th), -self.V_prev*np.sin(th)],
                          [np.sin(th), self.V_prev*np.cos(th)]])
        
        a, om = np.linalg.solve(J, u)
        V = self.V_prev + a*dt

        # # apply control limits
        # V = np.clip(V, -self.V_max, self.V_max)
        # om = np.clip(om, -self.om_max, self.om_max)

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om
        
        msg = TurtleBotControl()
        msg.v = V
        msg.omega = om
        return msg
    
    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState, occupancy: StochOccupancyGrid2D, resolution: float, horizon: float) -> TrajectoryPlan:
        x = state.x
        y = state.y
        x_d = goal.x
        y_d = goal.y
        
        astar = AStar(
            (x - horizon * 0.5, y - horizon * 0.5),  # lower bounds
            (x + horizon * 0.5, y + horizon * 0.5),  # upper bounds
            (x, y),  # start
            (x_d, y_d),  # goal
            occupancy,
            resolution
        )
        
        if not astar.solve() or len(astar.path) < 4:
            #return None
            return None
            # path_array = np.array([[state.x, state.y]])  # <- make sure it's an array
            # return TrajectoryPlan(
            #     path=path_array,
            #     path_x_spline=splrep([0], [state.x]),
            #     path_y_spline=splrep([0], [state.y]),
            #     duration=0.0,
            # )
        
        self.reset()
        return self.compute_smooth_plan(astar.path)
    
    def reset(self) -> None:
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0
    
    def compute_smooth_plan(self, path, v_desired=0.15, spline_alpha=0.075) -> TrajectoryPlan:
        # Compute and set the following variables:
        #   1. ts: 
        #      Compute an array of time stamps for each planned waypoint assuming some constant 
        #      velocity between waypoints. 
        #
        #   2. path_x_spline, path_y_spline:
        #      Fit cubic splines to the x and y coordinates of the path separately
        #      with respect to the computed time stamp array.
        #      Hint: Use scipy.interpolate.splrep

        path = np.array(path)
        total_time = (len(path[:, 0]) - 1) / v_desired
        ts = np.linspace(0, total_time, len(path))
        path_x_spline = splrep(ts, path[:, 0], s=spline_alpha)
        path_y_spline = splrep(ts, path[:, 1], s=spline_alpha)
        
        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )
    
if __name__ == "__main__": 
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()