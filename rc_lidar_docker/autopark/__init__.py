"""트레일러 후진 자동주차 (노트북 실행).  Hybrid A* → Planning MPC → Tracking MPC.

  python -m autopark run <Pi-IP>            Pi 웹 브리지에 자동주차 클라이언트로 접속
  python -m autopark plan --map maps/x.yaml --start X Y YAW BETA --goal X Y YAW --svg out.svg
  python -m unittest autopark.test_autopark
"""
import os

import yaml

from .hybrid_astar import HybridAStar, PlanResult
from .model import Footprint, GridMap, geometry_from_config, state_from_trailer_goal
from .planning_mpc import PlanningMPC, RefTrajectory
from .tracking_mpc import TrackingMPC

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.yaml')


def load_config(path=None):
    with open(path or DEFAULT_CONFIG, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def plan_parking(cfg, geom, grid, start, goal_trailer, cancel=None):
    """시작 상태(x,y,th,beta) + 목표 트레일러 자세(x,y,yaw) → (PlanResult, RefTrajectory 또는 None, goal_state)."""
    goal = state_from_trailer_goal(goal_trailer[0], goal_trailer[1], goal_trailer[2], geom)
    res = HybridAStar(geom, grid, cfg).plan(start, goal, cancel=cancel)
    if not res.ok:
        return res, None, goal
    traj = PlanningMPC(geom, grid, cfg).optimize(res, goal)
    return res, traj, goal
