"""Exercise actual stage function with simulated action/ROS I/O, not Nav2.

AST loads only function definitions because this laptop has no nav2_msgs.
The geometry/selection functions remain real; action transport is a test double.
"""
import ast
from enum import Enum
import math
from pathlib import Path as FilePath
import sys
from types import SimpleNamespace as NS, ModuleType

import pytest

from nav_to_goal import hospital_avoidance as geometry


class Status(Enum):
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'


class PathMessage:
    def __init__(self):
        self.header = NS(frame_id='map', stamp=0)
        self.poses = []


def pose_message(nav, x, y, yaw):
    return NS(pose=NS(position=NS(x=x, y=y), orientation=NS(
        x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2))))


def stage_harness(monkeypatch, mode):
    clock = NS(t=1.)

    def step(*args, **kwargs):
        clock.t += .05

    def build(nav, route):
        path = PathMessage()
        path.poses = [pose_message(nav, i*.05, 0, 0) for i in range(801)]
        return path

    helper = ModuleType('nav_to_goal.hospital_mission')
    helper.MissionStatus = Status
    helper.STATIC_BLOCK_PERSISTENCE_S = 1.5
    helper.STATIC_BLOCK_POSITION_TOLERANCE_M = .5
    helper.create_pose = pose_message
    helper.build_path = build
    helper.remaining_path = lambda path, tf, index: (path, 0)
    helper.task_result_to_status = lambda result: result
    planner_calls = []
    def detour(*args):
        planner_calls.append(args)
        if mode not in ('static', 'stopped_person'):
            return None
        lane = geometry.LaneFrame(0, 0, 0, 40)
        _, points = geometry.offset_candidates(lane, (0, 0, 0))[0]
        points += [lane.world(9.5+i*.05, 0) for i in range(1, 11)]
        path = PathMessage()
        path.poses = [pose_message(None, *point) for point in points]
        return path
    helper.request_detour = detour
    monkeypatch.setitem(sys.modules, 'nav_to_goal.hospital_mission', helper)

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Navigator:
        def __init__(self):
            self.goals, self.events, self.publishers = [], [], {}
            self.canceled = False
            self.destroyed = []

        def followPath(self, path, **kwargs):
            self.goals.append(path)
            self.canceled = False
            return True

        def isTaskComplete(self):
            step()
            return self.canceled or (mode in ('clear', 'avoid') and clock.t > 2.) or (
                mode in ('static', 'stopped_person') and len(self.goals) > 1 and clock.t > 4.)

        def cancelTask(self):
            self.canceled = True

        def getResult(self):
            return Status.CANCELED if self.canceled else Status.SUCCEEDED

        def get_clock(self):
            return NS(now=lambda: NS(to_msg=lambda: clock.t))

        def get_logger(self):
            return NS(info=self.events.append)

        def create_publisher(self, typ, name, qos):
            self.publishers[name] = Publisher()
            return self.publishers[name]

        def destroy_subscription(self, sub):
            self.destroyed.append(sub)

        def destroy_publisher(self, pub):
            self.destroyed.append(pub)

    class Observations:
        static = local = object()

        def __init__(self, *args, **kwargs):
            self.subscriptions = ['tracks', 'odom', 'map', 'costmap']

        def now(self):
            return clock.t

        def snapshot(self):
            if mode == 'missing':
                return None
            tracks = []
            if mode == 'yield':
                tracks = [geometry.MovingBody(1, 2, 0, -1.1, 0, .4)]
            elif mode == 'avoid':
                tracks = [geometry.MovingBody(1, 6.8, 0, -1.1, 0, .4)]
            elif mode == 'stopped_person':
                tracks = [geometry.MovingBody(1, 4, 0, 0, 0, .4)]
            return (0, 0, 0), (.6, 0), tracks

        def path_clear(self, *args):
            return True

    namespace = {name: getattr(geometry, name) for name in (
        'SafetySettings', 'choose_candidate', 'densify_planner_path',
        'forward_path_valid', 'lane_for_path',
        'offset_candidates', 'path_clearance', 'path_escapes', 'tail_clear_for_rejoin', 'wrap')}
    namespace.update(math=math, SafetyObservations=Observations, Path=PathMessage,
        yaw_of=lambda q: 2*math.atan2(q.z, q.w),
        rclpy=NS(ok=lambda: True, spin_once=step),
        time=NS(monotonic=lambda: clock.t, sleep=step),
        QoSProfile=lambda **kw: None, QoSDurabilityPolicy=NS(TRANSIENT_LOCAL=1),
        String=lambda **kwargs: NS(**kwargs))
    if mode in ('static', 'stopped_person'):
        namespace['choose_candidate'] = lambda *args, **kwargs: None
    source = FilePath(__file__).parents[1]/'nav_to_goal/hospital_stage_runner.py'
    tree = ast.parse(source.read_text())
    functions = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[])
    exec(compile(functions, str(source), 'exec'), namespace)
    nav = Navigator()
    plan_pub = Publisher()
    monitor = (NS(observe=lambda *args: {'map_xy': (4., 0.),
                                        'dynamic': mode == 'stopped_person'})
               if mode in ('static', 'stopped_person') else None)
    result = namespace['follow_stage'](nav, None, plan_pub, 'lane_upper', None,
                                      'FollowPathMPPI', 'transit_goal_checker',
                                      blockage_monitor=monitor)
    return result, nav, plan_pub, planner_calls


def test_clear_stage_has_no_unnecessary_offset_or_planner(monkeypatch):
    result, nav, _, planner = stage_harness(monkeypatch, 'clear')
    assert result == Status.SUCCEEDED and len(nav.goals) == 1 and not planner
    assert len(nav.destroyed) == 6


def test_avoidance_preempts_once_and_displays_executed_path(monkeypatch):
    result, nav, plan, planner = stage_harness(monkeypatch, 'avoid')
    assert result == Status.SUCCEEDED
    assert len(nav.goals) == 2 and not planner
    assert any('state=AVOIDING' in line for line in nav.events)
    assert max(p.pose.position.y for p in plan.messages[-1].poses) >= 1.5


def test_person_yield_does_not_trigger_six_second_planner(monkeypatch):
    result, nav, _, planner = stage_harness(monkeypatch, 'yield')
    assert result == Status.FAILED and not planner
    assert len(nav.goals) == 1
    assert any('yield_budget_exceeded_not_proof_of_lane_blockage' in line for line in nav.events)


def test_missing_observations_prevent_initial_goal(monkeypatch):
    result, nav, _, planner = stage_harness(monkeypatch, 'missing')
    assert result == Status.FAILED and not nav.goals and not planner


@pytest.mark.parametrize('mode', ['static', 'stopped_person'])
def test_persistent_obstruction_uses_validated_planner_path(monkeypatch, mode):
    result, nav, _, planner = stage_harness(monkeypatch, mode)
    assert result == Status.SUCCEEDED
    assert len(planner) == 1 and len(nav.goals) == 2
    assert any('validated_static_planner' in line for line in nav.events)
    assert max(p.pose.position.y for p in nav.goals[-1].poses) >= 1.5
