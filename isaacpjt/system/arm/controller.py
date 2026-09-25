"""로봇 한 대의 팔 작업 상태 기계 — pick_and_place_detection.main() 을 인스턴스로 옮긴 것.

명령 (ArmRosIO: /robot{i}/arm/command)
    load    스캔 -> 하강 -> (검출 대기) -> 중앙 정렬 -> (재검출) -> 파지/랙 적재  x 랙 3칸
            -> 랙 관측(ArUco 긴급도) -> 홈.  실패는 원본과 같이 복구하고 미션을 멈추지 않는다
    unload  적재된 랙 칸을 3번부터 꺼내 책상 DESK_SLOTS 에 놓는다
    stop    진행 중인 동작을 멈춘다 (팔은 마지막 목표에 멈춰 선다)

원본과 다른 점
    - /mission_state, /nav_done 핸드셰이크와 --drive-only 가 없다. 적재 완료는 status 의
      result=done 으로 알리고, 하역은 unload 명령으로 시작한다 (원본의 wait_unload / U 키 자리).
    - 손목 카메라는 load 동안만 켠다 (하역은 검출을 안 쓴다). 로봇 3대 동시 발행 시 약 2 Hz (R7).
    - tick() 은 물리 콜백(60 Hz)에서 부른다. 원본도 60 Hz 에 1틱이었으므로 스텝/대기 프레임 수의
      시뮬 시간 의미가 같다. 단 카메라는 렌더(30 Hz)마다 4프레임에 한 번이라 원본의 절반 빈도다.
"""
import carb
import numpy as np
import omni.usd
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from pxr import Usd, UsdPhysics

from .motion import *  # noqa: F401,F403  (geometry/config 포함)
from .rosio import parse_command
from .trays import TrayHooks

STATUS_PERIOD_TICKS = 30   # 상태가 안 바뀌어도 이 간격(0.5 s)으로 다시 발행한다 (늦게 붙은 구독자용)


class ArmTaskController:
    def __init__(self, index, robot_prim, io, registry):
        self.index = index
        self.tag = f"[robot{index}]"
        self.robot_prim = robot_prim
        self.io = io
        self.hooks = TrayHooks(registry, self.tag)
        self.art_root = f"{robot_prim}/{ART_ROOT_REL}"
        self.robot = self.lula = self.ik = None
        self.arm_indices = []
        self.color_camera = None
        self._base = SingleXFormPrim(f"{robot_prim}/{ARM_BASE_REL}")
        self._last_command = ""
        self._status_tick = 0
        self._status_sent = None
        self.cmd = ""
        self.cmd_text = ""           # 받은 명령 문자열 그대로(id 포함) — 에이전트가 자기 명령의 결과인지 맞춰 본다
        self.result = "idle"
        self.detail = ""
        self.loaded = [False] * len(RACK_SLOTS)
        self.aruco = [-1] * len(RACK_SLOTS)
        self.unloaded = [False] * len(RACK_SLOTS)
        self._unload_started = [False] * len(RACK_SLOTS)
        self.warnings = []           # 제자리에 못 놓은 트레이 등 — status 로 관제에 알린다
        self._reset_run_state()

    def log(self, text):
        print(f"{self.tag} {text}", flush=True)

    def _reset_run_state(self):
        # 원본 main() 의 지역 변수들
        self.state = None          # None = 대기. 원본 pick_state
        self.settle_next = None    # "center" | "pick"
        self.sequence = None
        self.seq_mark = -1
        self.wait_frames = 0
        self.retry_count = 0
        self.slot_index = 0
        self.unload_queue = []
        self.votes, self.last_marker_seq, self.collect_start = [], -1, -1
        self.step_retries = 0
        self.stage_fails = 0
        self.fail_streak = 0
        self.tick_count = 0

    # ── 준비 (world.reset 전) ─────────────────────────────────────
    def setup_drives(self, stage):
        """팔 관절 Drive 강화 + 그리퍼 최대 힘 제한 (원본 setup_arm_drives / setup_gripper_drive).
        복제 로봇은 세션 레이어에 정의돼 있어 세션 레이어에 써야 가려지지 않는다."""
        count = 0
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for prim in Usd.PrimRange(stage.GetPrimAtPath(self.robot_prim)):
                name = prim.GetName()
                if name in ARM_JOINTS:
                    values = (DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE)
                elif name in GRIPPER_JOINTS:
                    values = (GRIPPER_DRIVE_STIFFNESS, GRIPPER_DRIVE_DAMPING, GRIPPER_DRIVE_MAX_FORCE)
                else:
                    continue
                for drive_type in ("angular", "linear"):
                    drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                    if drive:
                        drive.GetStiffnessAttr().Set(values[0])
                        drive.GetDampingAttr().Set(values[1])
                        drive.GetMaxForceAttr().Set(values[2])
                        count += 1
        self.log(f"drives       {count}")

    def register(self, world):
        """로봇과 그리퍼를 Articulation 으로 등록한다 (원본 register_robot). 아티큘레이션 루트 = chassis_link."""
        ee = self._find_named(EE_LINK_NAME, self.robot_prim)
        if not ee:
            raise RuntimeError(f"{self.tag} '{EE_LINK_NAME}' not found under {self.robot_prim}")
        gripper = ParallelGripper(
            end_effector_prim_path=ee[0],
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )
        self.robot = world.scene.add(SingleManipulator(
            prim_path=self.art_root, name=f"m0609_robot_{self.index}",
            end_effector_prim_path=ee[0], gripper=gripper))
        self.log(f"EE frame     {ee[0]}")

    @staticmethod
    def _find_named(name, root_path):
        stage = omni.usd.get_context().get_stage()
        return [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root_path))
                if p.GetName() == name]

    # ── 준비 (world.reset 후) ─────────────────────────────────────
    def post_reset(self, world):
        self.init_robot(world)
        rsd455 = self._find_named(D455_CAMERA_NAME, self.robot_prim)
        color = self._find_named(D455_COLOR_CAMERA_NAME, rsd455[0]) if rsd455 else []
        if not color:
            raise RuntimeError(f"{self.tag} wrist camera {D455_COLOR_CAMERA_NAME} not found")
        self.color_camera = color[0]

        self.lula = LulaKinematicsSolver(robot_description_path=DESCRIPTION_PATH, urdf_path=URDF_PATH)
        self.lula.set_default_position_tolerance(IK_POSITION_TOLERANCE)
        self.lula.set_default_orientation_tolerance(IK_ORIENTATION_TOLERANCE)
        missing = [j for j in self.lula.get_joint_names() if j not in self.robot.dof_names]
        if missing:
            raise RuntimeError(f"{self.tag} descriptor joints missing in USD: {missing}")
        ik = ArticulationKinematicsSolver(robot_articulation=self.robot, kinematics_solver=self.lula,
                                          end_effector_frame_name=EE_LINK_NAME)
        self.arm_indices = [self.robot.get_dof_index(j) for j in ARM_JOINTS]
        self.ik = SingularityGuardedIK(ik, self.robot, self.arm_indices)
        base_pos, _ = self.sync_base()
        self.log(f"arm base     {vec(base_pos)}   camera {self.color_camera}")

    def init_robot(self, world):
        """Articulation/그리퍼 초기화 후 팔만 홈 관절로 (원본 init_robot). 카터 dof 가 섞여 있어
        팔 관절만 골라 덮는다 — 전부 덮으면 바퀴까지 리셋돼 로봇이 튄다."""
        self.robot.initialize()
        self.robot.gripper.initialize(
            physics_sim_view=world.physics_sim_view,
            articulation_apply_action_func=self.robot.apply_action,
            get_joint_positions_func=self.robot.get_joint_positions,
            set_joint_positions_func=self.robot.set_joint_positions,
            dof_names=self.robot.dof_names,
        )
        q = self.robot.get_joint_positions()
        for name, angle in zip(ARM_JOINTS, READY_JOINTS_RAD):
            q[self.robot.get_dof_index(name)] = angle
        self.robot.set_joint_positions(q)

    def sync_base(self):
        """팔 base_link 의 현재 월드 pose 를 솔버에 넘긴다 (원본 sync_base_pose)."""
        base_pos, base_quat = self._base.get_world_pose()
        if self.lula is not None:
            self.lula.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
        return base_pos, base_quat

    # ── Play / Stop ─────────────────────────────────────────────
    def on_play(self, world):
        self.init_robot(world)
        self._reset_run_state()
        self.hooks.reset()
        # 제네릭 구독 노드는 이전 Play 때 받은 명령을 들고 있다 — 그걸 새 명령으로 보지 않는다
        self._last_command = self.io.read_command()
        self.cmd, self.cmd_text, self.result, self.detail = "", "", "idle", ""
        self.io.set_camera(False)
        self.publish(force=True)

    def on_stop(self):
        self._reset_run_state()
        self.io.set_camera(False)

    # ── 명령 ─────────────────────────────────────────────────────
    def _new(self, steps):
        seq = PickPlaceSequence(self.robot, self.ik, self.arm_indices, steps, tag=self.tag)
        seq.reset()
        self.tick_count = 0
        return seq

    def poll_command(self):
        text = self.io.read_command()
        if text == self._last_command:
            return
        self._last_command = text
        self.command(parse_command(text), text)

    def command(self, cmd, text=""):
        if cmd in ("load", "unload", "stop") and (self.state is None or cmd == "stop"):
            self.cmd_text = text
        if cmd == "stop":
            self._reset_run_state()
            self.io.set_camera(False)
            self.cmd, self.result, self.detail = cmd, "stopped", text
            self.log("stop         진행 중인 동작을 멈췄다")
        elif cmd not in ("load", "unload"):
            self.log(f"command      알 수 없는 명령 무시: {text!r}")
            return
        elif self.state is not None:
            self.log(f"command      {self.cmd} 진행 중 — {text!r} 무시")
            self.detail = f"busy, ignored {text}"
        elif cmd == "load":
            self._reset_run_state()
            self.hooks.reset()
            self.cmd, self.result, self.detail = cmd, "running", text
            self.loaded = [False] * len(RACK_SLOTS)
            self.aruco = [-1] * len(RACK_SLOTS)
            self.warnings = []
            self.io.set_camera(True)
            self.state = "scan"
            self.sequence = self._new(build_scan_steps())
            self.log(f"load         시작 ({text})")
        else:  # unload
            self._reset_run_state()
            # 이 컨트롤러가 적재한 칸만. 적재 기록이 없으면(재시작 등) 원본처럼 세 칸 다
            filled = [k for k in range(len(RACK_SLOTS)) if self.loaded[k]] or list(range(len(RACK_SLOTS)))
            self.unload_queue = sorted(filled, reverse=True)   # 3번부터 (원본 unload_slot 순서)
            self.unloaded = [False] * len(RACK_SLOTS)
            self._unload_started = [k in self.unload_queue for k in range(len(RACK_SLOTS))]
            self.warnings = []
            self.cmd, self.result, self.detail = cmd, "running", text
            self._start_unload_slot()
        self.publish(force=True)

    def _start_unload_slot(self):
        slot = self.unload_queue[0]
        base_pos, base_quat = self.sync_base()
        self.log(f"unload       랙 {slot + 1}번 -> 책상 {slot + 1}자리 (z {DESK_Z_M:.3f})")
        self.state = "unload"
        self.sequence = self._new(build_unload_steps(self.lula, slot, DESK_Z_M, base_pos, base_quat,
                                                         self.hooks.registry.tray_yaw_base_deg(), self.hooks))

    def _check_placed(self, where, target_base, z_expected):
        """target_base(팔 base 기준) 에 트레이가 제대로 놓였는지. 가장 가까운 트레이 body 원점을 본다.

        시퀀스가 끝났다고 트레이가 제자리에 있는 건 아니다 — 물린 채 기울면 랙 테두리에 걸리거나
        하역 중 떨어진다(2026-09-25 실측). 결과를 status(loaded/unloaded/warnings)에 그대로 올린다."""
        base_pos, base_quat = self.sync_base()
        target = np.asarray(base_pos, dtype=float) + quat_to_matrix(base_quat) @ np.asarray(target_base, dtype=float)
        path, dist = self.hooks.registry.nearest(target, ())
        if path is None:
            self.warnings.append(f"{where}: no tray")
            return False
        pos, _ = self.hooks.registry.body(path).get_world_pose()
        here = world_to_base_pos(pos, base_pos, base_quat)
        z = float(here[2])
        dx, dy = (here[:2] - np.asarray(target_base, dtype=float)[:2]) * 100   # base x = 랙 칸 폭 방향
        ok = dist <= PLACE_XY_TOL_M and abs(z - z_expected) <= PLACE_Z_TOL_M
        self.log(f"placed       {where}: {path}  수평 오차 {dist * 100:.1f}cm (base x {dx:+.1f} y {dy:+.1f})  z(base) {z:+.3f} "
                 f"(기준 {z_expected:+.3f})  {'OK' if ok else '제자리 아님'}")
        if not ok:
            self.warnings.append(f"{where}: {path} off by {dist * 100:.0f}cm, z {z:+.3f}")
        return ok

    def _finish(self, result, detail=""):
        self._reset_run_state()
        self.io.set_camera(False)
        self.result, self.detail = result, detail
        self.publish(force=True)

    # ── 상태 발행 ─────────────────────────────────────────────────
    def status(self):
        return {
            "robot": f"robot{self.index}",
            "cmd": self.cmd,
            "cmd_text": self.cmd_text,
            "result": self.result,           # idle | running | done | failed | stopped
            "state": self.state or "idle",
            "slot": self.slot_index if self.cmd == "load" else (self.unload_queue[0] if self.unload_queue else -1),
            "loaded": list(self.loaded),           # 랙 제자리에 놓인 것만 True
            "aruco": list(self.aruco),             # 칸별 ArUco id (0 하 / 1 중 / 2 상 / -1 미검출)
            "urgency": aruco_to_urgency(self.aruco),   # 1~3, 3 이 가장 높다 (D2)
            "unloaded": list(self.unloaded),       # 책상 제자리에 놓인 것만 True
            "warnings": list(self.warnings),
            "detail": self.detail,
        }

    def publish(self, force=False):
        status = self.status()
        self._status_tick += 1
        if force or status != self._status_sent or self._status_tick >= STATUS_PERIOD_TICKS:
            self.io.publish_status(status)
            self._status_sent = status
            self._status_tick = 0

    # ── 검출 (원본 iter_tray_optical / first_tray_grasp) ───────────
    def _iter_tray_optical(self, verbose=False):
        _seq, detections = self.io.read_detections(verbose=verbose)
        for i, (point_optical, conf) in enumerate(detections):
            depth = float(point_optical[2])
            if not (DETECT_DEPTH_MIN_M <= depth <= DETECT_DEPTH_MAX_M):
                self.log(f"트레이{i + 1} 무시 — 깊이 {depth:.3f}m 가 "
                         f"{DETECT_DEPTH_MIN_M}~{DETECT_DEPTH_MAX_M}m 범위 밖이다")
                continue
            yield point_optical, conf

    def _first_tray_grasp(self, base_pos, base_quat, verbose=False, centered_first=False):
        cands = list(self._iter_tray_optical(verbose=verbose))
        if centered_first:
            cands.sort(key=lambda c: abs(float(c[0][0])))
        for i, (point_optical, conf) in enumerate(cands):
            off = abs(float(point_optical[0]))
            if centered_first and off > CENTER_TOL_M:
                self.log(f"후보{i + 1} 무시 — 화면 중앙에서 {off * 100:.0f}cm 벗어났다 "
                         f"(허용 {CENTER_TOL_M * 100:.0f}cm). 정렬한 그 트레이가 아니다")
                continue
            tray_world = optical_to_world(point_optical, self.color_camera)
            grasp, yaw = plan_grasp(self.lula, tray_world, base_pos, base_quat,
                                    self.hooks.registry.tray_yaw_base_deg())
            reach = float(np.linalg.norm(grasp))
            if not (GRASP_REACH_MIN_M <= reach <= GRASP_REACH_MAX_M):
                self.log(f"후보{i + 1} 무시 — 파지점이 base 에서 {reach:.3f}m "
                         f"({GRASP_REACH_MIN_M}~{GRASP_REACH_MAX_M}m 밖). 다음 후보로 넘어간다")
                continue
            self.log(f"트레이 conf {conf:.2f}  cam {vec(point_optical, 3)}  world {vec(tray_world)}  "
                     f"파지(base) {vec(grasp, 4)}  reach {reach:.3f}m  yaw {yaw:+.1f}deg")
            return grasp, yaw
        return None

    def _begin_recover(self, reason, base_pos, base_quat, grasp=None):   # grasp = (파지점, yaw)
        """적재 실패 복구 — 들고 있으면 원래 자리에 되돌려 놓고, 아니면 홈으로만 (원본 begin_recover)."""
        held = self.sequence.gripper
        self.stage_fails += 1
        self.step_retries = 0
        where = "트레이를 원래 자리에 되돌려 놓고" if grasp is not None else "홈으로 빠져"
        self.log(f"복구         {reason} — {where} 계속한다 (적재 실패 {self.stage_fails}/{MAX_STAGE_FAILS})")
        self.sequence = self._new(build_return_steps(self.lula, *grasp, base_pos, base_quat)
                                  if grasp is not None else [home_step()])
        self.sequence.gripper = held
        self.state = "recover"

    def _retry_or_recover(self, base_pos, base_quat):
        self.retry_count += 1
        if self.retry_count > MAX_DETECT_RETRIES:
            self._begin_recover(f"{MAX_DETECT_RETRIES}번 재시도해도 트레이를 못 찾았다", base_pos, base_quat)
        else:
            self.log(f"pick         트레이를 못 찾음 — 재시도 ({self.retry_count}/{MAX_DETECT_RETRIES})")
            self.seq_mark, self.wait_frames = self.io.read_detections()[0], 0

    # ── 한 틱 (물리 콜백) ─────────────────────────────────────────
    def tick(self):
        self.poll_command()
        if self.state is None:
            self.publish()
            return
        base_pos, base_quat = self.sync_base()

        if self.state == "observe_settle":
            self._tick_observe_settle()
        elif self.state == "settle":
            self._tick_settle(base_pos, base_quat)
        else:
            self._tick_sequence(base_pos, base_quat)
        if self.state is not None:
            self.publish()

    def _tick_observe_settle(self):
        self.wait_frames += 1
        seq_now, markers = self.io.read_markers()
        fresh = seq_now - self.seq_mark >= FRESH_SEQ_ADVANCE
        if self.wait_frames < DETECT_SETTLE_FRAMES or not (fresh or self.wait_frames > FRESH_TIMEOUT_FRAMES):
            return
        if fresh and seq_now != self.last_marker_seq:      # 새 메시지마다 한 표
            self.last_marker_seq = seq_now
            self.votes.append(assign_slots(markers))
            if self.collect_start < 0:
                self.collect_start = self.wait_frames
        if (fresh and self.wait_frames - self.collect_start < OBSERVE_COLLECT_FRAMES
                and self.wait_frames <= FRESH_TIMEOUT_FRAMES):
            return
        if not self.votes:
            self.log(f"observe      마커 토픽을 {FRESH_TIMEOUT_FRAMES} 프레임 동안 못 받았다 — 미검출로 진행")
        self.aruco = merge_urgencies(self.votes)
        self.log(f"observe      {len(self.votes)}프레임 누적 (칸별 판독 횟수 "
                 f"{[sum(v[k] >= 0 for v in self.votes) for k in range(len(RACK_SLOTS))]}) "
                 f"-> 랙 {format_urgencies(self.aruco)}")
        self.state = "observe_home"
        self.sequence = self._new([home_step()])

    def _tick_settle(self, base_pos, base_quat):
        self.wait_frames += 1
        seq_now, _ = self.io.read_detections()
        if self.wait_frames < DETECT_SETTLE_FRAMES or seq_now - self.seq_mark < FRESH_SEQ_ADVANCE:
            if self.wait_frames > FRESH_TIMEOUT_FRAMES:
                self._begin_recover(f"새 검출을 {FRESH_TIMEOUT_FRAMES} 프레임 동안 못 받았다", base_pos, base_quat)
            return

        if self.settle_next == "center":
            found = next(self._iter_tray_optical(verbose=True), None)
            if found is None:
                self._retry_or_recover(base_pos, base_quat)
                return
            self.retry_count = 0
            point_optical, _conf = found
            self.sequence = self._new(build_center_step(self.ik, self.color_camera, point_optical))
            self.state = "center"
            return

        # settle_next == "pick" — 중앙 정렬 후 다시 검출해서 최종 파지점을 구한다
        try:
            found = self._first_tray_grasp(base_pos, base_quat, verbose=True, centered_first=True)
        except Exception:
            import traceback
            self.log("pick         계획 실패 — 아래 오류를 보고할 것")
            traceback.print_exc()
            found = None
        if found is None:
            self._retry_or_recover(base_pos, base_quat)
            return
        self.retry_count = 0
        grasp, yaw = found
        self.sequence = self._new(build_pick_steps(self.lula, grasp, yaw, base_pos, base_quat,
                                                   self.slot_index, self.hooks))
        self.state = "pick"

    def _tick_sequence(self, base_pos, base_quat):
        sequence = self.sequence
        solved = sequence.tick()
        if solved:
            self.fail_streak = 0
        else:
            self.fail_streak += 1
            if self.fail_streak == 1 or self.fail_streak % LOG_INTERVAL == 0:
                carb.log_warn(f"{self.tag} IK 미수렴 ({self.fail_streak})  [{self.state}] step {sequence.index}")

        if sequence.failed:
            self.log(f"[{self.state}] {sequence.failed}")
            if self.step_retries < MAX_STEP_RETRIES:
                # 손목을 특이점에서 띄우고 실패한 스텝부터 다시 — 뒤 스텝은 절대 목표라 그대로 이어진다
                held = sequence.gripper
                self.step_retries += 1
                self.log(f"복구         손목을 풀고 실패 스텝부터 재시도 ({self.step_retries}/{MAX_STEP_RETRIES})")
                self.sequence = self._new([wrist_unlock_step()] + sequence.steps[sequence.index:])
                self.sequence.gripper = held
                return
            self.step_retries = 0
            # 홈으로 빠지지 않고 다음 스텝으로 — 같은 시퀀스라 그리퍼 상태(들고 있는 트레이)가 유지된다
            self.log(f"건너뜀       step {sequence.index} '{sequence.current['label']}' — 다음 스텝으로 넘어간다")
            sequence.skip()
            self.tick_count = 0
            return

        if self.tick_count % LOG_INTERVAL == 0 and not sequence.done:
            self.log(f"[{self.state}] step {sequence.index} '{sequence.current['label']}'   "
                     f"{sequence.step_tick}/{sequence.n_steps}")
        self.tick_count += 1
        if sequence.done:
            self._on_sequence_done(base_pos, base_quat)

    def _on_sequence_done(self, base_pos, base_quat):
        state = self.state
        if state == "scan":
            # joint 회전만으로는 원하는 높이가 안 나온다 — 회전 직후 실제 위치에서 z 만 낮춘다
            self.state = "descend"
            self.sequence = self._new(build_descend_step(self.ik, base_pos, base_quat,
                                                         SCAN_DESCEND_Z_M, self.color_camera))
        elif state == "descend":
            self.state, self.settle_next = "settle", "center"
            self.seq_mark, self.wait_frames = self.io.read_detections()[0], 0
        elif state == "center":
            self.state, self.settle_next = "settle", "pick"
            self.seq_mark, self.wait_frames = self.io.read_detections()[0], 0
        elif state == "recover":
            if self.stage_fails >= MAX_STAGE_FAILS or self.slot_index >= len(RACK_SLOTS):
                self.log(f"복구         적재 실패 {self.stage_fails}회 — 남은 칸을 포기하고 관측으로 넘어간다")
                self._start_observe()
            else:
                self.log(f"복구         복구 완료 — 다음 트레이 스캔 (랙 {self.slot_index + 1}번부터 다시)")
                self._start_scan()
        elif state == "observe_go":
            self.state = "observe_settle"
            self.seq_mark, self.wait_frames = self.io.read_markers()[0], 0
            self.votes, self.last_marker_seq, self.collect_start = [], -1, -1
        elif state == "observe_home":
            self.log(f"load         완료 — 적재 {self.loaded}  긴급도 {aruco_to_urgency(self.aruco)}")
            self._finish("done" if any(self.loaded) else "failed",
                         "" if any(self.loaded) else "no tray loaded")
        elif state == "unload":
            slot = self.unload_queue.pop(0)
            self.unloaded[slot] = self._check_placed(f"책상 {slot + 1}자리", DESK_SLOTS[slot], DESK_TRAY_Z_BASE)
            self.loaded[slot] = False
            if self.unload_queue:
                self._start_unload_slot()
            else:
                ok = all(self.unloaded[k] for k in range(len(RACK_SLOTS)) if self._unload_started[k])
                self.log(f"unload       하역 완료 — 제자리 {self.unloaded}")
                self._finish("done" if ok else "failed", "" if ok else "tray not on desk slot")
        else:  # pick 완료 — 다음 칸이 남았으면 홈에서 다시 스캔한다
            self.loaded[self.slot_index] = self._check_placed(
                f"랙 {self.slot_index + 1}번", RACK_SLOTS[self.slot_index], RACK_TRAY_Z_BASE)
            self.slot_index += 1
            self.step_retries = 0
            if self.slot_index >= len(RACK_SLOTS):
                self.log("pick         랙이 다 찼다 — 랙을 위에서 관측해 긴급도를 읽는다")
                self._start_observe()
            else:
                self.log(f"pick         랙 {self.slot_index}번 완료 — 다음 트레이 스캔")
                self._start_scan()

    def _start_scan(self):
        self.state = "scan"
        self.sequence = self._new(build_scan_steps())
        self.seq_mark, self.wait_frames, self.retry_count = -1, 0, 0

    def _start_observe(self):
        self.state = "observe_go"
        self.sequence = self._new(build_observe_step())
