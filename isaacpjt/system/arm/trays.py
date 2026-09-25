"""책상 트레이 복제 + 랙 정렬 — pick_and_place_detection.py 에서 옮겼다.

원본은 트레이 상태를 모듈 전역에 뒀다. 여기서는 둘로 나눈다.
  TrayRegistry  씬에 하나. 트레이 목록, rigid body 핸들, 랙 '반듯한 자세' 기준(스폰 자세).
  TrayHooks     로봇마다 하나. 물고 있는 트레이(p_rel), 이번 적재에서 이미 정렬한 트레이.
                원본은 정렬 기록을 실행 내내 들고 있었는데, 운송을 여러 번 돌면 다음 사이클의
                트레이가 후보에서 빠진다 — 적재 명령마다 reset() 한다.
"""
import random

import numpy as np
import omni.usd
from isaacsim.core.prims import SingleRigidPrim
from pxr import Gf, Sdf, UsdGeom, UsdShade

from .geometry import *  # noqa: F401,F403


def attach_aruco(tray_path, marker_id):
    """트레이 손잡이 윗판 위에 ArUco 텍스처 사각형(시각 전용, 콜리전 없음)을 붙인다.

    handle Xform 의 자식이라 트레이가 움직이면 같이 따라간다. 위에서 볼 때 반시계 순서로
    점을 두고 st 를 같은 순서로 주면 이미지가 거울상 없이 보인다(거울상이면 id 가 안 읽힌다)."""
    stage = omni.usd.get_context().get_stage()
    root = f"{tray_path}/{ARUCO_HANDLE_REL}/aruco"
    h, z = ARUCO_SIDE_M / 2, ARUCO_TOP_Z_M
    mesh = UsdGeom.Mesh.Define(stage, root)
    mesh.CreatePointsAttr([(-h, -h, z), (h, -h, z), (h, h, z), (-h, h, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0, 0, 1)] * 4)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                            UsdGeom.Tokens.varying).Set([(0, 0), (1, 0), (1, 1), (0, 1)])

    mat = UsdShade.Material.Define(stage, f"{root}/mat")
    pbr = UsdShade.Shader.Define(stage, f"{root}/mat/pbr")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)   # 반사광으로 셀이 날아가지 않게
    tex = UsdShade.Shader.Define(stage, f"{root}/mat/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"{ARUCO_DIR}/aruco_{marker_id}.png")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    st = UsdShade.Shader.Define(stage, f"{root}/mat/st")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.CreateOutput("result", Sdf.ValueTypeNames.Float2))
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
    mat.CreateSurfaceOutput().ConnectToSource(pbr.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    print(f"   aruco        {tray_path}  id {marker_id} (긴급도 {'하중상'[marker_id]})")
def prim_world_quat(path):
    """프림의 월드 자세(쿼터니언 w,x,y,z). xformOp 구성이 무엇이든 합성 행렬에서 뽑는다"""
    stage = omni.usd.get_context().get_stage()
    mat = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    q = mat.RemoveScaleShear().ExtractRotationQuat().GetNormalized()
    return np.array([q.GetReal(), *q.GetImaginary()], dtype=float)



TRAY_BODY_REL = "tray"   # 실제 rigid body 는 한 단계 아래다 (/World/tray/tray)


class TrayRegistry:
    def __init__(self):
        self.bodies = []            # 트레이 rigid body 프림 경로
        self._rigid = {}            # 경로 -> SingleRigidPrim (처음 쓸 때 만들어 재사용)
        # 스폰 자세(base 기준). 랙에서 '반듯한' 자세의 기준이다 — 책상 위에 반듯하게 놓여 있고
        # IK 를 안 거친 자세다. 로봇은 모두 같은 모델이고 같은 식으로 도킹하므로 한 번 재서 같이 쓴다
        self.spawn_quat_base = None

    def spawn_copies(self, base_pos, base_quat, update):
        """원본 트레이를 TRAY_COPIES 개 복제해 한 줄로 흩뿌리고, 전부에 긴급도 마커를 붙인다.
        world.reset() 전에 부른다. base_pos/base_quat = 트레이 책상에 도킹한 로봇의 팔 base.

        벌리는 방향은 **base 에서 트레이를 향하는 방향에 수직인 수평축**이다. 반지름 방향으로
        벌리면 파지점 도달거리가 0.69~1.09m 로 퍼지는데, 1.10m 부근은 joint_5 가 0 으로 밀려
        손목 특이점이다. 가로로만 벌리면 0.90~0.93m 에 모여 전부 안전하다 (원본 spawn_tray_copies)."""
        stage = omni.usd.get_context().get_stage()
        translate = stage.GetPrimAtPath(TRAY_PRIM_PATH).GetAttribute("xformOp:translate")
        origin = np.array(translate.Get(), dtype=float)
        # 원본 트레이를 도킹한 로봇 기준 제자리(TRAY_ORIGIN_BASE)로 옮긴다. 높이(책상 위)는 씬 값 그대로
        target = np.asarray(base_pos, dtype=float) + quat_to_matrix(base_quat) @ np.array([*TRAY_ORIGIN_BASE, 0.0])
        moved = float(np.linalg.norm(target[:2] - origin[:2]))
        origin[:2] = target[:2]
        translate.Set(Gf.Vec3d(*origin))
        print(f"   tray origin  world {vec(origin)}  (base {TRAY_ORIGIN_BASE}, 씬 위치에서 {moved * 100:.1f}cm 이동)")
        radial = world_to_base_pos(origin, base_pos, base_quat)[:2]
        radial = radial / np.linalg.norm(radial)
        lateral = quat_to_matrix(base_quat) @ np.array([-radial[1], radial[0], 0.0])
        print(f"   tray spread  가로축(월드) {vec(lateral)}  범위 +-{TRAY_SPREAD_R_M * 100:.0f}cm")

        self.bodies = [f"{TRAY_PRIM_PATH}/{TRAY_BODY_REL}"]
        self.spawn_quat_base = quat_mul(
            quat_conj(base_quat), prim_world_quat(f"{TRAY_PRIM_PATH}/{TRAY_BODY_REL}"))
        print(f"   tray spawn   기준 자세(base) rpy "
              f"{vec(matrix_to_rpy(quat_to_matrix(self.spawn_quat_base)), 1)}  "
              f"방위각 {tray_yaw_deg(self.spawn_quat_base):+.1f}deg")

        placed = [origin]
        attach_aruco(TRAY_PRIM_PATH, random.choice(ARUCO_IDS))
        for i in range(1, TRAY_COPIES + 1):
            for _ in range(20):
                pos = origin + lateral * np.random.uniform(-TRAY_SPREAD_R_M, TRAY_SPREAD_R_M)
                if all(np.linalg.norm(pos[:2] - q[:2]) >= TRAY_MIN_GAP_M for q in placed):
                    break
            placed.append(pos)
            path = f"{TRAY_PRIM_PATH}_{i}"
            omni.usd.duplicate_prim(stage, TRAY_PRIM_PATH, path)
            stage.GetPrimAtPath(path).GetAttribute("xformOp:translate").Set(Gf.Vec3d(*pos))
            print(f"   tray copy    {path}  world {vec(pos)}")
            attach_aruco(path, random.choice(ARUCO_IDS))
            self.bodies.append(f"{path}/{TRAY_BODY_REL}")
        update()
        return origin

    def preload_rack(self, robot_index, base_pos, base_quat, update):
        """로봇 랙 3칸에 트레이를 실어 둔다 — 하역만 따로 시험할 때 (spawn_copies 뒤, world.reset 전).

        자리는 적재가 실제로 남기는 자리: body 원점 = 칸 TCP 목표 + (0, RACK_TRAY_ORIGIN_DY), 높이는 칸 바닥,
        방위는 책상 스폰 방위 + 90도 (랙에서 트레이 로컬 +X 가 base -x, 2026-09-25 적재 실측과 같다)."""
        stage = omni.usd.get_context().get_stage()
        rack_quat_base = quat_mul(quat_from_axis([0, 0, 1], 90.0), self.spawn_quat_base)
        world_quat = quat_mul(base_quat, rack_quat_base)
        for slot, tcp in enumerate(RACK_SLOTS):
            origin_base = np.array([tcp[0], tcp[1] + RACK_TRAY_ORIGIN_DY, RACK_TRAY_Z_BASE + 0.003])
            pos = np.asarray(base_pos, dtype=float) + quat_to_matrix(base_quat) @ origin_base
            path = f"{TRAY_PRIM_PATH}_rack{robot_index}_{slot + 1}"
            omni.usd.duplicate_prim(stage, TRAY_PRIM_PATH, path)
            prim = stage.GetPrimAtPath(path)
            prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*pos))
            prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(float(world_quat[0]), *map(float, world_quat[1:])))
            attach_aruco(path, random.choice(ARUCO_IDS))
            self.bodies.append(f"{path}/{TRAY_BODY_REL}")
            print(f"   rack preload robot{robot_index} 랙 {slot + 1}번  {path}  world {vec(pos)}")
        update()

    def tray_yaw_base_deg(self):
        """책상 트레이의 base 기준 방위(로컬 +X). 파지 yaw 를 이 축에 맞춘다 (geometry.square_grasp_yaw)."""
        return tray_yaw_deg(self.spawn_quat_base)

    def reset_handles(self):
        """Play 를 다시 누를 때 — stop/play 로 물리 뷰가 새로 만들어지면 예전 핸들은 죽는다."""
        self._rigid.clear()

    def body(self, path):
        """트레이 rigid body 핸들. 처음 쓸 때 만들어 두고 재사용한다"""
        body = self._rigid.get(path)
        if body is None:
            body = SingleRigidPrim(path, name="tray_body_" + path.strip("/").replace("/", "_"))
            body.initialize()
            self._rigid[path] = body
        return body

    def nearest(self, world_xy, exclude):
        """world_xy 에 가장 가까운, exclude 에 없는 트레이 rigid body 의 (경로, 거리).

        이미 정렬한 것을 후보에서 빼는 게 핵심이다 — 2·3번 칸을 다룰 때 옆 칸에 이미 세워 둔
        트레이가 후보로 들어오지 않는다. 트레이 하나당 정확히 한 번만 손댄다."""
        best, best_d = None, None
        for path in self.bodies:
            if path in exclude:
                continue
            pos, _ = self.body(path).get_world_pose()
            dist = float(np.linalg.norm(np.asarray(pos, dtype=float)[:2] - np.asarray(world_xy)[:2]))
            if best_d is None or dist < best_d:
                best, best_d = path, dist
        return best, best_d

    def rack_square_quat(self, quat_now_world, base_quat):
        """지금 자세를 '랙과 나란한' 가장 가까운 자세로 스냅한 월드 쿼터니언.

        랙에서 반듯한 자세는 **스폰 자세를 base z 축으로 90도의 정수배만큼 돌린 것들** 중 하나다.
        정수배 k 는 지금 자세에서 읽는다. 그리퍼에 물린 각도는 보지 않는다 (원본 rack_square_quat)."""
        spawn_base = self.spawn_quat_base
        now_base = quat_mul(quat_conj(base_quat), np.asarray(quat_now_world, dtype=float))
        k = round((tray_yaw_deg(now_base) - tray_yaw_deg(spawn_base)) / 90.0)
        target_base = quat_mul(quat_from_axis([0, 0, 1], k * 90.0), spawn_base)
        target = quat_mul(base_quat, target_base)
        return target / np.linalg.norm(target)


class TrayHooks:
    """로봇 한 대의 랙 정렬 상태. build_pick_steps 의 on_enter 훅으로 불린다."""

    def __init__(self, registry, tag=""):
        self.registry = registry
        self.tag = tag
        self.held = None        # 그리퍼에 물린 트레이와 body 원점 어긋남. capture 가 채운다
        self.aligned = set()    # 이번 적재에서 이미 정렬한 트레이

    def reset(self):
        self.held = None
        self.aligned.clear()

    def capture(self, seq, slot):
        """그리퍼를 열기 **직전에** 트레이 원점이 TCP 기준 어디에 있는지 잰다 (p_rel).

        set_world_pose 는 **body 원점**을 옮기는데 그게 트레이 형상 중심이 아닐 수 있다.
        파지점은 TCP 가 트레이 수평 중심에 오게 잡으므로 p_rel 이 곧 '형상 중심 대비 body 원점의
        어긋남'이고, 이걸 슬롯 좌표에 더해야 트레이가 칸 **중앙**에 선다 (원본 capture_tray_in_gripper)."""
        self.held = None
        if not RACK_TRAY_ALIGN:
            return
        try:
            tcp_pos, tcp_quat = seq._current_flange_pose()
            path, dist = self.registry.nearest(tcp_pos, self.aligned)
            if path is None or dist > RACK_TRAY_MATCH_R_M:
                near = "없다" if dist is None else f"최근접 {dist * 100:.1f}cm"
                print(f"{self.tag}   rack align   랙 {slot + 1}번 — 그리퍼 근처에 트레이가 {near}. 정렬을 건너뛴다")
                return
            pos, _quat = self.registry.body(path).get_world_pose()
            self.held = {
                "path": path,
                "slot": slot,
                "p_rel": quat_to_matrix(tcp_quat).T @ (np.asarray(pos, dtype=float) - tcp_pos),
            }
        except Exception as exc:
            print(f"{self.tag}   rack align   랙 {slot + 1}번 파지 상태 측정 실패 (정렬 생략): {exc}")

    def align(self, slot, base_pos, base_quat):
        """방금 놓은 트레이를 칸 중앙·칸 방향으로 한 번 맞춰 놓는다. 후퇴 스텝 진입 때 한 번.

        랙 칸에 정렬 가이드(자석)가 있다는 설정이다 — 팔 동작은 건드리지 않고 트레이 pose 를 직접 맞춘다.
        자세는 rack_square_quat, 위치는 슬롯 중심 xy + p_rel, **z 는 물리가 잡은 값 그대로**.
        선/각속도를 0 으로 눌러야 PhysX 가 곧바로 다시 틀지 않는다 (원본 align_tray_on_rack)."""
        held, self.held = self.held, None
        if not RACK_TRAY_ALIGN or held is None or held["slot"] != slot:
            return
        if held["path"] in self.aligned:   # 손목 풀기 재시도로 스텝이 다시 실행된 경우
            return
        try:
            body = self.registry.body(held["path"])
            pos, quat = body.get_world_pose()
            pos = np.asarray(pos, dtype=float)

            # 칸 바닥에 반듯이 앉은 트레이만 맞춘다 — 걸려서 기운 걸 세우면 칸막이에 박혀 튕겨 나간다
            local = quat_to_matrix(quat_mul(quat_conj(base_quat), np.asarray(quat, dtype=float)))
            tilt = float(np.degrees(np.arccos(np.clip(local[2, 2], -1.0, 1.0))))
            z = float(world_to_base_pos(pos, base_pos, base_quat)[2])
            if tilt > ALIGN_MAX_TILT_DEG or abs(z - RACK_TRAY_Z_BASE) > PLACE_Z_TOL_M:
                print(f"{self.tag}   rack align   랙 {slot + 1}번 {held['path']} — 기울기 {tilt:.1f}deg, "
                      f"z(base) {z:+.3f} (바닥 {RACK_TRAY_Z_BASE:+.3f}). 칸에 안 앉았다 — 정렬하지 않는다")
                return

            target_quat = self.registry.rack_square_quat(quat, base_quat)
            ideal_pos, ideal_quat = base_to_world(RACK_SLOTS[slot], POINT4_RPY, base_pos, base_quat)
            target_pos = ideal_pos + quat_to_matrix(ideal_quat) @ held["p_rel"]

            base_rot_t = quat_to_matrix(base_quat).T
            yaw_before = tray_yaw_deg(quat_mul(quat_conj(base_quat), np.asarray(quat, dtype=float)))
            yaw_after = tray_yaw_deg(quat_mul(quat_conj(base_quat), target_quat))
            before_rpy = matrix_to_rpy(base_rot_t @ quat_to_matrix(np.asarray(quat, dtype=float)))
            after_rpy = matrix_to_rpy(base_rot_t @ quat_to_matrix(target_quat))
            moved = float(np.linalg.norm(target_pos[:2] - pos[:2]))

            body.set_world_pose(position=np.array([target_pos[0], target_pos[1], pos[2]]),
                                orientation=target_quat)
            body.set_linear_velocity(np.zeros(3))
            body.set_angular_velocity(np.zeros(3))
            self.aligned.add(held["path"])

            print(f"{self.tag}   rack align   랙 {slot + 1}번  {held['path']}")
            print(f"{self.tag}                방위각(base) {yaw_before:+.1f} -> {yaw_after:+.1f}deg "
                  f"({yaw_after - yaw_before:+.1f})   xy {moved * 100:.1f}cm 이동")
            print(f"{self.tag}                rpy(base) {vec(before_rpy, 1)} -> {vec(after_rpy, 1)}")
        except Exception as exc:
            print(f"{self.tag}   rack align   랙 {slot + 1}번 정렬 실패 (무시하고 계속): {exc}")
