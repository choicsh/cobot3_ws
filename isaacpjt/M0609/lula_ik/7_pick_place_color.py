"""M0609 wrist-camera color Pick & Place for Isaac Sim Standalone.

PC A publishes an rgb8 wrist-camera image, receives the color selected by PC B,
and runs exactly one simulated Pick & Place mission.  This file does not call a
real robot, controller, gripper driver, or Doosan ROS service.
"""

from isaacsim import SimulationApp


simulation_app = SimulationApp({'headless': False})

# The ROS 2 bridge must be enabled immediately after SimulationApp is created.
from isaacsim.core.utils.extensions import enable_extension


enable_extension('isaacsim.ros2.bridge')
simulation_app.update()

from pathlib import Path
import time

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from isaacsim.sensors.camera import Camera


# Paths
THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent
USD_PATH = str(
    M0609_DIR / 'Collected_m0609_camera_cube/m0609_camera_cube.usd'
)
URDF_PATH = str(M0609_DIR / 'doosan-robot2/urdf/m0609_isaac_sim.urdf')
DESCRIPTION_PATH = str(M0609_DIR / 'descriptor/m0609_description.yaml')


# ROS 2 contract
IMAGE_TOPIC = '/m0609/wrist_camera/image_raw'
COLOR_TOPIC = '/m0609/color_id'
CAMERA_FREQUENCY = 15.0
CAMERA_RESOLUTION = (640, 480)
COLOR_MAX_AGE_SEC = 2.0


# Robot and gripper configuration reused from the previous Lula exercises.
ROBOT_PRIM_PATH = '/World/m0609'
EE_LINK_NAME = 'link_6'
ARM_JOINTS = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6',
]
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
ROBOT_BASE_POS = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

GRIPPER_JOINTS = ['finger_joint', 'right_inner_knuckle_joint']
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSE_POS = 0.8
GRIPPER_WAIT = 120

FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# Mission geometry.  The current checked-in USD has no color-mission objects,
# so a complete missing set is created at explicit paths and reported.
MISSION_ROOT = '/World/ColorMission'
GENERATED_PRIMS = {
    'blue_cube': f'{MISSION_ROOT}/blue_cube',
    'green_cube': f'{MISSION_ROOT}/green_cube',
    'blue_marker': f'{MISSION_ROOT}/blue_place_marker',
    'green_marker': f'{MISSION_ROOT}/green_place_marker',
}
PICK_X_RANGE = (0.22, 0.34)
PICK_Y_RANGE = (-0.08, 0.08)
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT = 0.23

TCP_SPEED = 0.004
MIN_STEPS = 60
MAX_STEPS = 600
LOG_INTERVAL = 60

APPROACH_ROLL_DEG = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG = 0.0


def section(title):
    print(f"\n{'─' * 72}\n {title}\n{'─' * 72}")


def vec(value, digits=3):
    return '[' + ' '.join(f'{item:+.{digits}f}' for item in value) + ']'


def quat_mul(a, b):
    """Multiply quaternions in (w, x, y, z) order."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, degrees):
    half = np.radians(degrees) / 2.0
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return np.concatenate([[np.cos(half)], axis * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    quat = quat_mul(
        quat_from_axis([1, 0, 0], roll_deg),
        quat_from_axis([0, 1, 0], pitch_deg),
    )
    quat = quat_mul(quat, quat_from_axis([0, 0, 1], yaw_deg))
    return quat / np.linalg.norm(quat)


def quat_to_matrix(quat):
    w, x, y, z = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


def tcp_to_flange(tcp_position, quat):
    return np.asarray(tcp_position) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose(robot):
    position, quat = robot.end_effector.get_world_pose()
    return position + quat_to_matrix(quat) @ TCP_OFFSET


def steps_for(start, goal):
    distance = float(np.linalg.norm(goal - start))
    steps = int(np.clip(distance / TCP_SPEED, MIN_STEPS, MAX_STEPS))
    return steps, distance


def normalize_name(value):
    return ''.join(character for character in value.lower()
                   if character.isalnum())


def prim_path(prim):
    return str(prim.GetPath())


def print_prim_candidates(stage, heading):
    """Print relevant discovered prims instead of hiding a path mismatch."""
    print(f'\n   {heading}')
    relevant_tokens = ('camera', 'rgb', 'color', 'cube', 'block',
                       'marker', 'place', 'target', 'd455')
    relevant = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        path = prim_path(prim)
        if any(token in path.lower() for token in relevant_tokens):
            relevant.append(f'{path} [{prim.GetTypeName() or "untyped"}]')
    if not relevant:
        print('      (관련 이름을 가진 prim 없음)')
        return
    for item in relevant[:200]:
        print(f'      {item}')
    if len(relevant) > 200:
        print(f'      ... {len(relevant) - 200}개 생략')


def find_wrist_camera_path(stage):
    cameras = [
        prim for prim in Usd.PrimRange(stage.GetPseudoRoot())
        if prim.IsA(UsdGeom.Camera)
    ]
    if not cameras:
        print_prim_candidates(stage, '발견된 카메라 관련 prim')
        raise RuntimeError('USD stage에서 UsdGeom.Camera prim을 찾지 못했습니다.')

    def score(camera_prim):
        path = prim_path(camera_prim).lower()
        value = 0
        value += 8 if path.startswith(ROBOT_PRIM_PATH.lower() + '/') else 0
        value += 5 if 'color' in path or 'rgb' in path else 0
        value += 3 if 'd455' in path or 'realsense' in path else 0
        value += 1 if 'camera' in path else 0
        value -= 6 if 'depth' in path or 'infrared' in path else 0
        value -= 20 if 'omniversekit_persp' in path else 0
        return value

    ranked = sorted(cameras, key=score, reverse=True)
    best_score = score(ranked[0])
    best = [candidate for candidate in ranked if score(candidate) == best_score]
    if best_score <= 0 or len(best) != 1:
        print('   발견된 Camera prim:')
        for candidate in ranked:
            print(f'      {prim_path(candidate)} score={score(candidate)}')
        raise RuntimeError(
            'Wrist RGB Camera를 하나로 확정할 수 없습니다. '
            '위 실제 prim 이름을 확인해 선택 규칙을 갱신하세요.'
        )
    return prim_path(best[0])


def role_candidates(stage, color, role):
    result = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        path = prim_path(prim)
        if prim.GetTypeName() in ('Material', 'Shader', 'NodeGraph'):
            continue
        if not prim.IsA(UsdGeom.Xformable):
            continue
        normalized = normalize_name(path)
        if color not in normalized:
            continue
        looks_like_marker = any(
            token in normalized for token in ('marker', 'place', 'target')
        )
        looks_like_cube = any(
            token in normalized for token in ('cube', 'block')
        )
        if role == 'marker' and looks_like_marker:
            result.append(prim)
        elif role == 'cube' and looks_like_cube and not looks_like_marker:
            result.append(prim)
    return result


def create_colored_cube(stage, path, position, size, color, physical):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.XformCommonAPI(cube.GetPrim())
    xform.SetTranslate(Gf.Vec3d(*position))
    xform.SetScale(Gf.Vec3f(*size))
    if physical:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
        mass_api.CreateMassAttr(0.05)
    return cube.GetPrim()


def create_mission_prims(stage):
    UsdGeom.Xform.Define(stage, MISSION_ROOT)
    created = {
        'blue_cube': create_colored_cube(
            stage, GENERATED_PRIMS['blue_cube'],
            (0.22, -0.18, 0.025), (0.05, 0.05, 0.05),
            (0.03, 0.12, 0.95), True,
        ),
        'green_cube': create_colored_cube(
            stage, GENERATED_PRIMS['green_cube'],
            (0.22, 0.18, 0.025), (0.05, 0.05, 0.05),
            (0.03, 0.80, 0.12), True,
        ),
        'blue_marker': create_colored_cube(
            stage, GENERATED_PRIMS['blue_marker'],
            (0.48, -0.16, 0.003), (0.12, 0.12, 0.006),
            (0.03, 0.12, 0.95), False,
        ),
        'green_marker': create_colored_cube(
            stage, GENERATED_PRIMS['green_marker'],
            (0.48, 0.16, 0.003), (0.12, 0.12, 0.006),
            (0.03, 0.80, 0.12), False,
        ),
    }
    print('   USD에 과제용 prim 4개가 없어 명시적으로 생성했습니다:')
    for role, prim in created.items():
        print(f'      {role:12s} {prim_path(prim)}')
    return created


def resolve_mission_prims(stage):
    roles = {
        'blue_cube': role_candidates(stage, 'blue', 'cube'),
        'green_cube': role_candidates(stage, 'green', 'cube'),
        'blue_marker': role_candidates(stage, 'blue', 'marker'),
        'green_marker': role_candidates(stage, 'green', 'marker'),
    }

    if all(len(candidates) == 0 for candidates in roles.values()):
        print_prim_candidates(stage, '기존 색상 과제 prim 탐색 결과')
        return create_mission_prims(stage)

    errors = []
    resolved = {}
    for role, candidates in roles.items():
        if len(candidates) == 1:
            resolved[role] = candidates[0]
        else:
            paths = [prim_path(candidate) for candidate in candidates]
            errors.append(f'{role}: {paths or "없음"}')
    if errors:
        print_prim_candidates(stage, '불완전하거나 모호한 색상 과제 prim')
        raise RuntimeError(
            '기존 mission prim을 안전하게 확정할 수 없습니다: ' +
            '; '.join(errors)
        )

    print('   기존 USD mission prim을 사용합니다:')
    for role, prim in resolved.items():
        print(f'      {role:12s} {prim_path(prim)}')
    return resolved


def world_position(prim):
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    translation = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array(translation, dtype=float)


def set_world_position(prim, position):
    parent = prim.GetParent()
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    parent_to_world = cache.GetLocalToWorldTransform(parent)
    world_to_parent = parent_to_world.GetInverse()
    local = world_to_parent.Transform(Gf.Vec3d(*position))
    if not UsdGeom.XformCommonAPI(prim).SetTranslate(local):
        raise RuntimeError(f'prim 위치를 설정할 수 없습니다: {prim_path(prim)}')


def world_bounds(prim):
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
    )
    aligned = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    minimum = np.array(aligned.GetMin(), dtype=float)
    maximum = np.array(aligned.GetMax(), dtype=float)
    if not np.all(np.isfinite(minimum)) or not np.all(np.isfinite(maximum)):
        raise RuntimeError(f'유효한 world bounds가 없습니다: {prim_path(prim)}')
    return minimum, maximum


def disable_legacy_red_blocks(stage):
    """Keep the old tutorial red block out of this blue/green mission."""
    disabled = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        normalized = normalize_name(prim.GetName())
        if 'redblock' not in normalized and 'redcube' not in normalized:
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable:
            imageable.MakeInvisible()
        for descendant in Usd.PrimRange(prim):
            collision = UsdPhysics.CollisionAPI.Get(descendant)
            if collision:
                collision.GetCollisionEnabledAttr().Set(False)
            rigid_body = UsdPhysics.RigidBodyAPI.Get(descendant)
            if rigid_body:
                rigid_body.GetRigidBodyEnabledAttr().Set(False)
        disabled.append(prim_path(prim))
    for path in disabled:
        print(f'   legacy block  disabled {path}')


def configure_random_mission(stage, prims):
    disable_legacy_red_blocks(stage)
    rng = np.random.default_rng()
    selected_color = str(rng.choice(['blue', 'green']))
    selected_key = f'{selected_color}_cube'
    other_key = 'green_cube' if selected_color == 'blue' else 'blue_cube'

    selected = prims[selected_key]
    original_position = world_position(selected)
    pick_xy = np.array([
        rng.uniform(*PICK_X_RANGE),
        rng.uniform(*PICK_Y_RANGE),
    ])
    set_world_position(
        selected,
        np.array([pick_xy[0], pick_xy[1], original_position[2]]),
    )
    UsdGeom.Imageable(selected).MakeVisible()
    UsdGeom.Imageable(prims[other_key]).MakeInvisible()

    _, selected_max = world_bounds(selected)
    selected_min, _ = world_bounds(selected)
    cube_height = float(selected_max[2] - selected_min[2])
    pick_z = float(selected_max[2])

    place_xy = {}
    place_z = {}
    for color in ('blue', 'green'):
        marker = prims[f'{color}_marker']
        marker_position = world_position(marker)
        _, marker_max = world_bounds(marker)
        place_xy[color] = marker_position[:2]
        place_z[color] = float(marker_max[2] + cube_height)

    print(f'   selected     {selected_color.upper()} (debug truth only)')
    print(f'   selected prim {prim_path(selected)}')
    print(f'   random pick  {vec(pick_xy)}  tcp_z={pick_z:.3f}')
    print(f'   blue place   {vec(place_xy["blue"])}')
    print(f'   green place  {vec(place_xy["green"])}')

    return {
        'selected_color': selected_color,
        'pick_xy': pick_xy,
        'pick_z': pick_z,
        'place_xy': place_xy,
        'place_z': place_z,
    }


def find_prim_path(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim_path(prim)
    return None


class M0609Task(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        self.camera_path = None
        self.mission = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        stage = omni.usd.get_context().get_stage()
        self.camera_path = find_wrist_camera_path(stage)
        mission_prims = resolve_mission_prims(stage)
        self.mission = configure_random_mission(stage, mission_prims)
        self._setup_arm_drives(stage)
        self._register_robot(scene)
        print(f'   wrist camera {self.camera_path}')
        print('   scene         ready')

    @staticmethod
    def _load_usd():
        if not Path(USD_PATH).is_file():
            raise FileNotFoundError(f'USD가 없습니다: {USD_PATH}')
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath('/World')
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, '/World').GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f'   USD           {USD_PATH}')

    @staticmethod
    def _setup_arm_drives(stage):
        robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        if not robot_prim.IsValid():
            print_prim_candidates(stage, '로봇 prim 탐색 결과')
            raise RuntimeError(f'로봇 prim이 없습니다: {ROBOT_PRIM_PATH}')
        count = 0
        for prim in Usd.PrimRange(robot_prim):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ('angular', 'linear'):
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    count += 1
        if count < len(ARM_JOINTS):
            raise RuntimeError(
                f'팔 관절 Drive가 부족합니다: {count}/{len(ARM_JOINTS)}'
            )
        print(f'   arm drives    {count}')

    def _register_robot(self, scene):
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(
                f'{ROBOT_PRIM_PATH} 아래에 {EE_LINK_NAME}이 없습니다.'
            )
        gripper = ParallelGripper(
            end_effector_prim_path=ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )
        self._robot = scene.add(SingleManipulator(
            prim_path=ROBOT_PRIM_PATH,
            name='m0609_robot',
            end_effector_prim_path=ee_path,
            gripper=gripper,
        ))
        print(f'   EE frame      {ee_path}')

    @property
    def robot(self):
        return self._robot


def init_gripper(robot, world):
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    joints = np.zeros(robot.num_dof)
    joints[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(joints)


def create_ik_solver(robot):
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_robot_base_pose(
        robot_position=ROBOT_BASE_POS,
        robot_orientation=ROBOT_BASE_QUAT,
    )
    print(f'   controlled    {", ".join(lula.get_joint_names())}')
    return ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )


class PcARosNode(Node):
    """Publish rgb8 frames and keep the newest color_id with arrival time."""

    def __init__(self):
        super().__init__('m0609_pick_place_pc_a')
        self.image_publisher = self.create_publisher(
            Image, IMAGE_TOPIC, qos_profile_sensor_data
        )
        self.color_subscription = self.create_subscription(
            Int32, COLOR_TOPIC, self._color_callback, 10
        )
        self.color_id = 0
        self.color_received_at = None
        self._last_reported_id = None

    def _color_callback(self, message):
        if message.data not in (0, 1, 2):
            self.get_logger().warning(
                f'범위 밖 color_id 무시: {message.data}'
            )
            return
        self.color_id = int(message.data)
        self.color_received_at = time.monotonic()
        if self.color_id != self._last_reported_id:
            self.get_logger().info(f'color_id 수신: {self.color_id}')
            self._last_reported_id = self.color_id

    def clear_color(self):
        self.color_id = 0
        self.color_received_at = None

    def fresh_color(self):
        if self.color_id not in (1, 2) or self.color_received_at is None:
            return None
        age = time.monotonic() - self.color_received_at
        if age > COLOR_MAX_AGE_SEC:
            return None
        return self.color_id


class WristImagePublisher:
    def __init__(self, camera_path, ros_node):
        self._ros_node = ros_node
        self._camera = Camera(
            prim_path=camera_path,
            name='m0609_wrist_camera',
            frequency=CAMERA_FREQUENCY,
            resolution=CAMERA_RESOLUTION,
        )
        self._camera.initialize()
        self._camera.add_rgb_to_frame()
        self._frame_id = camera_path.rsplit('/', 1)[-1]
        self._period = 1.0 / CAMERA_FREQUENCY
        self._last_publish = 0.0
        self._last_warning = 0.0

    def publish_if_due(self):
        now = time.monotonic()
        if now - self._last_publish < self._period:
            return
        rgb = self._camera.get_rgb()
        if rgb is None:
            if now - self._last_warning >= 1.0:
                self._ros_node.get_logger().warning(
                    'Wrist Camera RGB frame을 아직 받지 못했습니다.'
                )
                self._last_warning = now
            return

        rgb = np.asarray(rgb)
        if rgb.ndim != 3 or rgb.shape[2] < 3:
            raise RuntimeError(f'예상하지 못한 RGB shape: {rgb.shape}')
        rgb = rgb[:, :, :3]
        if rgb.dtype != np.uint8:
            if np.issubdtype(rgb.dtype, np.floating):
                rgb = np.clip(rgb * 255.0, 0, 255).astype(np.uint8)
            else:
                rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        rgb = np.ascontiguousarray(rgb)

        message = Image()
        message.header.stamp = self._ros_node.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.height = int(rgb.shape[0])
        message.width = int(rgb.shape[1])
        message.encoding = 'rgb8'
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = rgb.tobytes()
        self._ros_node.image_publisher.publish(message)
        self._last_publish = now


class PickPlaceFSM:
    NAMES = [
        'WAIT_COLOR', 'APPROACH', 'DESCEND', 'GRASP', 'LIFT',
        'MOVE', 'LOWER', 'RELEASE', 'DONE',
    ]
    WAIT_COLOR = 0
    APPROACH = 1
    DESCEND = 2
    GRASP = 3
    LIFT = 4
    MOVE = 5
    LOWER = 6
    RELEASE = 7
    DONE = 8
    GRIPPER_STATES = {GRASP: 'close', RELEASE: 'open'}

    def __init__(self, robot, mission):
        self._robot = robot
        self._mission = mission
        self.state = self.WAIT_COLOR
        self.step = 0
        self.start = None
        self.goal = None
        self.n_steps = 0
        self.gripper = 'open'
        self.place_color = None
        self.waypoints = {}
        print('   FSM           WAIT_COLOR')

    @property
    def done(self):
        return self.state == self.DONE

    @property
    def waiting(self):
        return self.state == self.WAIT_COLOR

    @property
    def is_gripper_state(self):
        return self.state in self.GRIPPER_STATES

    def accept_color(self, color_id):
        if not self.waiting or color_id not in (1, 2):
            return False
        # Place selection is based only on the received PC B result.
        self.place_color = 'blue' if color_id == 1 else 'green'
        debug_truth = self._mission['selected_color']
        print(f'   color result  {color_id} -> {self.place_color.upper()} place')
        print(f'   debug truth   {debug_truth.upper()} '
              f'({"MATCH" if debug_truth == self.place_color else "MISMATCH"})')

        pick_x, pick_y = self._mission['pick_xy']
        place_x, place_y = self._mission['place_xy'][self.place_color]
        pick_z = self._mission['pick_z']
        place_z = self._mission['place_z'][self.place_color]
        self.waypoints = {
            self.APPROACH: np.array([pick_x, pick_y, APPROACH_HEIGHT]),
            self.DESCEND: np.array([pick_x, pick_y, pick_z]),
            self.GRASP: np.array([pick_x, pick_y, pick_z]),
            self.LIFT: np.array([pick_x, pick_y, LIFT_HEIGHT]),
            self.MOVE: np.array([place_x, place_y, LIFT_HEIGHT]),
            self.LOWER: np.array([place_x, place_y, place_z]),
            self.RELEASE: np.array([place_x, place_y, place_z]),
        }
        print(f'   place target  {vec(self._mission["place_xy"][self.place_color])}')
        self._enter(self.APPROACH)
        return True

    def _enter(self, state):
        self.state = state
        self.step = 0
        self.start = None
        self.goal = None
        if self.state == self.DONE:
            print('   FSM           DONE (재실행하지 않음)')
            return

        self.start = get_tcp_pose(self._robot)
        self.goal = self.waypoints[self.state]
        self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)
        if self.is_gripper_state:
            self.n_steps = GRIPPER_WAIT
            distance = 0.0
        else:
            self.n_steps, distance = steps_for(self.start, self.goal)
        print(f'   FSM           {self.NAMES[self.state]:9s} '
              f'goal={vec(self.goal)} distance={distance:.4f} '
              f'steps={self.n_steps} gripper={self.gripper}')

    def current_target(self):
        if self.waiting or self.done:
            return None
        alpha = min(1.0, self.step / float(self.n_steps))
        return self.start + alpha * (self.goal - self.start)

    def advance(self):
        if self.waiting or self.done:
            return
        self.step += 1
        if self.step >= self.n_steps:
            self._enter(self.state + 1)


def print_status(robot, solved, fsm, target_tcp):
    state_name = fsm.NAMES[fsm.state]
    if not solved:
        print(f'   {state_name:9s} IK FAILED target={vec(target_tcp)}')
        return
    tcp = get_tcp_pose(robot)
    finger = robot.get_joint_positions()[
        robot.get_dof_index('finger_joint')
    ]
    print(f'   {state_name:9s} tcp={vec(tcp)} finger={finger:+.4f}')


def main():
    ros_node = None
    try:
        rclpy.init()
        ros_node = PcARosNode()
        world = World(stage_units_in_meters=1.0)

        section('SCENE')
        task = M0609Task(name='m0609_color_pick_place_task')
        world.add_task(task)
        world.reset()

        robot = task.robot
        robot.initialize()
        init_gripper(robot, world)
        set_ready_pose(robot)
        for _ in range(30):
            world.step(render=True)

        section('SOLVER')
        ik_solver = create_ik_solver(robot)
        target_quat = make_target_quat(
            APPROACH_ROLL_DEG,
            APPROACH_PITCH_DEG,
            GRIPPER_YAW_DEG,
        )

        section('ROS 2')
        image_publisher = WristImagePublisher(task.camera_path, ros_node)
        print(f'   publish       {IMAGE_TOPIC} sensor_msgs/Image rgb8')
        print(f'   subscribe     {COLOR_TOPIC} std_msgs/Int32')
        print('   press Play in the viewport')

        fsm = PickPlaceFSM(robot, task.mission)
        was_playing = False
        step = 0

        while simulation_app.is_running():
            world.step(render=True)
            rclpy.spin_once(ros_node, timeout_sec=0.0)
            time.sleep(0.005)
            is_playing = world.is_playing()

            if is_playing and not was_playing:
                if fsm.done:
                    print('   DONE 상태이므로 새 mission을 시작하지 않습니다.')
                else:
                    world.reset()
                    robot.initialize()
                    init_gripper(robot, world)
                    set_ready_pose(robot)
                    ros_node.clear_color()
                    fsm = PickPlaceFSM(robot, task.mission)
                    step = 0

            if is_playing:
                image_publisher.publish_if_due()

                if fsm.waiting:
                    fresh_color = ros_node.fresh_color()
                    if fresh_color is not None:
                        fsm.accept_color(fresh_color)

                target_tcp = fsm.current_target()
                if target_tcp is not None:
                    flange_target = tcp_to_flange(target_tcp, target_quat)
                    action, solved = ik_solver.compute_inverse_kinematics(
                        target_position=flange_target,
                        target_orientation=target_quat,
                    )
                    if solved:
                        robot.apply_action(action)

                    robot.apply_action(
                        robot.gripper.forward(action=fsm.gripper)
                    )

                    # Do not advance a motion state through an IK failure.
                    if solved or fsm.is_gripper_state:
                        fsm.advance()

                    if step % LOG_INTERVAL == 0:
                        print_status(robot, solved, fsm, target_tcp)
                    step += 1

            was_playing = is_playing

    finally:
        if ros_node is not None:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == '__main__':
    main()
