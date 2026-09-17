import numpy as np
import omni.usd

from pxr import Gf, UsdGeom


class WaypointMover:
    """
    Isaac Sim Timeline/World의 physics step에 맞춰 한 프레임씩 이동시키는 컨트롤러.

    중요:
    - 이 클래스 안에서는 simulation_app.update(), world.step(), sleep()을 호출하지 않는다.
    - 외부 메인 루프가 world.step(render=True)를 1회 수행한 뒤,
      Timeline이 Playing일 때만 step(dt)를 호출해야 한다.
    - 따라서 Pause/Stop 중에는 이동이 진행되지 않는다.
    - Pick & Place FSM과 같은 메인 루프에 연결할 수 있다.
    """

    def __init__(
        self,
        stage,
        prim_path: str,
        waypoints,
        speed_mps: float = 1.0,
        tolerance_m: float = 0.005,
    ):
        self.stage = stage
        self.prim_path = prim_path
        self.prim = stage.GetPrimAtPath(prim_path)

        if not self.prim.IsValid():
            raise RuntimeError(f"Move root does not exist: {prim_path}")

        self.waypoints = [
            np.asarray(point, dtype=float).copy()
            for point in waypoints
        ]

        if len(self.waypoints) < 2:
            raise ValueError("Waypoints need at least W0 and W1.")

        self.speed_mps = float(speed_mps)
        self.tolerance_m = float(tolerance_m)

        self.target_index = 1
        self.done = False

    def _world_to_local_position(self, world_position: np.ndarray):
        parent = self.prim.GetParent()
        parent_world = omni.usd.get_world_transform_matrix(parent)
        parent_world_inverse = parent_world.GetInverse()

        world_vec = Gf.Vec3d(
            float(world_position[0]),
            float(world_position[1]),
            float(world_position[2]),
        )
        return parent_world_inverse.Transform(world_vec)

    def _set_world_position(self, world_position: np.ndarray) -> None:
        """
        xformOp:orient를 건드리지 않고 translation만 변경한다.
        현재 robot Xform의 기존 회전/스케일 표현은 그대로 유지한다.
        """
        local_position = self._world_to_local_position(world_position)

        xformable = UsdGeom.Xformable(self.prim)
        ordered_ops = xformable.GetOrderedXformOps()

        # 기존 translate op가 있으면 그것만 변경
        for op in ordered_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                current_value = op.Get()

                if isinstance(current_value, Gf.Vec3f):
                    op.Set(
                        Gf.Vec3f(
                            float(local_position[0]),
                            float(local_position[1]),
                            float(local_position[2]),
                        )
                    )
                else:
                    op.Set(local_position)
                return

        # matrix transform으로 저장된 경우 translation만 변경
        for op in ordered_ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTransform:
                matrix = op.Get()
                matrix.SetTranslateOnly(local_position)
                op.Set(matrix)
                return

        # translate/transform op가 전혀 없으면 translate op 생성
        translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        translate_op.Set(local_position)

    def get_world_position(self) -> np.ndarray:
        matrix = omni.usd.get_world_transform_matrix(self.prim)
        translation = matrix.ExtractTranslation()

        return np.array(
            [translation[0], translation[1], translation[2]],
            dtype=float,
        )

    def reset_to_start(self) -> None:
        """
        시나리오 reset용.
        W0로 이동시키고 다음 목표를 W1로 되돌린다.
        """
        self._set_world_position(self.waypoints[0])
        self.target_index = 1
        self.done = False

    def step(self, dt: float) -> bool:
        """
        physics frame 1회분만 이동한다.

        Returns:
            True  : 마지막 waypoint 도착 완료
            False : 아직 이동 중
        """
        if self.done:
            return True

        if dt <= 0.0:
            return False

        current = self.get_world_position()
        target = self.waypoints[self.target_index]

        offset = target - current
        distance = float(np.linalg.norm(offset))

        if distance <= self.tolerance_m:
            self._set_world_position(target)
            print(f"[REACHED] W{self.target_index}: {target}")

            self.target_index += 1

            if self.target_index >= len(self.waypoints):
                self.done = True
                print("[MOVE DONE] Waypoint route complete.")
                return True

            return False

        travel = min(self.speed_mps * dt, distance)
        new_position = current + (offset / distance) * travel
        self._set_world_position(new_position)

        return False
