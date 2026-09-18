from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.timeline
import omni.usd

from isaacsim.core.api import World
from isaacsim.core.utils.stage import is_stage_loading, open_stage

# SimulationApp 생성 이후 import
from waypoint_mover import WaypointMover


USD_PATH = Path("/home/rokey/cobot3_ws/isaacpjt/assets/integration.usd")
MOVE_ROOT_PATH = "/World/robot"

WAYPOINTS = [
    np.array([0.0, 0.0, 0.0]),       # W0
    np.array([6.72354, 0.0, 0.0]),       # W1
    np.array([6.72354, 15.5, 0.0]),   # W2
    np.array([0, 15.5, 0.0]),   # W3
]

SPEED_MPS = 1.0


def main():
    if not USD_PATH.is_file():
        raise FileNotFoundError(f"USD file was not found: {USD_PATH}")

    if not open_stage(str(USD_PATH)):
        raise RuntimeError(f"Could not open USD: {USD_PATH}")

    while is_stage_loading():
        simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    world.reset()

    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()

    mover = WaypointMover(
        stage=stage,
        prim_path=MOVE_ROOT_PATH,
        waypoints=WAYPOINTS,
        speed_mps=SPEED_MPS,
    )

    # 처음 실행 시 W0를 시작 상태로 설정
    mover.reset_to_start()

    # standalone 실행 직후 자동으로 움직이지 않게 Stop 상태로 둔다.
    timeline.stop()
    simulation_app.update()

    print("[READY] Isaac Sim의 Play 버튼을 누르면 이동을 시작합니다.")
    print("[INFO] Pause: 현재 위치에서 일시정지 / Stop: 다음 Play에서 W0부터 재시작")

    reset_pending = False
    was_playing = False
    done_printed = False

    while simulation_app.is_running():
        # 전체 시스템에서 physics/render를 전진시키는 유일한 메인 step
        world.step(render=True)

        is_playing = timeline.is_playing()
        is_stopped = timeline.is_stopped()

        # Stop은 '시나리오 다시 시작'으로 처리
        if is_stopped:
            if was_playing:
                reset_pending = True
                print("[STOP] 다음 Play에서 이동 상태를 W0로 리셋합니다.")

            was_playing = False
            continue

        # Pause일 때는 아무 제어도 진행하지 않음
        if not is_playing:
            was_playing = False
            continue

        # Stop -> Play
        if reset_pending:
            mover.reset_to_start()
            reset_pending = False
            done_printed = False
            print("[RESET] W0부터 다시 시작합니다.")

        dt = world.get_physics_dt()

        # -----------------------------
        # FSM에서 이동 상태에 해당하는 부분
        # -----------------------------
        movement_done = mover.step(dt)

        if movement_done and not done_printed:
            print("[STATE] MOVE 완료 -> 여기에서 PICK 상태로 넘기면 됩니다.")
            done_printed = True

        was_playing = True


try:
    main()
finally:
    simulation_app.close()
