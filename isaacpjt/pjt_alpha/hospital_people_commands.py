"""Read loop start positions from our existing People command file."""
import math
from pathlib import Path

# GoTo 는 목표 0.25 m(final_target_distance) 안에 들어와야만 끝난다. 애니메이션이 가끔 그 밖에서
# 서 버리면 명령이 영영 안 끝나고 뒤의 Idle/GoTo/루프도 멈춘다 (2026-09-29 Kit 로그: Character_01,
# Character_02 가 GoTo 하나에 17~20분 걸린 채 종료). 걷는 중 STUCK_SECONDS 동안 STUCK_MOVE_M 도
# 못 움직이면 도착으로 처리해 NVIDIA 의 정상 종료 경로(감속 -> exit_command)를 태운다.
STUCK_SECONDS = 3.0
STUCK_MOVE_M = 0.05


def loop_origins(filename, expected=None):
    result = {}
    for line in Path(filename).read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) >= 5 and fields[1] == "GoTo":
            result[fields[0]] = tuple(float(value) for value in fields[2:5])
    if expected is None:
        expected = {"Upper_West", "Upper_East"} | {
            f"Lower_Cross_{i}" for i in range(1, 5)
        }
    if set(result) != expected:
        raise ValueError(f"Expected loop waypoints for {sorted(expected)}")
    return result


def unstick_goto(command, delta_time):
    """Command.walk 패치에서 원래 walk 뒤에 부른다. delta_time 은 시뮬 시간."""
    from omni.anim.people.scripts.utils import Utils

    target = command.navigation_manager.get_path_target_pos()
    if command.desired_walk_speed <= 0.0 or target is None:
        command._stuck_anchor = None
        return
    pos = Utils.get_character_pos(command.character)
    xy = (pos[0], pos[1])
    anchor = getattr(command, "_stuck_anchor", None)
    if anchor is None or math.dist(anchor, xy) > STUCK_MOVE_M:
        command._stuck_anchor, command._stuck_time = xy, 0.0
        return
    command._stuck_time += delta_time
    if command._stuck_time >= STUCK_SECONDS:
        print(f"[PEOPLE] {command.character_name} stuck at ({xy[0]:.2f}, {xy[1]:.2f}), "
              f"target ({target[0]:.2f}, {target[1]:.2f}), {math.dist(xy, (target[0], target[1])):.2f} m short "
              f"-> GoTo 종료 처리", flush=True)
        command.navigation_manager.clean_path_targets()
        command._stuck_anchor = None
