2026-09-23 백업. "Nav2 가 yaw 까지 맞추고 도킹은 직진만" 으로 되돌리기 직전 상태다.

여기 담긴 구성 (되돌리기 전):
  - params  general_goal_checker.yaw_goal_tolerance = 3.15 (Nav2 가 최종 방향을 안 봄)
  - script  ARRIVE_M 0.5 로 Nav2 를 조기 취소하고 스크립트가 도킹
  - script  do_dock 4단계: DOCK-SNAP(경유지 보정) -> DOCK-TURN -> DOCK-DRIVE -> DOCK-REALIGN

폐기한 이유 (실측 로그, 2026-09-23):
  DOCK-SNAP 으로 횡오차를 0.009 m 까지 맞춘 직후, DOCK-TURN 의 105도 제자리 회전만으로
  base_link 가 서 0.083 m / 남 0.203 m, 합 0.22 m 이동했다. 역산한 회전 반경은 0.138 m —
  base_link 가 실제 회전 중심(구동륜 축)에서 14 cm 떨어져 있다는 뜻이다.
  즉 **위치를 맞춘 뒤 회전하면 그 회전이 위치를 망친다.** 순서 문제라 4단계 구성 자체가 성립 안 한다.

이 수치는 남겨둘 것. 다시 스크립트 회전을 쓸 일이 있으면 회전 전후 위치 변화를 반드시 고려해야 한다.
uncommitted.patch 는 백업 시점의 git diff 다 (git apply 로 복원).
