from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import time
import omni.usd
from pxr import Usd, UsdGeom

USD_PATH = str(Path(__file__).resolve().parent / "scenes/carter_warehouse_navigation.usd")

# /World prim 생성 후 USD Reference 연결
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
world_prim = stage.GetPrimAtPath("/World")
world_prim.GetReferences().AddReference(USD_PATH)

for _ in range(15):
    simulation_app.update()

print("\n[완료] 씬 로드 — Play 버튼을 눌러 ROS2 토픽 발행을 시작하세요")

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.016)

simulation_app.close()