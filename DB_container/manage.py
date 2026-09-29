"""
manage.py — 병원 검체 운송 AMR 관리 노드 (1단계)

기능
  1) PostgreSQL / Redis 가 on 상태인지 확인 (하나라도 off 면 종료)
  2) robot_info 에 로봇이 없으면 최초 1회 insert (is_active 는 DB 기본값 FALSE)
  3) 로봇 위치 토픽이 처음 수신되면 = 로봇 on → is_active = TRUE
  4) 위치 토픽 → Redis robot:{id}:state 갱신, odom 토픽 → robot:{id}:heartbeat 갱신
  5) print_period(기본 3초) 간격으로 Redis 의 state 를 그대로 출력
  종료(Ctrl+C) 시 is_active = FALSE 로 되돌림

실행 (launch.py 를 띄운 상태에서)
  python3 manage.py
  python3 manage.py --ros-args -p robot_name:=AMR-01 -p pose_topic:=/amcl_pose -p pose_type:=amcl
  python3 manage.py --ros-args -p pose_topic:=/chassis/odom -p pose_type:=odom
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from hospital_system import db   # colcon build 후 install/setup.bash 를 source


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotManager(Node):
    def __init__(self):
        super().__init__("hospital_robot_manager")

        self.declare_parameter("robot_name", "AMR-01")
        self.declare_parameter("model", "nova_carter")
        self.declare_parameter("container_slots", 3)
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("pose_type", "amcl")        # amcl | odom
        self.declare_parameter("heartbeat_topic", "/chassis/odom")  # "" 이면 사용 안 함
        self.declare_parameter("print_period", 3.0)

        p = lambda name: self.get_parameter(name).value
        self.log = self.get_logger()
        self.active = False

        # 1) DB on 확인
        if not self.check_db():
            raise RuntimeError("DB 가 off 상태입니다.")

        # 2) 로봇 등록 (최초 1회)
        self.robot_id = self.ensure_robot(p("robot_name"), p("model"), int(p("container_slots")))

        # 3~4) 위치 토픽 구독
        pose_topic, pose_type = p("pose_topic"), p("pose_type")
        if pose_type == "amcl":
            # AMCL 은 reliable + transient_local 로 발행 → 구독 즉시 마지막 위치도 받음
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(PoseWithCovarianceStamped, pose_topic,
                                     lambda m: self.on_pose(m.pose.pose), qos)
        elif pose_type == "odom":
            self.create_subscription(Odometry, pose_topic,
                                     lambda m: self.on_pose(m.pose.pose), 10)
        else:
            raise ValueError("pose_type 은 'amcl' 또는 'odom' 이어야 합니다.")

        # heartbeat: /amcl_pose 는 로봇이 움직일 때만 발행되므로,
        # 정지 중에도 계속 들어오는 odom 으로 통신 상태(heartbeat)를 갱신한다.
        hb_topic = p("heartbeat_topic")
        if hb_topic and not (pose_type == "odom" and hb_topic == pose_topic):
            self.create_subscription(Odometry, hb_topic, lambda m: self.on_alive(), 10)
            self.log.info(f"[ROS] heartbeat 구독: {hb_topic}")

        # 5) 주기 출력
        period = float(p("print_period"))
        self.create_timer(period, self.print_state)
        self.log.info(f"[ROS] 위치 구독: {pose_topic} ({pose_type}), 출력 주기 {period:.0f}s")

    # ---------------- DB ----------------
    def check_db(self):
        status = db.db_check()
        hosts = {"postgres": db.PG_DSN.split("@")[-1], "redis": db.REDIS_URL.split("@")[-1]}
        for name, (on, err) in status.items():
            if on:
                self.log.info(f"[DB] {name} on ({hosts[name]})")
            else:
                self.log.error(f"[DB] {name} off: {err}")
        return all(on for on, _ in status.values())

    def ensure_robot(self, robot_name, model, container_slots):
        """robot_name 으로 등록된 로봇을 찾고, 없으면 insert. 반환: robot_id"""
        row = db.robot_info_get_by_name(robot_name)
        if row is not None:
            self.log.info(f"[DB] 기존 로봇: robot_id={row['robot_id']} "
                          f"name={robot_name} is_active={row['is_active']}")
            return row["robot_id"]
        rid = db.robot_info_insert(robot_name, model, container_slots)
        self.log.info(f"[DB] 로봇 최초 등록: robot_id={rid} name={robot_name} model={model}")
        return rid

    # ---------------- ROS 콜백 ----------------
    def activate(self):
        """첫 메시지 수신 = 로봇 on → is_active = TRUE (1회)"""
        if self.active:
            return
        try:
            db.robot_info_update(self.robot_id, is_active=True)
            self.active = True
            self.log.info(f"[DB] 로봇 on 감지 → robot_id={self.robot_id} is_active=TRUE")
        except Exception as e:
            self.log.error(f"[DB] is_active 갱신 실패: {e}")

    def on_alive(self):
        self.activate()
        try:
            db.robot_heartbeat(self.robot_id)
        except Exception as e:
            self.log.warn(f"[Redis] heartbeat 갱신 실패: {e}")

    def on_pose(self, pose):
        self.activate()
        try:
            db.robot_state(self.robot_id,
                           x=round(pose.position.x, 3),
                           y=round(pose.position.y, 3),
                           theta=round(yaw_from_quat(pose.orientation), 4))
            db.robot_heartbeat(self.robot_id)
        except Exception as e:
            self.log.warn(f"[Redis] state 갱신 실패: {e}")

    def print_state(self):
        try:
            st = db.robot_state_get(self.robot_id)
            alive = db.robot_is_alive(self.robot_id)
        except Exception as e:
            self.log.warn(f"[Redis] 조회 실패: {e}")
            return
        if st is None:
            self.log.info(f"[STATE] robot {self.robot_id}: 위치 미수신")
            return
        self.log.info(
            f"[STATE] robot {self.robot_id} | {st['status']} | "
            f"x={st['x']:.3f} y={st['y']:.3f} θ={math.degrees(st['theta']):.1f}° | "
            f"task={st['task_id']} | alive={alive} | updated_at={st['updated_at']}")

    def shutdown(self):
        if self.active:
            try:
                db.robot_info_update(self.robot_id, is_active=False)
                self.log.info(f"[DB] 종료 → robot_id={self.robot_id} is_active=FALSE")
            except Exception as e:
                self.log.warn(f"[DB] is_active 해제 실패: {e}")


def main():
    rclpy.init()
    node = None
    try:
        node = RobotManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"[manage] 종료: {e}")
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        db.close_all()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
