import typing as ty
import json
import rclpy
from mavros_msgs.msg import Waypoint, WaypointList, WaypointReached, State
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from enum import Enum


class MissionState(Enum):
    PRIMARY = 1
    SECONDARY = 2
    THIRD = 3
    DONE = 4


class SwitchStep(Enum):
    IDLE = 0
    ENSURE_LOITER = 1
    CLEAR = 2
    PUSH = 3
    PULL = 4
    SET_CURRENT = 5
    ENSURE_AUTO = 6
    DONE = 7


class ProjectPartTwo(Node):

    def __init__(self):
        super().__init__('project_part_two')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.waypoints_sub = self.create_subscription(
            WaypointList, '/mavros/mission/waypoints',
            self.waypoints_callback, qos_profile,
        )

        self.reached_sub = self.create_subscription(
            WaypointReached, '/mavros/mission/reached',
            self.waypoint_reached_callback, qos_profile,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix, 'mavros/global_position/global',
            self.global_position_callback, sensor_qos_profile,
        )

        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_callback, qos_profile,
        )

        self.clear_client = self.create_client(WaypointClear, 'mavros/mission/clear')
        self.push_client = self.create_client(WaypointPush, 'mavros/mission/push')
        self.pull_client = self.create_client(WaypointPull, 'mavros/mission/pull')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_current_client = self.create_client(WaypointSetCurrent, 'mavros/mission/set_current')

        self.primary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_A.waypoints"
        self.secondary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_B.waypoints"
        self.third_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_C.waypoints"

        self.current_global_position = None
        self.current_state = None
        self.current_fc_waypoints = []
        self.primary_waypoints = []
        self.secondary_waypoints = []
        self.third_waypoints = []

        self.last_reached_wp = None
        self.mission_state = None

        self.pending_new_waypoints: ty.List[Waypoint] = []
        self.switch_step = SwitchStep.IDLE
        self.switch_retry_count = 0
        self.switch_timer = self.create_timer(1.0, self.switch_timer_callback)

        self.wait_for_services()

    # ----- 回调函数 -----
    def waypoints_callback(self, msg: WaypointList):
        self.current_fc_waypoints = msg.waypoints

    def waypoint_reached_callback(self, msg: WaypointReached):
        self.last_reached_wp = msg.wp_seq
        fc_count = len(self.current_fc_waypoints)
        current_mode = self.current_state.mode if self.current_state else '?'
        self.get_logger().info(
            f'航点到达: wp={msg.wp_seq}, FC航点数={fc_count}, '
            f'状态={self.mission_state}, 飞控模式={current_mode}'
        )

        if not self.current_fc_waypoints or self.mission_state == MissionState.DONE:
            return

        if self.switch_step != SwitchStep.IDLE:
            self.get_logger().info(f'航线切换进行中 (步骤={self.switch_step}), 忽略此次到达')
            return

        complete_wp = fc_count - 2

        if self.mission_state == MissionState.PRIMARY and self.last_reached_wp == complete_wp:
            self.get_logger().info('第一航线执行完毕, 请求切换第二航线')
            self.mission_state = MissionState.SECONDARY
            self.start_switch(self.secondary_waypoints)

        if self.mission_state == MissionState.SECONDARY and self.last_reached_wp == complete_wp:
            self.get_logger().info('第二航线执行完毕, 请求切换第三航线')
            self.mission_state = MissionState.THIRD
            self.start_switch(self.third_waypoints)

        if self.mission_state == MissionState.THIRD and self.last_reached_wp == complete_wp:
            self.get_logger().info('第三航线执行完毕')
            self.mission_state = MissionState.DONE

    def global_position_callback(self, msg: NavSatFix):
        self.current_global_position = (msg.latitude, msg.longitude, msg.altitude)

    def state_callback(self, msg: State):
        self.current_state = msg

    # ----- 服务就绪 -----
    def wait_for_services(self):
        self.get_logger().info('等待 MAVROS 服务就绪...')
        for client, name in [
            (self.push_client, 'WaypointPush'),
            (self.pull_client, 'WaypointPull'),
            (self.clear_client, 'WaypointClear'),
            (self.set_current_client, 'WaypointSetCurrent'),
            (self.set_mode_client, 'SetMode'),
        ]:
            while not client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning(f'等待 {name} ...')
        self.get_logger().info('所有服务就绪')

    # ----- 基础服务操作 -----
    def clear_mission(self) -> bool:
        req = WaypointClear.Request()
        future = self.clear_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().success:
            self.get_logger().info('清空航线成功')
            return True
        self.get_logger().error('清空航线失败')
        return False

    def push_mission(self, waypoints: ty.List[Waypoint]) -> bool:
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints
        future = self.push_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'推送成功, {future.result().wp_transfered} 个航点')
            return True
        self.get_logger().error('推送航线失败')
        return False

    def pull_mission(self) -> bool:
        req = WaypointPull.Request()
        future = self.pull_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'拉取航线成功, {future.result().wp_received} 个航点')
            return True
        self.get_logger().error('拉取航线失败')
        return False

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'模式切换成功: {mode}')
            return True
        self.get_logger().warn(f'模式切换未确认: {mode} (可能已在该模式)')
        return False

    def set_current_waypoint(self, wp_seq: int) -> bool:
        req = WaypointSetCurrent.Request()
        req.wp_seq = wp_seq
        future = self.set_current_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'当前航点设为: {wp_seq}')
            return True
        self.get_logger().error(f'设置当前航点失败: {wp_seq}')
        return False

    # ----- 异步航线切换 (定时器驱动, 避免回调内嵌套spin) -----
    def start_switch(self, new_waypoints: ty.List[Waypoint]):
        current_mode = self.current_state.mode if self.current_state else '?'
        self.get_logger().info(f'===== 请求切换航线 (当前飞控模式: {current_mode}) =====')
        self.pending_new_waypoints = new_waypoints
        self.switch_step = SwitchStep.ENSURE_LOITER
        self.switch_retry_count = 0

    def switch_timer_callback(self):
        if self.switch_step == SwitchStep.IDLE:
            return

        self.get_logger().info(f'切换步骤: {self.switch_step}, 重试: {self.switch_retry_count}')

        if self.switch_step == SwitchStep.ENSURE_LOITER:
            if self.current_state is not None and self.current_state.mode == 'AUTO.LOITER':
                self.switch_step = SwitchStep.CLEAR
                self.switch_retry_count = 0
                return
            self.set_mode('AUTO.LOITER')
            if self.switch_retry_count < 5:
                self.switch_retry_count += 1
            else:
                self.get_logger().error('LOITER 模式切换失败, 强制继续')
                self.switch_step = SwitchStep.CLEAR
                self.switch_retry_count = 0

        elif self.switch_step == SwitchStep.CLEAR:
            if self.clear_mission():
                self.switch_step = SwitchStep.PUSH
                self.switch_retry_count = 0
            elif self.switch_retry_count < 3:
                self.switch_retry_count += 1
            else:
                self.get_logger().error('清空航线失败, 强制继续')
                self.switch_step = SwitchStep.PUSH
                self.switch_retry_count = 0

        elif self.switch_step == SwitchStep.PUSH:
            if self.push_mission(self.pending_new_waypoints):
                self.switch_step = SwitchStep.PULL
                self.switch_retry_count = 0
            elif self.switch_retry_count < 3:
                self.switch_retry_count += 1
            else:
                self.get_logger().error('推送航线失败, 强制继续')
                self.switch_step = SwitchStep.PULL
                self.switch_retry_count = 0

        elif self.switch_step == SwitchStep.PULL:
            if self.pull_mission():
                self.switch_step = SwitchStep.SET_CURRENT
                self.switch_retry_count = 0
            elif self.switch_retry_count < 3:
                self.switch_retry_count += 1
            else:
                self.get_logger().warn('拉取航线失败, 继续')
                self.switch_step = SwitchStep.SET_CURRENT
                self.switch_retry_count = 0

        elif self.switch_step == SwitchStep.SET_CURRENT:
            if self.set_current_waypoint(0):
                self.switch_step = SwitchStep.ENSURE_AUTO
                self.switch_retry_count = 0
            elif self.switch_retry_count < 3:
                self.switch_retry_count += 1
            else:
                self.get_logger().warn('设置当前航点失败, 继续')
                self.switch_step = SwitchStep.ENSURE_AUTO
                self.switch_retry_count = 0

        elif self.switch_step == SwitchStep.ENSURE_AUTO:
            if self.current_state is not None and self.current_state.mode == 'AUTO.MISSION':
                self.switch_step = SwitchStep.IDLE
                self.switch_retry_count = 0
                self.pending_new_waypoints = []
                self.get_logger().info('===== 航线切换完成 =====')
                return
            self.set_mode('AUTO.MISSION')
            if self.switch_retry_count < 5:
                self.switch_retry_count += 1
            else:
                self.get_logger().warn('AUTO.MISSION 模式切换未确认, 但切换流程结束')
                self.switch_step = SwitchStep.IDLE
                self.switch_retry_count = 0
                self.pending_new_waypoints = []

    # ----- 航点文件读取 -----
    def read_waypoints_from_file(self, file_path: str) -> ty.List[Waypoint]:
        waypoints = []
        with open(file_path, 'r') as f:
            lines = f.readlines()
        for line in lines[1:]:
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) < 12:
                self.get_logger().warning(f'跳过无效行: {stripped}')
                continue

            seq = int(parts[0])
            is_current = int(parts[1]) == 1
            frame = int(parts[2])
            command = int(parts[3])
            param1 = float(parts[4])
            param2 = float(parts[5])
            param3 = float(parts[6])
            param4 = float(parts[7])
            lat = float(parts[8])
            lon = float(parts[9])
            alt = float(parts[10])
            autocontinue = int(parts[11]) == 1

            if seq == 0 and lat == 0.0 and lon == 0.0 and alt == 0.0:
                if self.current_global_position is not None:
                    lat, lon, alt = self.current_global_position
                    command = 16
                    frame = 0
                    self.get_logger().info(f'空家点替换为 GPS: ({lat:.7f}, {lon:.7f})')
                else:
                    self.get_logger().warning('空家点 GPS 未锁定, 跳过')
                    continue

            wp = Waypoint()
            wp.frame = frame
            wp.command = command
            wp.is_current = is_current
            wp.autocontinue = autocontinue
            wp.param1 = param1
            wp.param2 = param2
            wp.param3 = param3
            wp.param4 = param4
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            waypoints.append(wp)

        self.get_logger().info(f'从 {file_path} 加载了 {len(waypoints)} 个航点')
        return waypoints

    # ----- 辅助函数 -----
    def print_waypoint_info(self, waypoints: ty.List[Waypoint]):
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'航点总数: {len(waypoints)}')
        self.get_logger().info('=' * 60)
        for i, wp in enumerate(waypoints):
            cmd = {16: 'WAYPOINT', 17: 'LOITER_UNLIM', 19: 'LOITER_TIME',
                   21: 'LAND', 22: 'TAKEOFF', 84: 'VTOL_TKO', 85: 'VTOL_LAND'}.get(wp.command, str(wp.command))
            self.get_logger().info(
                f'[{i}] {cmd} | Lat={wp.x_lat:.6f} Lon={wp.y_long:.6f} Alt={wp.z_alt:.1f}m')
        self.get_logger().info('=' * 60)

    # ----- 主函数 -----
    def run(self):
        self.primary_waypoints = self.read_waypoints_from_file(self.primary_path)
        self.print_waypoint_info(self.primary_waypoints)

        self.secondary_waypoints = self.read_waypoints_from_file(self.secondary_path)
        self.print_waypoint_info(self.secondary_waypoints)

        self.third_waypoints = self.read_waypoints_from_file(self.third_path)
        self.print_waypoint_info(self.third_waypoints)

        self.mission_state = MissionState.PRIMARY
        self.clear_mission()
        self.push_mission(self.primary_waypoints)
        self.pull_mission()
        self.set_current_waypoint(0)
        self.set_mode('AUTO.MISSION')

        self.get_logger().info('第一航线已启动, 等待航点到达...')


def main(args=None):
    rclpy.init(args=args)
    node = ProjectPartTwo()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()
