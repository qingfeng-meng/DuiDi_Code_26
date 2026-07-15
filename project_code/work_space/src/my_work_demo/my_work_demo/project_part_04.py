import typing as ty
import json
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    Mission,
    MissionResult,
    DatamanRequest,
    DatamanResponse,
    VehicleCommand,
    VehicleStatus,
    VehicleGlobalPosition,
    ActionRequest,
)
from enum import Enum


class MissionState(Enum):
    PRIMARY = 1
    SECONDARY = 2
    THIRD = 3
    DONE = 4


class ProjectPartFour(Node):

    def __init__(self):
        super().__init__('project_part_four')

        # PX4 uORB QoS (BEST_EFFORT + TRANSIENT_LOCAL)
        uorb_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 创建话题发布者 (向飞控发送命令)
        #1. 上传航线 2. 发送 dataman 请求 3. 发送车辆指令
        self.mission_pub = self.create_publisher(
            Mission, '/fmu/in/mission', uorb_qos,
        )

        self.dataman_request_pub = self.create_publisher(
            DatamanRequest, '/fmu/in/dataman_request', uorb_qos,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', uorb_qos,
        )

        # 创建话题订阅者用于获取飞行器的当前信息
        #1. 获取任务执行结果 2. 获取当前状态信息 3. 获取当前GPS信息
        self.mission_result_sub = self.create_subscription(
            MissionResult,
            '/fmu/out/mission_result',
            self.mission_result_callback,
            uorb_qos,
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            uorb_qos,
        )

        self.vehicle_global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback,
            uorb_qos,
        )

        # 三段航点文件路径
        self.primary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_A.waypoints"
        self.secondary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_B.waypoints"
        self.third_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_C.waypoints"

        self.current_global_position = None              # 当前GPS坐标 (lat, lon, alt)

        self.current_nav_state = None                    # 当前导航状态 (NAVIGATION_STATE_*)

        self.current_mission_seq_current = None          # 当前任务序号
        self.current_mission_seq_total = None            # 任务总航点数量
        self.current_mission_finished = False            # 任务是否完成

        self.primary_waypoints: ty.List[dict] = []
        self.secondary_waypoints: ty.List[dict] = []
        self.third_waypoints: ty.List[dict] = []

        self.last_reached_wp = None                      # 当前到达的航点序号
        self.mission_state = None

        self.switching = False                           # 正在切换航线标志

        self.primary_waypoints_compeleted = False
        self.secondary_waypoints_compeleted = False
        self.third_waypoints_compeleted = False

        self.get_logger().info('PX4 DDS 节点就绪')

#回调函数
    def mission_result_callback(self, msg: MissionResult):
        """飞控任务执行结果回调"""
        self.current_mission_seq_current = msg.seq_current
        self.current_mission_seq_total = msg.seq_total
        self.current_mission_finished = msg.finished

        if msg.seq_reached >= 0:
            self.last_reached_wp = msg.seq_reached
            self.get_logger().info(
                f'航点到达: seq={msg.seq_reached}, '
                f'当前={msg.seq_current}/{msg.seq_total}, '
                f'finished={msg.finished}, 状态={self.mission_state}'
            )

            if self.switching:
                self.get_logger().info('航线切换进行中, 忽略此次到达')
                return

            self._check_mission_completion()

    def vehicle_status_callback(self, msg: VehicleStatus):
        """当前飞行器状态"""
        self.current_nav_state = msg.nav_state

    def vehicle_global_position_callback(self, msg: VehicleGlobalPosition):
        """当前GPS坐标"""
        self.current_global_position = (msg.lat, msg.lon, msg.alt)

#航线完成检查
    def _check_mission_completion(self):
        """检查当前航线是否执行完毕"""
        if self.mission_state == MissionState.DONE:
            return

        if self.current_mission_seq_total is None or self.current_mission_seq_total <= 0:
            return

        complete_wp = self.current_mission_seq_total - 2

        if self.mission_state == MissionState.PRIMARY and self.last_reached_wp >= complete_wp:
            self.get_logger().info('第一航线已执行完毕')
            self.primary_waypoints_compeleted = True
            self.mission_state = MissionState.SECONDARY
            self.switch_mission(self.secondary_waypoints)

        if self.mission_state == MissionState.SECONDARY and self.last_reached_wp >= complete_wp:
            self.get_logger().info('第二航线已执行完毕')
            self.secondary_waypoints_compeleted = True
            self.mission_state = MissionState.THIRD
            self.switch_mission(self.third_waypoints)

        if self.mission_state == MissionState.THIRD and self.last_reached_wp >= complete_wp:
            self.get_logger().info('第三航线已执行完毕')
            self.third_waypoints_compeleted = True
            self.mission_state = MissionState.DONE
            self.get_logger().info('所有航点已执行完毕')

#航线切换 (原生 DDS, 无 service 调用)
    def switch_mission(self, new_waypoints: ty.List[dict]):
        """通过 DDS 发布切换航线"""
        current_mode = f'nav_state={self.current_nav_state}'
        self.get_logger().info(f'===== 开始切换航线 (当前导航状态: {current_mode}) =====')

        self.switching = True

        # 步骤1: 切到 LOITER 模式
        self._send_set_mode_loiter()

        # 步骤2: 通过 DatamanRequest 上传新航线
        self._upload_mission_via_dataman(new_waypoints)

        # 步骤3: 切回 AUTO.MISSION
        self._send_set_mode_mission()

        self.get_logger().info('===== 航线切换请求已发送, 等待飞控响应 =====')

#原生 DDS 命令封装

    # PX4 navigation_state 常量 (来自 VehicleStatus)
    # NAVIGATION_STATE_AUTO_MISSION = 3
    # NAVIGATION_STATE_AUTO_LOITER   = 4

    def _send_set_mode_loiter(self):
        """通过 VehicleCommand 设置 AUTO.LOITER"""
        cmd = VehicleCommand()
        cmd.timestamp = self._now_us()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0                                  # 使用 custom mode
        cmd.param2 = 4.0                                  # AUTO.LOITER = 4
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True

        self.get_logger().info('发送模式切换: AUTO.LOITER')
        self.vehicle_command_pub.publish(cmd)

    def _send_set_mode_mission(self):
        """通过 VehicleCommand 设置 AUTO.MISSION"""
        cmd = VehicleCommand()
        cmd.timestamp = self._now_us()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0                                  # 使用 custom mode
        cmd.param2 = 3.0                                  # AUTO.MISSION = 3
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True

        self.get_logger().info('发送模式切换: AUTO.MISSION')
        self.vehicle_command_pub.publish(cmd)

    def _upload_mission_via_dataman(self, waypoints: ty.List[dict]):
        """通过 DatamanRequest 逐航点上传航线数据到飞控"""
        self.get_logger().info(f'开始上传航线: {len(waypoints)} 个航点')

        # 先清空原有的任务数据
        self._dataman_clear(0, 0)

        # 逐航点写入 dataman
        for i, wp in enumerate(waypoints):
            self._dataman_write_mission_item(0, i, wp)

        # 通知飞控加载任务
        self._publish_mission_count(len(waypoints))

    def _dataman_write_mission_item(self, client_id: int, index: int, wp: dict):
        """
        将单个航点序列化为二进制, 写入 PX4 dataman.
        mission_item_s 结构体布局 (PX4 固件版本相关):
          double lat;               // 8 bytes
          double lon;               // 8 bytes
          float altitude;           // 4 bytes
          float acceptance_radius;  // 4 bytes
          float loiter_radius;      // 4 bytes
          float yaw;                // 4 bytes
          uint16_t nav_cmd;         // 2 bytes
          uint16_t do_jump_ix;      // 2 bytes
          uint16_t do_jump_repeat;  // 2 bytes
          uint8_t frame;            // 1 byte
          uint8_t autocontinue;     // 1 byte
          uint8_t origin;           // 1 byte
          uint8_t padding[3];       // 3 bytes (对齐)
          总计: 44 bytes
        """
        raw = struct.pack(
            '<ddfffffHHHBBB3x',
            float(wp['lat']),
            float(wp['lon']),
            float(wp['alt']),
            float(wp.get('acceptance_radius', 15.0)),
            float(wp.get('loiter_radius', 50.0)),
            float(wp.get('yaw', 0.0)),
            float(0.0),                    # 额外 float padding (time_inside)
            int(wp['command']),
            0,                              # do_jump_ix
            0,                              # do_jump_repeat
            int(wp['frame']),
            int(wp['autocontinue']),
            0,                              # origin
        )

        self._dataman_send(client_id, 1, 0, index, raw)

    def _dataman_clear(self, client_id: int, item: int):
        """清空指定 dataman slot"""
        self._dataman_send(client_id, 2, item, 0, b'')

    def _dataman_send(self, client_id: int, request_type: int,
                       item: int, index: int, data: bytes):
        """发送 DatamanRequest"""
        req = DatamanRequest()
        req.timestamp = self._now_us()
        req.client_id = client_id
        req.request_type = request_type
        req.item = item
        req.index = index
        req.data_length = len(data)

        # data 字段为 uint8[56], 逐字节赋值
        data_bytes = list(data[:56])
        data_bytes += [0] * (56 - len(data_bytes))
        req.data = data_bytes

        self.dataman_request_pub.publish(req)

    def _publish_mission_count(self, count: int):
        """通过 Mission 消息通知飞控任务航点数量并启动"""
        msg = Mission()
        msg.timestamp = self._now_us()
        msg.mission_dataman_id = 0                       # 0 = 主任务
        msg.count = count
        msg.current_seq = 0                              # 从第0个航点开始

        self.get_logger().info(f'发布任务指令: count={count}, current_seq=0')
        self.mission_pub.publish(msg)

    def _now_us(self) -> int:
        """获取当前时间戳 (微秒)"""
        return self.get_clock().now().nanoseconds // 1000

#从waypoints文件中读取航点 (解析为 dict, 方便序列化)
    def read_waypoints_from_waypoints(self, file_path: str) -> ty.List[dict]:
        """从 QGC 导出的 .wpl 文件加载航点（支持空家点自动覆盖）"""
        waypoints = []

        with open(file_path, 'r') as f:
            lines = f.readlines()

        # 跳过第一行文件头（QGC WPL 110）
        for line in lines[1:]:
            # 1. 跳过空行
            stripped_line = line.strip()
            if not stripped_line:
                continue

            # 2. 按空白切分
            parts = stripped_line.split()
            if len(parts) < 12:
                self.get_logger().warning(f"跳过无效行（列数不足12）: {line.strip()}")
                continue

            # 3. 解析各字段
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

            # ===== 处理空家点（seq=0 且坐标全0） =====
            if seq == 0 and lat == 0.0 and lon == 0.0 and alt == 0.0:
                if self.current_global_position is not None:
                    # 用当前 GPS 覆盖
                    lat, lon, alt = self.current_global_position
                    command = 16   # 强制改为普通航点（MAV_CMD_NAV_WAYPOINT）
                    frame = 0      # 全局坐标系
                    self.get_logger().info(f"空家点已替换为当前GPS: ({lat:.7f}, {lon:.7f})")
                else:
                    # GPS 未就绪，跳过这一行（丢弃空家点）
                    self.get_logger().warning("空家点且GPS未锁定，跳过该航点")
                    continue

            # 4. 存储为 dict
            wp = {
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'frame': frame,
                'command': command,
                'autocontinue': int(autocontinue),
            }
            waypoints.append(wp)

        self.get_logger().info(f"从 {file_path} 加载了 {len(waypoints)} 个有效航点")
        return waypoints

#辅助函数
    def print_waypoint_info(self, waypoints: ty.List[dict]):
        """Print information about waypoints."""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total waypoints: {len(waypoints)}')
        self.get_logger().info('=' * 60)

        for i, wp in enumerate(waypoints):
            cmd_name = self.get_command_name(wp['command'])
            frame_name = self.get_frame_name(wp['frame'])

            self.get_logger().info(
                f'WP {i}: {cmd_name} | '
                f'Frame: {frame_name} | '
                f'Lat: {wp["lat"]:.6f}, Lon: {wp["lon"]:.6f}, Alt: {wp["alt"]:.2f}m'
            )
        self.get_logger().info('=' * 60)

    @staticmethod
    def get_command_name(command: int) -> str:
        """Convert MAVLink command number to name."""
        commands = {
            16: 'WAYPOINT',
            17: 'LOITER_UNLIM',
            18: 'LOITER_TURNS',
            19: 'LOITER_TIME',
            21: 'LAND',
            22: 'TAKEOFF',
            84: 'NAV_VTOL_TAKEOFF',
            85: 'NAV_VTOL_LAND',
        }
        return commands.get(command, f'CMD_{command}')

    @staticmethod
    def get_frame_name(frame: int) -> str:
        """Convert MAVLink frame number to name."""
        frames = {
            0: 'GLOBAL',
            3: 'GLOBAL_RELATIVE_ALT',
            6: 'GLOBAL_TERRAIN_ALT',
        }
        return frames.get(frame, f'FRAME_{frame}')

    #主函数
    def run(self):
        self.primary_waypoints = self.read_waypoints_from_waypoints(self.primary_path)
        self.print_waypoint_info(self.primary_waypoints)
        self.secondary_waypoints = self.read_waypoints_from_waypoints(self.secondary_path)
        self.print_waypoint_info(self.secondary_waypoints)
        self.third_waypoints = self.read_waypoints_from_waypoints(self.third_path)
        self.print_waypoint_info(self.third_waypoints)

        self.mission_state = MissionState.PRIMARY
        self.switching = True

        # 启动第一航线: 清空 -> 上传 -> 切 AUTO.MISSION
        self._dataman_clear(0, 0)
        for i, wp in enumerate(self.primary_waypoints):
            self._dataman_write_mission_item(0, i, wp)
        self._publish_mission_count(len(self.primary_waypoints))
        self._send_set_mode_mission()

        self.get_logger().info('第一航线已启动, 等待航点到达...')


def main(args=None):
    rclpy.init(args=args)
    node = ProjectPartFour()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()
