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


class ProjectPartThree(Node):

    def __init__(self):
        super().__init__('project_part_three')

        # QoS配置：PX4官方推荐的回调函数QoS参数
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # GPS 等传感器数据 mavros 采用 BEST_EFFORT 发布，订阅端也需匹配
        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 创建话题订阅者，用于获取飞行器的当前信息：
        #   1. 当前航点列表  2. 当前已到达航点  3. 当前GPS信息  4. 当前状态信息
        self.waypoints_sub = self.create_subscription(
            WaypointList, '/mavros/mission/waypoints',
            self.waypoints_callback, qos_profile,
        )

        self.waypoints_reached_sub = self.create_subscription(
            WaypointReached, '/mavros/mission/reached',
            self.waypoint_reached_callback, qos_profile,
        )

        self.global_position_sub = self.create_subscription(
            NavSatFix, 'mavros/global_position/global',
            self.global_position_callback, sensor_qos_profile,
        )

        self.state_sub = self.create_subscription(State, '/mavros/state',self.state_callback, qos_profile,)

        # self.secondary_waypoints_sub = self.create_subscription(WaypointList,'/secondary_waypoints',self.secondary_waypoints_callback,qos_profile)

        # 创建服务客户端，用于向飞控发送指令：
        #   1. 清除航点  2. 推送航点  3. 设置模式  4. 设置当前航点  5. 拉取航点
        self.clear_mission_client = self.create_client(WaypointClear, 'mavros/mission/clear')
        self.push_mission_client = self.create_client(WaypointPush, 'mavros/mission/push')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_current_client = self.create_client(WaypointSetCurrent, 'mavros/mission/set_current')
        self.pull_mission_client = self.create_client(WaypointPull, 'mavros/mission/pull')

        # 三段航点文件路径
        self.primary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/tingjiping/7_19_test_A.waypoints"
        self.secondary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/tingjiping/7_19_test_B.waypoints"
        self.third_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/tingjiping/7_19_test_C.waypoints"

        self.current_global_position = None              # 当前GPS坐标

        self.current_state = None                        # 当前状态

        self.current_callback_waypoints = None           # 当前回调函数的航点列表
        self.current_waypoints = None                    # 当前航点列表(执行的航点列表)
        self.primary_waypoints = []                      # 第一段航线列表
        self.secondary_waypoints = []                    # 第二段航线列表
        self.third_waypoints = []                        # 第三段航线列表

        self.wait_for_service()

        self.last_reached_wp = None                      # 当前到达的航点序号
        self.mission_state = None                       # 当前任务阶段

        self.switching = False                           # 正在切换航线标志
        self.pending_new_waypoints: ty.List[Waypoint] = []   # 待推送的新航线列表
        self.switch_phase = ''                           # 当前切换阶段 (set_loiter/clear/push/pull/set_current/wait_auto)
        self.switch_retry = 0                            # 切换步骤重试计数
        self.switch_timer = self.create_timer(0.5, self._switch_timer_callback)  # 切换状态机定时器（每0.5秒检查一次）

    # 回调函数
    def waypoints_callback(self, msg: WaypointList):
        """飞控返回的当前航点列表（仅用于调试）
        当航线执行出现异常时，可打印此列表来确认飞控实际收到的航点是否正确"""
        self.current_callback_waypoints = msg.waypoints

    def waypoint_reached_callback(self, msg: WaypointReached):
        """当前已到达的航点"""
        self.last_reached_wp = msg.wp_seq
        fc_count = len(self.current_callback_waypoints) if self.current_callback_waypoints else 0
        current_mode = self.current_state.mode if self.current_state else '?'
        self.get_logger().info(
            f'航点到达: wp={msg.wp_seq}, FC航点数={fc_count}, '
            f'状态={self.mission_state}, 飞控模式={current_mode}'
        )

        if not self.current_callback_waypoints or self.mission_state == MissionState.DONE:
            return

        if self.switching:
            self.get_logger().info('航线切换进行中, 忽略此次到达')
            return

        complete_wp = fc_count - 2

        if self.mission_state == MissionState.PRIMARY and self.last_reached_wp == complete_wp:
            self.get_logger().info('第一航线已执行完毕')
            self.mission_state = MissionState.SECONDARY
            self.last_reached_wp = 0
            self.switch_mission(self.secondary_waypoints)

        if self.mission_state == MissionState.SECONDARY and self.last_reached_wp == complete_wp:
            self.get_logger().info('第二航线已执行完毕')
            self.mission_state = MissionState.THIRD
            self.last_reached_wp = 0
            self.switch_mission(self.third_waypoints)

        if self.mission_state == MissionState.THIRD and self.last_reached_wp == complete_wp:
            self.get_logger().info('第三航线已执行完毕')
            self.mission_state = MissionState.DONE
            self.get_logger().info('所有航点已执行完毕')

    def global_position_callback(self, msg: NavSatFix):
        """当前GPS坐标"""
        self.current_global_position = (msg.latitude, msg.longitude, msg.altitude)

    def state_callback(self, msg: State):
        """当前状态"""
        self.current_state = msg

    # def secondary_waypoints_callback(self,msg):
    #     self.secondary_waypoints = msg
# 等待服务就绪
    def wait_for_service(self):
        self.get_logger().info('等待 MAVROS 连接服务就绪...')

        services = [
            (self.push_mission_client, 'WaypointPush'),
            (self.pull_mission_client, 'WaypointPull'),
            (self.clear_mission_client, 'WaypointClear'),
            (self.set_current_client, 'WaypointSetCurrent'),
            (self.set_mode_client, 'SetMode'),
        ]

        for service, name in services:
            while not service.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning(f'等待 {name} 连接...')

        self.get_logger().info('所有 MAVROS 准备就绪!')

# 原生异步调用（使用 Future 链式回调替代 spin_until_future_complete）
    def _do_clear_mission(self, on_done: ty.Callable):
        """异步清空航线"""
        request = WaypointClear.Request()
        future = self.clear_mission_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_service_done(f, '清空航线', on_done)
        )

    def _do_push_mission(self, waypoints: ty.List[Waypoint], on_done: ty.Callable):
        """异步推送航线"""
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints

        future = self.push_mission_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_service_done(f, '推送航线', on_done)
        )

    def _do_pull_mission(self, on_done: ty.Callable):
        """异步拉取航线"""
        request = WaypointPull.Request()
        future = self.pull_mission_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_service_done(f, '拉取航线', on_done)
        )

    def _do_set_mode(self, mode: str, on_done: ty.Callable):
        """异步设置模式"""
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_set_mode_done(f, mode, on_done)
        )

    def _do_set_current_waypoint(self, wp_seq: int, on_done: ty.Callable):
        """异步设置当前航点"""
        request = WaypointSetCurrent.Request()
        request.wp_seq = wp_seq

        future = self.set_current_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_service_done(f, '设置当前航点', on_done)
        )

    def _handle_service_done(self, future, name: str, on_done: ty.Callable):
        """通用服务调用结果处理"""
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'{name}成功')
        else:
            self.get_logger().error(f'{name}失败')
        on_done()

    def _handle_set_mode_done(self, future, mode: str, on_done: ty.Callable):
        """模式切换结果处理"""
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'模式切换成功: {mode}')
        else:
            self.get_logger().warn(f'模式切换未确认: {mode} (可能已在该模式)')
        on_done()

# 异步航线切换链（Future 回调 + 定时器状态机，每步确认飞控状态后再走下一步）
    def switch_mission(self, new_waypoints: ty.List[Waypoint]):
        """启动异步航线切换"""
        current_mode = self.current_state.mode if self.current_state else '?'
        self.get_logger().info(f'===== 开始切换航线 (当前飞控模式: {current_mode}) =====')

        self.switching = True
        self.pending_new_waypoints = new_waypoints
        self.switch_phase = 'set_loiter'
        self.switch_retry = 0

        # 步骤1: 发送 LOITER 命令 (异步)
        # self._do_set_mode('AUTO.LOITER', self._on_mode_sent)
        self._do_set_mode('LOITER', self._on_mode_sent)
    def _on_mode_sent(self):
        """模式切换命令已发出, 定时器等待飞控确认"""
        self.switch_retry = 0

    def _switch_timer_callback(self):
        """定时器驱动的切换状态机, 每步确认飞控状态"""
        if not self.switching:
            return

        self.get_logger().info(f'切换阶段: {self.switch_phase}, 重试: {self.switch_retry}')

        # --- 阶段1: 等待 LOITER 模式确认 ---
        if self.switch_phase == 'set_loiter':
            # if self.current_state is not None and self.current_state.mode == 'AUTO.LOITER':
            if self.current_state is not None and self.current_state.mode == 'AUTO':
                self.get_logger().info('[切换] LOITER 已确认, 清空航线')
                self.switch_phase = 'clear'
                self.switch_retry = 0
                self._do_clear_mission(self._on_step_done)
            elif self.switch_retry >= 10:
                self.get_logger().info('[切换] LOITER 等待超时, 强制继续')
                self.switch_phase = 'clear'
                self.switch_retry = 0
                self._do_clear_mission(self._on_step_done)
            elif self.switch_retry == 0:
                # 首次进入, 可能命令还没发出, 再次发送   
                # self._do_set_mode('AUTO.LOITER', self._on_mode_sent)
                self._do_set_mode('LOITER', self._on_mode_sent)
                self.switch_retry += 1
            else:
                self.switch_retry += 1

        # --- 阶段2: 清空航线 ---
        elif self.switch_phase == 'clear':
            pass  # 等待 _do_clear_mission 的完成回调

        # --- 阶段3: 推送航线 ---
        elif self.switch_phase == 'push':
            pass

        # --- 阶段4: 拉取验证 ---
        elif self.switch_phase == 'pull':
            pass

        # --- 阶段5: 设置当前航点 ---
        elif self.switch_phase == 'set_current':
            pass

        # --- 阶段6: 等待 AUTO.MISSION 模式确认 ---
        elif self.switch_phase == 'wait_auto':
            self._check_auto_mode(15, '===== 航线切换完成, 等待航点到达 =====')

        # --- 初始化阶段: 等待 AUTO.MISSION 模式确认 ---
        elif self.switch_phase == 'init_auto':
            self._check_auto_mode(30, '第一航线已启动, 等待航点到达...')

    def _check_auto_mode(self, max_retries: int, done_msg: str):
        """等待飞控确认 AUTO.MISSION 模式"""
        # if self.current_state is not None and self.current_state.mode == 'AUTO.MISSION':
        if self.current_state is not None and self.current_state.mode == 'AUTO':
            self.switching = False
            self.pending_new_waypoints = []
            self.switch_phase = ''
            self.get_logger().info(done_msg)
        elif self.switch_retry >= max_retries:
            self.get_logger().info(f'[切换] AUTO.MISSION 等待超时({max_retries}), 强制结束')
            self.switching = False
            self.pending_new_waypoints = []
            self.switch_phase = ''
        elif self.switch_retry == 0:
            self._do_set_mode('AUTO', self._on_mode_sent)
            self.switch_retry += 1
        else:
            self.switch_retry += 1

    def _on_step_done(self):
        """服务调用完成，推进切换阶段"""
        if self.switch_phase == 'clear':
            self.get_logger().info('[切换] 清空完成, 推送新航线')
            self.switch_phase = 'push'
            self._do_push_mission(self.pending_new_waypoints, self._on_step_done)

        elif self.switch_phase == 'push':
            self.get_logger().info('[切换] 推送完成, 拉取验证')
            self.switch_phase = 'pull'
            self._do_pull_mission(self._on_step_done)

        elif self.switch_phase == 'pull':
            self.get_logger().info('[切换] 拉取完成, 设置当前航点为0')
            self.switch_phase = 'set_current'
            self._do_set_current_waypoint(0, self._on_step_done)

        elif self.switch_phase == 'set_current':
            # self.get_logger().info('[切换] 当前航点已设置, 切回 AUTO.MISSION')
            self.get_logger().info('[切换] 当前航点已设置, 切回 AUTO')
            self.switch_phase = 'wait_auto'
            self.switch_retry = 0
            # self._do_set_mode('AUTO.MISSION', self._on_mode_sent)
            self._do_set_mode('AUTO', self._on_mode_sent)

# 从waypoints文件中读取航点
    def read_waypoints_from_waypoints(self, file_path: str) -> ty.List[Waypoint]:
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

            # 4. 创建 Waypoint 对象
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

        self.get_logger().info(f"从 {file_path} 加载了 {len(waypoints)} 个有效航点")
        return waypoints

# 辅助函数
    def print_waypoint_info(self, waypoints: ty.List[Waypoint]):
        """打印航点信息"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total waypoints: {len(waypoints)}')
        self.get_logger().info('=' * 60)

        for i, wp in enumerate(waypoints):
            cmd_name = self.get_command_name(wp.command)
            frame_name = self.get_frame_name(wp.frame)

            self.get_logger().info(
                f'WP {i}: {cmd_name} | '
                f'Frame: {frame_name} | '
                f'Lat: {wp.x_lat:.6f}, Lon: {wp.y_long:.6f}, Alt: {wp.z_alt:.2f}m | '
                f'Current: {wp.is_current}'
            )
        self.get_logger().info('=' * 60)

    @staticmethod
    def get_command_name(command: int) -> str:
        """将MAVLink命令编号转换为名称"""
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
        """将MAVLink帧编号转换为名称"""
        frames = {
            0: 'GLOBAL',
            3: 'GLOBAL_RELATIVE_ALT',
            6: 'GLOBAL_TERRAIN_ALT',
        }
        return frames.get(frame, f'FRAME_{frame}')

    # 主函数
    def run(self):
        self.primary_waypoints = self.read_waypoints_from_waypoints(self.primary_path)
        self.print_waypoint_info(self.primary_waypoints)
        self.secondary_waypoints = self.read_waypoints_from_waypoints(self.secondary_path)
        self.print_waypoint_info(self.secondary_waypoints)
        self.third_waypoints = self.read_waypoints_from_waypoints(self.third_path)
        self.print_waypoint_info(self.third_waypoints)

        self.mission_state = MissionState.PRIMARY
        self.switching = True
        self.switch_retry = 0

        # 使用异步链启动第一航线
        self.switch_phase = 'init_clear'
        self._do_clear_mission(self._on_init_step_done)

    def _on_init_step_done(self):
        """初始化异步链回调：清空→推送→拉取→设置当前航点→切AUTO模式"""
        if self.switch_phase == 'init_clear':
            self.get_logger().info('[初始化] 清空完成, 推送第一航线')
            self.switch_phase = 'init_push'
            self._do_push_mission(self.primary_waypoints, self._on_init_step_done)
        elif self.switch_phase == 'init_push':
            self.get_logger().info('[初始化] 推送完成, 拉取验证')
            self.switch_phase = 'init_pull'
            self._do_pull_mission(self._on_init_step_done)
        elif self.switch_phase == 'init_pull':
            self.get_logger().info('[初始化] 拉取完成, 设置当前航点为0')
            self.switch_phase = 'init_current'
            self._do_set_current_waypoint(0, self._on_init_step_done)
        elif self.switch_phase == 'init_current':
            # self.get_logger().info('[初始化] 切到 AUTO.MISSION')
            self.get_logger().info('[初始化] 切到 AUTO')
            self.switch_phase = 'init_auto'
            self.switch_retry = 0
            # self._do_set_mode('AUTO.MISSION', self._on_mode_sent)
            self._do_set_mode('AUTO', self._on_mode_sent)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectPartThree()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()
