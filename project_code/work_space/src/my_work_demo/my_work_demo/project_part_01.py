import typing as ty
import json
import rclpy
from mavros_msgs.msg import Waypoint,WaypointList,WaypointReached,State
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from enum import Enum

class MissionState(Enum):
    TAKEOFF_AND_SEACH = 1
    ATTACK              = 2
    RETURN_AND_LAND     = 3

class ProjectPartOne(Node):
    def __init__(self):
        super().__init__('project_part_one')

        # QoS PX4官方的用于CallBack函数
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

        # 创建话题订阅者用于获取飞行器的当前信息
        #1. 获取当前航点列表 2. 获取当前航点已 reached 3. 获取当前GPS信息 4. 获取当前状态信息 5. 获取第二航线列表
        self.waypoints_sub = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_callback,
            qos_profile,
        )

        self.waypoints_reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            qos_profile,
        )

        self.global_position_sub = self.create_subscription(
            NavSatFix,
            'mavros/global_position/global',
            self.global_position_callback,sensor_qos_profile
        )

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,qos_profile
        )

        # self.secondary_waypoints_sub = self.create_subscription(
        #     WaypointList,
        #     '/secondary_waypoints',
        #     self.secondary_callback,qos_profile
        # )

        #创建服务客户端
        #1. 清除航点 2. 推送航点 3. 设置模式 4. 设置当前航点 5. 拉取航点
        self.clear_mission_client = self.create_client(WaypointClear, 'mavros/mission/clear')
        self.push_mission_client = self.create_client(WaypointPush, 'mavros/mission/push')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.set_current_client = self.create_client(WaypointSetCurrent, 'mavros/mission/set_current')
        # self.pull_mission_client = self.create_client(WaypointPull, 'mavros/mission/pull')

        # 三段航点文件路径
        self.primary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_A.waypoints"
        self.secondary_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_B.waypoints"
        self.third_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/waypoints/dacaochang/dacaochang_test_02_C.waypoints"


        self.current_global_position = None              # 当前GPS坐标

        self.current_state = None                        # 当前状态

        self.current_callback_waypoints = None           # 当前回调函数的航点列表
        self.current_waypoints = None                    # 当前航点列表(执行的航点列表)
        self.primary_waypoints = None
        self.secondary_waypoints = None
        self.third_waypoints = None

        self.wait_for_service()

        self.last_reached_wp = None                      # 当前到达的航点序号
        self.mission_state = None

        self.primary_waypoints_compeleted = False
        self.secondary_waypoints_compeleted = False
        self.third_waypoints_compeleted = False
    
#回调函数
    def waypoints_callback(self, msg: WaypointList):
        """"从飞控中获取的航点列表"""
        self.current_callback_waypoints = msg.waypoints

    def waypoint_reached_callback(self, msg: WaypointReached):
        """"当前已到达的航点"""
        self.last_reached_wp = msg.wp_seq
        self.get_logger().info(f'已到达航点 {msg.wp_seq}')

        if self.mission_state == MissionState.TAKEOFF_AND_SEACH and self.last_reached_wp == len(self.primary_waypoints) - 2:
            self.last_reached_wp = 0
            self.get_logger().info('第一航线已执行完毕, 切换第二航线')
            self.secondary_waypoints = self.read_waypoints_from_waypoints(self.secondary_path)
            self.print_waypoint_info(self.secondary_waypoints)      
            self.mission_state = MissionState.ATTACK
            self.update_mission(self.secondary_waypoints)

        if self.mission_state == MissionState.ATTACK and self.last_reached_wp == len(self.secondary_waypoints) - 2:
            self.last_reached_wp = 0
            self.get_logger().info('第二航线已执行完毕, 切换第三航线')
            self.third_waypoints = self.read_waypoints_from_waypoints(self.third_path)
            self.print_waypoint_info(self.third_waypoints)              
            self.mission_state = MissionState.RETURN_AND_LAND
            self.update_mission(self.third_waypoints)


    def global_position_callback(self, msg: NavSatFix):
        """当前GPS坐标"""
        self.current_global_position = (msg.latitude, msg.longitude, msg.altitude)

    def state_callback(self, msg: State):
        """当前状态"""
        self.current_state = msg

    # def secondary_callback(self, msg: WaypointList):
    #     """第二航线订阅获取"""
    #     self.secondary_waypoints = msg.waypoints

#等待服务就绪
    def wait_for_service(self):
        self.get_logger().info('等待 MAVROS 连接服务就绪...')

        services = [
            (self.push_mission_client, 'WaypointPush'),
            # (self.pull_mission_client, 'WaypointPull'),
            (self.clear_mission_client, 'WaypointClear'),
            (self.set_current_client, 'WaypointSetCurrent'),
            (self.set_mode_client, 'SetMode'),
        ]

        for service, name in services:
            while not service.wait_for_service(timeout_sec=3.0):
                self.get_logger().warning(f'等待 {name} 连接...')

        self.get_logger().info('所有 MAVROS 准备就绪!')

    #清空航点
    def clear_mission(self):        
        request = WaypointClear.Request()

        future = self.clear_mission_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('成功清空航点')
                return True
            else:
                self.get_logger().error('清空航点失败')
                return False
        else:
            self.get_logger().error('服务请求失败')
            return False

    #推送航点        
    def push_mission(self, waypoints: ty.List[Waypoint]) -> bool:
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints

        future = self.push_mission_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('成功推送航点')
                return True
            else:
                self.get_logger().error('推送航点失败')
                return False
        
        else:
            self.get_logger().error('服务请求失败')
            return False
        
    #设置模式    
    def set_mode(self, mode: str = 'AUTO') -> bool:
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'成功切换模式为 {mode}')
                return True
            else:
                self.get_logger().error(f'切换模式为 {mode} 失败')
                return False
        else:
            self.get_logger().error('服务请求失败')
            return False
        
    #设置当前航点    
    def set_current_waypoint(self, wp_seq: int) -> bool:
        request = WaypointSetCurrent.Request()
        request.wp_seq = wp_seq

        future = self.set_current_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'成功设置当前航点为 {wp_seq}')
                return True
            else:
                self.get_logger().error(f'设置当前航点为 {wp_seq} 失败')
                return False
            
        else:
            self.get_logger().error('服务请求失败')
            return False

#从waypoints文件中读取航点
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

    def update_mission(self, new_waypoints: ty.List[Waypoint]) -> bool:
        """在此函数中进行航点更新"""
        """上传新的航线，需要先更换航线"""
        #先换成盘旋模式
        self.get_logger().info('切换模式为盘旋模式')
        self.set_mode('AUTO.LOITER')

        #清除当前航线
        self.get_logger().info('清除当前航线')
        self.clear_mission()

        #推送新的航线
        self.get_logger().info('推送新的航线')
        self.push_mission(new_waypoints)

        #设置当前航点为第一个航点
        self.set_current_waypoint(0)
        
        #切换模式为自动模式
        self.get_logger().info('切换模式为自动模式')
        self.set_mode('AUTO.MISSION')

#辅助函数
    def print_waypoint_info(self, waypoints: ty.List[Waypoint]):
        """Print information about waypoints."""
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


        self.mission_state = MissionState.TAKEOFF_AND_SEACH
        self.clear_mission()
        self.push_mission(self.primary_waypoints)
        self.set_current_waypoint(0)
        self.set_mode('AUTO.MISSION')




def main(args=None):
    rclpy.init(args=args)
    node = ProjectPartOne()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()

        
