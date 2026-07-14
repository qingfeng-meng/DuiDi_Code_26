import typing as ty
import time
import rclpy
from mavros_msgs.msg import Waypoint,WaypointList,WaypointReached
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped
from enum import Enum

class MissionState(Enum):
    TAKEOFF_AND_SEACH = 1
    ATTACK              = 2
    RETURN_AND_LAND     = 3

class WaypointMissionManager(Node):
    #初始化
    def __init__(self):
        super().__init__('waypoint_mission_manager')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        #创建客户端
        self.push_client = self.create_client(WaypointPush, '/mavros/mission/push')         #将一组航点上传到飞控任务列表               
        self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')         #从飞控下载当前储存的全部航点       成功会返回飞控上现有航电的数量和列表
        self.clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')      #清空飞控上的所有任务
        self.set_current_client = self.create_client(
            WaypointSetCurrent, '/mavros/mission/set_current'
        )                                                                                   #将某个指定的航点设置为当前正在执行的航点    参数：航点序列号（wp_seq)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')              #切换飞控的飞行模式

        #创建订阅者
        self.waypoints_sub = self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_callback,
            qos_profile,
        )                                                                                   #用于查看飞控上现在是什么任务

        self.reached_sub = self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            qos_profile,
        )    

        # State variables
        self.current_waypoints = []
        self.last_reached_wp = None
        self.mission_state = MissionState.TAKEOFF_AND_SEACH

        self.get_logger().info('Waypoint Mission Manager initialized')
        self.wait_for_services()

    #第一航线：创建起飞侦察航线 (约 310m×300m 范围)
    def create_seach_mission(self) -> ty.List[Waypoint]:
        waypoints = []

        # ===== 原版大航线（已注释） =====
        # home  = self.create_waypoint(28.656984 , 115.825884 , 1.0 , frame=3 , command=22 ,is_current=True)
        # waypoints.append(home)
        # take_off = self.create_waypoint(28.657821 , 115.825873 , 35.0 ,frame=3 , command=16 ,param1=18.0)
        # waypoints.append(take_off)
        # waypoints.append(self.create_waypoint(28.658085 , 115.825882 , 35.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.658221 , 115.826049 , 35.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.658196 , 115.826580 , 35.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.658019 , 115.826747 , 35.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.657015 , 115.826728 , 35.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.656836 , 115.826483 , 35.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.656825 , 115.826059 , 35.0 , frame=3 , command=16, param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.657040 , 115.825897 , 35.0 , frame=3 , command=16, param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.658057 , 115.825882 ,  0.0 , frame=3 , command=21 ))
        # home  = self.create_waypoint(28.656984 , 115.825884 , 1.0 , frame=3 , command=22 , param1=18.0 )
        # waypoints.append(home)

        # take_off = self.create_waypoint(28.658436 , 115.825836 , 25.0 ,frame=3 , command=22 ,param1=18.0, is_current=True)
        # waypoints.append(take_off)
        # waypoints.append(self.create_waypoint(28.659213 , 115.826520 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.659782 , 115.827022 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.659651 , 115.828607 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.658669 , 115.829293 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.656732 , 115.829241 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.655484 , 115.829015 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.655333 , 115.827458 , 25.0 , frame=3 , command=19 , param1 = 30.0 ,param3 = 30.0))
        # waypoints.append(self.create_waypoint(28.655924 , 115.826177 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.658529 , 115.826030 , 0.0 , frame=3 , command=21 ))

        # ===== 缩小版航线 (~120m×100m 平滑曲线，降落角<18°) =====
        # 航线设计：椭圆搜索路径，无直角转弯，降落下滑角约14°
        wpt = self.create_waypoint  # 简写

        wp_takeoff = wpt(28.658436, 115.825836, 25.0, frame=3, command=22, param1=18.0, is_current=True)
        waypoints.append(wp_takeoff)

        # 椭圆搜索路径（顺时针，弧线转弯）
        waypoints.append(wpt(28.65890, 115.82620, 25.0, frame=3, command=16, param2=5.0))
        waypoints.append(wpt(28.65910, 115.82680, 25.0, frame=3, command=16, param2=5.0))
        waypoints.append(wpt(28.65860, 115.82690, 25.0, frame=3, command=16, param2=5.0))
        waypoints.append(wpt(28.65820, 115.82630, 25.0, frame=3, command=16, param2=5.0))
        waypoints.append(wpt(28.65800, 115.82590, 25.0, frame=3, command=16, param2=5.0))

        # 盘旋点
        waypoints.append(wpt(28.65850, 115.82630, 25.0, frame=3, command=19, param1=30.0, param3=30.0))

        # 降落进近点（~100m水平距离，25m高差→14.1°下滑角<18°）
        waypoints.append(wpt(28.65890, 115.82510, 25.0, frame=3, command=16, param2=5.0))

        wp_land = wpt(28.658529, 115.826030, 0.0, frame=3, command=21)
        waypoints.append(wp_land)

        return waypoints
    #第二航线：创建攻击航线 (缩小版)
    def create_attack_mission(self) -> ty.List[Waypoint]:
        waypoints = []

        # ===== 原版大航线（已注释） =====
        # waypoints.append(self.create_waypoint(28.660799 , 115.827137 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.659620 , 115.829609 , 25.0 , frame=3 , command=16 , param2 = 3.0))
        # waypoints.append(self.create_waypoint(28.657693 , 115.830195 , 25.0 , frame=3 , command=19 , param1 = 30.0 , param3 = 30.0 ))
        # waypoints.append(self.create_waypoint(28.657826 , 115.828601 , 25.0 , frame=3 , command=16 ))
        # waypoints.append(self.create_waypoint(28.658613 , 115.825949 , 20.0 , frame=3 , command=21 ))

        # ===== 缩小版攻击航线（平滑路径，统一降落进近） =====
        wpt = self.create_waypoint

        waypoints.append(wpt(28.65880, 115.82660, 25.0, frame=3, command=16, param2=5.0))

        # 打击点
        waypoints.append(wpt(28.65860, 115.82720, 25.0, frame=3, command=16, param2=3.0))

        # 盘旋点
        waypoints.append(wpt(28.65820, 115.82700, 25.0, frame=3, command=19, param1=30.0, param3=30.0))

        waypoints.append(wpt(28.65790, 115.82630, 25.0, frame=3, command=16, param2=5.0))

        # 降落进近点（~100m，下滑角<18°）
        waypoints.append(wpt(28.65890, 115.82510, 25.0, frame=3, command=16, param2=5.0))

        wp_land = wpt(28.658529, 115.826030, 0.0, frame=3, command=21)
        waypoints.append(wp_land)

        return waypoints
    #第三航线：创建返航航线 (缩小版)
    def create_return_mission(self) -> ty.List[Waypoint]:
        waypoints = []

        # ===== 原版大航线（已注释） =====
        # waypoints.append(self.create_waypoint(28.657986 , 115.829223 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.657107 , 115.829162 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.656713 , 115.827070 , 25.0 , frame=3 , command=16 , param2 = 5.0))
        # waypoints.append(self.create_waypoint(28.658022 , 115.825871 , 0.0 , frame=3 , command=21 ))

        # ===== 缩小版返航航线（平滑路径，统一降落进近） =====
        wpt = self.create_waypoint

        waypoints.append(wpt(28.65840, 115.82680, 25.0, frame=3, command=16, param2=5.0))
        waypoints.append(wpt(28.65800, 115.82630, 25.0, frame=3, command=16, param2=5.0))

        # 降落进近点（~100m，下滑角<18°）
        waypoints.append(wpt(28.65890, 115.82510, 25.0, frame=3, command=16, param2=5.0))

        wp_land = wpt(28.658529, 115.826030, 0.0, frame=3, command=21)
        waypoints.append(wp_land)

        return waypoints
        
#功能函数封装
    #等待全部服务就绪
    def wait_for_services(self):

        self.get_logger().info('等待 MAVROS 连接服务就绪...')

        services = [
            (self.push_client, 'WaypointPush'),
            (self.pull_client, 'WaypointPull'),
            (self.clear_client, 'WaypointClear'),
            (self.set_current_client, 'WaypointSetCurrent'),
            (self.set_mode_client, 'SetMode'),
        ]

        for client, name in services:
            while not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(f'等待 {name} 连接...')

        self.get_logger().info('所有 MAVROS 准备就绪!')

    #回调函数创建
    def waypoints_callback(self, msg: WaypointList):
        self.current_waypoints = msg.waypoints
        # self.get_logger().info(f'Received {len(self.current_waypoints)} waypoints from mission')

    def waypoint_reached_callback(self, msg: WaypointReached):
        self.last_reached_wp = msg.wp_seq
        self.get_logger().info(f'current state : {self.mission_state}')
        self.get_logger().info(f'Waypoint {msg.wp_seq} reached!')

        if self.mission_state == MissionState.TAKEOFF_AND_SEACH and self.last_reached_wp == len(self.create_seach_mission()) - 2:
            self.last_reached_wp = 0
            self.get_logger().info('Mission TAKEOFF_AND_SEACH completed!')

            self.mission_state = MissionState.ATTACK

            self.update_mission(self.create_attack_mission())

        if self.mission_state == MissionState.ATTACK and self.last_reached_wp == len(self.create_attack_mission()) - 2:
            self.last_reached_wp = 0
            self.get_logger().info('Mission RETURN_AND_LAND completed!')
            self.mission_state = MissionState.RETURN_AND_LAND
            self.update_mission(self.create_return_mission())

    def set_mode(self, mode: str = 'AUTO') -> bool:
        # """
        # Set the flight mode.
        #
        # :param mode: Flight mode string (e.g., 'AUTO', 'GUIDED', 'STABILIZE', 'LOITER')
        # :returns: True if successful, False otherwise
        # """
        request = SetMode.Request()
        request.custom_mode = mode

        self.get_logger().info(f'Setting flight mode to {mode}...')

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'Successfully set mode to {mode}')
                return True
            else:
                self.get_logger().error(f'Failed to set mode to {mode}')
                return False
        else:
            self.get_logger().error('SetMode service call failed')
            return False
        

    #更新任务函数
    def update_mission(self, new_waypoints: ty.List[Waypoint]) -> bool:
        """"
        LOITER -> 清空 -> 上传 -> 下拉 -> 切换AUTO
        """
        self.get_logger().info('\n--- change mode to LOITER ---')
        self.set_mode('LOITER')
    

        self.get_logger().info('\n--- Clearing mission ---')
        self.clear_waypoints()
  

        self.get_logger().info('\n--- updating and Pushing Sample Mission ---')
        waypoints = new_waypoints
        self.print_waypoint_info(waypoints)
        self.push_waypoints(waypoints)


        self.get_logger().info('\n--- pulling mission ---')
        pulled_waypoints = self.pull_waypoints()
        if pulled_waypoints:
            self.print_waypoint_info(pulled_waypoints)


        self.get_logger().info('\n--- Setting Current Waypoint ---')
        self.set_current_waypoint(0)


        self.get_logger().info('\n--- Setting Mode to AUTO ---')
        self.set_mode('AUTO.MISSION')

        self.get_logger().info('\nNode is running. Press Ctrl+C to exit.')
        self.get_logger().info('Monitoring waypoint reached events...')

        
    def create_waypoint(
        self,
        lat: float,
        lon: float,
        alt: float,
        frame: int = 0,
        command: int = 16,
        is_current: bool = False,
        autocontinue: bool = True,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
    ) -> Waypoint:
        # """
        # Create a MAVLink waypoint.
        #
        # :param lat: Latitude in degrees
        # :param lon: Longitude in degrees
        # :param alt: Altitude in meters
        # :param frame: MAVLink frame (0=GLOBAL, 3=GLOBAL_RELATIVE_ALT, etc.)
        # :param command: MAVLink command (16=WAYPOINT, 22=TAKEOFF, 21=LAND, etc.)
        # :param is_current: Whether this is the current waypoint
        # :param autocontinue: Auto-continue to next waypoint
        # :param param1-4: Command-specific parameters
        # :returns: Waypoint message
        # """
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

        return wp
    
    def push_waypoints(self, waypoints: ty.List[Waypoint]) -> bool:
        # """
        # Upload waypoints to the flight controller.
        #
        # :param waypoints: List of Waypoint messages
        # :returns: True if successful, False otherwise
        # """
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints

        #self.get_logger().info(f'Pushing {len(waypoints)} waypoints to FCU...')

        future = self.push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully pushed {response.wp_transfered} waypoints')
                return True
            else:
                self.get_logger().error('Failed to push waypoints')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
        
    def pull_waypoints(self) -> ty.Optional[ty.List[Waypoint]]:
        # """
        # Download waypoints from the flight controller.
        #
        # :returns: List of waypoints or None if failed
        # """
        request = WaypointPull.Request()

        #self.get_logger().info('Pulling waypoints from FCU...')

        future = self.pull_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully pulled {response.wp_received} waypoints')
            else:
                self.get_logger().error('Failed to pull waypoints')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None

    def clear_waypoints(self) -> bool:
        # """
        # Clear all waypoints from the flight controller.
        #
        # :returns: True if successful, False otherwise
        # """
        request = WaypointClear.Request()

        # self.get_logger().info('Clearing all waypoints from FCU...')

        future = self.clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully cleared all waypoints')
                return True
            else:
                self.get_logger().error('Failed to clear waypoints')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
        
    def set_current_waypoint(self, wp_seq: int) -> bool:
        # """
        # Set the current active waypoint.
        #
        # :param wp_seq: Waypoint sequence number to set as current
        # :returns: True if successful, False otherwise
        # """
        request = WaypointSetCurrent.Request()
        request.wp_seq = wp_seq

        # self.get_logger().info(f'Setting waypoint {wp_seq} as current...')

        future = self.set_current_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully set waypoint {wp_seq} as current')
                return True
            else:
                self.get_logger().error(f'Failed to set waypoint {wp_seq} as current')
                return False
        else:
            self.get_logger().error('Service call failed')
            return False
        
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
    
def main(args=None):
    rclpy.init(args=args)

    manager = WaypointMissionManager()

    try:
        # Wait a moment for initial connections
     

        manager.get_logger().info('=== Example 2: Basic Waypoint Mission ===')
        manager.get_logger().info('\n--- Clearing Existing Waypoints ---')
        manager.clear_waypoints()

    

        # Example 2: Create and push a sample mission
        manager.get_logger().info('\n--- Creating and Pushing Sample Mission ---')
        seach_mission = manager.create_seach_mission()
        manager.print_waypoint_info(seach_mission)
        manager.push_waypoints(seach_mission)

     

        # Example 3: Pull waypoints from FCU
        manager.get_logger().info('\n--- Pulling Waypoints from FCU ---')
        pulled_waypoints = manager.pull_waypoints()
        if pulled_waypoints:
            manager.print_waypoint_info(pulled_waypoints)

      

        # Example 4: Set a specific waypoint as current (skip to waypoint 2)
        manager.get_logger().info('\n--- Setting Current Waypoint ---')
        manager.set_current_waypoint(0)


        # Example 5: Set mode to AUTO
        manager.get_logger().info('\n--- Setting Mode to AUTO ---')
        manager.set_mode('AUTO.MISSION')

        # Keep node alive to receive callbacks
        manager.get_logger().info('\nNode is running. Press Ctrl+C to exit.')
        manager.get_logger().info('Monitoring waypoint reached events...')

        rclpy.spin(manager)

    except KeyboardInterrupt:
        manager.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        manager.destroy_node()
        rclpy.shutdown()


        
