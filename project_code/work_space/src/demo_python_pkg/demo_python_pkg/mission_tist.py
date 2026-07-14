
import typing as ty
import json
import math
import numpy as np
from tf_transformations import euler_from_quaternion,quaternion_from_euler,quaternion_multiply
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import Waypoint,WaypointList,WaypointReached,State,PositionTarget,GlobalPositionTarget,VfrHud,AttitudeTarget
from mavros_msgs.srv import SetMode, WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent,CommandInt,CommandLong
from geometry_msgs.msg import PoseStamped,TwistStamped,Quaternion

from vision_msgs.msg import TargetCoord     #自定义消息

class WaypointMissionManager(Node):
    """Manager node."""
    #初始化
    def __init__(self):
        super().__init__('waypoint_mission_manager')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        offboard_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
    
        # Create service clients
        self.push_client = self.create_client(WaypointPush, '/mavros/mission/push')         #将一组航点上传到飞控任务列表               
        self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')         #从飞控下载当前储存的全部航点       成功会返回飞控上现有航电的数量和列表
        self.clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')      #清空飞控上的所有任务
        self.set_current_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')                   #将某个指定的航点设置为当前正在执行的航点    参数：航点序列号（wp_seq)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')              #切换飞控的飞行模式
        self.command_int_client = self.create_client(CommandInt, '/mavros/cmd/command')    
        self.waypoints_sub = self.create_subscription(WaypointList,'/mavros/mission/waypoints',self.waypoints_callback,qos_profile,)           #用于查看飞控上现在是什么任务
        self.reached_sub = self.create_subscription(WaypointReached,'/mavros/mission/reached',self.waypoint_reached_callback,qos_profile,)     #监听任务执行进度，了解目前飞到了第几个航点
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, offboard_qos)
        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', offboard_qos)
        self.globl_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_raw/global', qos_profile)
        self.vel_pub = self.create_publisher(TwistStamped,'/mavros/setpoint_velocity/cmd_vel',offboard_qos)         #速度控制
        self.local_pos_pub = self.create_publisher(PoseStamped,'/mavros/setpoint_position/local',offboard_qos)      #位置控制
        self.attitude_pub = self.create_publisher(AttitudeTarget,'/mavros/setpoint_raw/attitude',offboard_qos)      #姿态控制(四元数控制)
        self.local_pos_sub = self.create_subscription(PoseStamped,'/mavros/local_position/pose',self.local_pos_callback,offboard_qos)   #订阅当前位置
        self.airspeed_sub = self.create_subscription(VfrHud,"/mavros/vfr_hud",self.airspeed_callback,offboard_qos)  #空速订阅
        self.servo_cli = self.create_client(CommandLong, "/mavros/cmd/command_long")
        self.vis_pub = self.create_subscription(TargetCoord,"/vision/custom_target",self.vision_callback,offboard_qos)
        
        self.offboard_requested = None  #offboard状态灯
        self.dj_reacher = False     #打击状态灯
        self.current_state = None   #订阅飞控状态
        self.current_pose = None    #当前位置信息
        self.distance = 0           #判断是否完成
        self.current_seq = 0        #当前航点序列
        self.current_airspeed = 0   #当前空速
        self.current_raw = []        #当前角度信息[roll,pitch,yaw]
        self.hit_point = None       #订阅得
        self.current_waypoints = [] #航线
        self.setpoint_counter = 0   #记数器
        self.future = None
        self.timer = self.create_timer(1.0 / 20.0,self.timer_callback)  # 10Hz频率，满足≥2Hz要求

        self.servo_ch = 7             #舵机硬件通道号
        self.pwm = 1100             #PWM值

        self.pose = PoseStamped()   #看门狗(位置)
        self.pose.pose.position.x = 300.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 25.0
        self.last_time = self.get_clock().now()
        self.T_msg = TwistStamped()
        self.T_msg.twist.linear.x = 15.0
        self.T_msg.twist.linear.y = 0.0
        self.T_msg.twist.angular.z = 0.0

        self.last_error_yaw = 0.0
        self.last_time_yaw = self.get_clock().now()

        self.get_logger().info('Waypoint Mission Manager initialized')
        self.wait_for_services()
    
    def wait_for_services(self):
        """Wait for all MAVROS services to become available."""
        self.get_logger().info('Waiting for MAVROS services...')

        services = [
            (self.push_client, 'WaypointPush'),
            (self.pull_client, 'WaypointPull'),
            (self.clear_client, 'WaypointClear'),
            (self.set_current_client, 'WaypointSetCurrent'),
            (self.set_mode_client, 'SetMode'),
        ]

        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name} service...')

        self.get_logger().info('All MAVROS services are available!')

    def waypoints_callback(self, msg: WaypointList):
        self.current_waypoints = msg.waypoints
        # self.get_logger().info(f'Received {len(self.current_waypoints)} waypoints from mission')

    def waypoint_reached_callback(self, msg: WaypointReached):
        self.current_seq = msg.wp_seq
        self.get_logger().info(f'Waypoint {msg.wp_seq + 1} reached!')

    def state_cb(self, msg: State):
        """State callback."""
        self.current_state = msg
    def local_pos_callback(self,msg:PoseStamped):
        self.current_pose = msg
        q = msg.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll,pitch,yaw = euler_from_quaternion(quaternion)
        self.current_raw = [roll,pitch,yaw]
    def normalize_angle(self,angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def airspeed_callback(self,msg:VfrHud):
        if msg.airspeed < 0.1 or msg.airspeed > 50.0:
            return
        self.current_airspeed = msg.airspeed
    def vision_callback(self,msg:TargetCoord):
        self.hit_point = msg


    def timer_callback(self):
        """启动 OFFBOARD 控制 - 先预热再切换模式"""
        con_seq = (self.current_seq < 4)
        con_reach = (self.dj_reacher is True)
        if con_seq or con_reach:
            return
            
        if self.setpoint_counter < 100:
            # 预热阶段：持续发送setpoint
            self.local_pos_pub.publish(self.pose)   #发布控制点
            #self.vel_pub.publish(self.T_msg)       #发布速度控制
            self.setpoint_counter += 1
            return

        current_time = self.get_clock().now()
        time_since_last_req = (current_time - self.last_time).nanoseconds / 1e9
           
        if self.current_state.mode != 'OFFBOARD' and time_since_last_req > 5.0:
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            req.base_mode = 0
            self.future = self.set_mode_client.call_async(req)
            #self.future.add_done_callback(self.set_mode_done_callback(self.future))

            if not self.offboard_requested:
                self.get_logger().info('OFFBOARD mode requested')
                self.offboard_requested = True
            self.last_time = current_time

        #self.local_pos_pub.publish(self.pose)
        
        self.offboard_dj(self.hit_point.p_x,self.hit_point.p_y,24.0)
        
        #self.offboard_dj(300.0,0.0,24.0)

    def set_mode_done_callback(self,future):
        """Set mode done callback."""
        response = future.result()
        if response.success:
            self.get_logger().info('成功进入offboard')
        else:
            self.get_logger().info('OFFBOARD:False')

    def release_point(self):  #计算投弹点
        free_fall_time = math.sqrt(2*self.current_pose.pose.position.z / 9.8)
        vertical_drag_factor = 0.2      #垂直阻力系数
        fall_time = free_fall_time*(1 + vertical_drag_factor*1) + 0.1
        vel_north_ideal = (self.hit_point.p_x - self.current_pose.pose.position.x) / fall_time
        vel_east_ideal = (self.hit_point.p_y - self.current_pose.pose.position.y) / fall_time
        speed_north = self.current_airspeed * math.cos(self.current_raw[2])
        speed_east = self.current_airspeed * math.sin(self.current_raw[2])
        north_error = speed_north - vel_north_ideal
        east_error = speed_east - vel_east_ideal
        Kp_north = 0.8
        Kp_east = 0.9
        dynamic_north = self.hit_point.p_x - self.current_pose.pose.position.x + Kp_north*north_error
        dynamic_east = self.hit_point.p_y - self.current_pose.pose.position.y + Kp_east*east_error
        final_north = max(0.0,min(dynamic_north,50.0))
        final_east = max(0.0,min(dynamic_east,50.0))
        return final_north,final_east
    
    def send_servo(self, channel: int, pwm: int):   #舵机控制
        """下发舵机控制指令 MAV_CMD_DO_SET_SERVO=183"""
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(channel)
        req.param2 = float(pwm)
        req.param3 = req.param4 = req.param5 = req.param6 = req.param7 = 0.0
        future = self.servo_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.success:
            self.get_logger().info(f"舵机{channel} PWM输出: {pwm}")
        else:
            self.get_logger().error(f"舵机指令下发失败 PWM={pwm}")
        return resp.success

    def offboard_dj(self, target_x: float, target_y: float, target_z: float):
        """
        针对动态目标优化的 OFFBOARD 控制
        改进点：
        1. 目标点低通滤波（平滑抖动）
        2. 动态前瞻距离（根据距离调整转弯半径）
        3. 速度矢量对齐（更平滑的航向计算）
        """
        if self.current_pose is None:
            return

        # 1. 获取当前状态
        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        pz = self.current_pose.pose.position.z
        current_yaw = self.current_raw[2]

        # 2. 目标点平滑处理 (低通滤波)
        # 防止YOLO识别框抖动导致飞机剧烈晃动
        # alpha越大，响应越快但越抖；alpha越小，越平滑但延迟越大
        alpha = 0.1
        if not hasattr(self, 'smoothed_target'):
            self.smoothed_target = [target_x, target_y, target_z]

        self.smoothed_target[0] = (1 - alpha) * self.smoothed_target[0] + alpha * target_x
        self.smoothed_target[1] = (1 - alpha) * self.smoothed_target[1] + alpha * target_y
        self.smoothed_target[2] = (1 - alpha) * self.smoothed_target[2] + alpha * target_z

        tx, ty, tz = self.smoothed_target

        # 3. 计算误差和距离
        dx = tx - px
        dy = ty - py
        dz = tz - pz
        horizontal_dist = math.sqrt(dx * dx + dy * dy)

        # 4. 到达判断与退出逻辑
        # 如果距离非常近，且是动态目标，建议不要立即切换模式，而是进入“盘旋/保持”
        # 这里保持原有的切换逻辑，但增加距离阈值判断
        if horizontal_dist < 10.0:
            self.get_logger().info('Target Reached (Dynamic Mode)')
            # 可以在这里添加盘旋逻辑，或者保持当前逻辑切换回任务模式
            # 为了防止在目标点附近反复横跳，这里仅打印日志，不立即切换，除非距离极近
            if horizontal_dist < 5.0:
                req = SetMode.Request()
                req.custom_mode = 'AUTO.MISSION'
                self.future = self.set_mode_client.call_async(req)
                self.dj_reacher = True
            return

        # 5. 动态前瞻距离计算 (Dynamic Lookahead)
        # 距离越远，前瞻点越远，防止转弯过急
        # 距离越近，前瞻点越近，为了对准目标
        min_lookahead = 15.0
        max_lookahead = 50.0
        # 简单的线性映射，或者根据速度调整
        lookahead_dist = min(max_lookahead, max(min_lookahead, horizontal_dist * 0.5))

        # 6. 计算期望航向与滚转角
        target_yaw = math.atan2(dy, dx)
        error_yaw = self.normalize_angle(target_yaw - current_yaw)

        # --- 核心改进：基于距离的滚转角限制 ---
        # 距离远时允许大角度转弯，距离近时必须小角度修正，防止失控
        max_roll_allowed = math.radians(45)  # 最大45度
        if horizontal_dist < 30.0:
            # 距离30米以内，线性减小最大允许滚转角，最低减到15度
            max_roll_allowed = math.radians(15) + (math.radians(30) * (horizontal_dist / 30.0))

        # --- PD 控制计算期望滚转 ---
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_yaw).nanoseconds / 1e9
        if dt <= 0: dt = 0.01

        # 这里的 Kp 可以适当调大，因为后面有限幅保护
        Kp_roll = 1.0
        Kd_roll = 0.3
        d_error = (error_yaw - self.last_error_yaw) / dt

        desired_roll = (Kp_roll * error_yaw) + (Kd_roll * d_error)

        # 应用动态限制
        desired_roll = max(-max_roll_allowed, min(max_roll_allowed, desired_roll))

        self.last_error_yaw = error_yaw
        self.last_time_yaw = current_time

        # 7. 俯仰角控制 (高度保持)
        # 增加死区，防止在目标高度附近频繁调整俯仰
        Kp_pitch = 0.05
        desired_pitch = dz * Kp_pitch
        # 限制俯仰角
        desired_pitch = max(math.radians(-15), min(math.radians(20), desired_pitch))

        # 8. 计算前瞻点坐标 (Lookahead Point)
        # 将设定点放在期望航向的前方，引导飞机“追逐”这个点
        # 这样飞机会有持续的前飞速度，不会悬停
        setpoint_yaw = target_yaw  # 直接使用目标航向作为期望机头朝向

        next_x = px + lookahead_dist * math.cos(setpoint_yaw)
        next_y = py + lookahead_dist * math.sin(setpoint_yaw)
        # 高度设定点：稍微超前一点，或者保持当前高度平滑过渡
        next_z = pz + dz * 0.1  # 只修正10%的高度误差，防止俯冲

        # 9. 构建 PoseStamped 消息
        q = quaternion_from_euler(desired_roll, desired_pitch, setpoint_yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # 或者是 "local_origin"，取决于你的TF树，通常local_origin更稳妥

        msg.pose.position.x = next_x
        msg.pose.position.y = next_y
        msg.pose.position.z = next_z

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        # 10. 发布
        self.local_pos_pub.publish(msg)
        '''
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z
        self.local_pos_pub.publish(msg)
        '''
        # 调试信息
        # self.get_logger().info(f'Dist: {horizontal_dist:.1f}, Roll: {math.degrees(desired_roll):.1f}, Lookahead: {lookahead_dist:.1f}')

    def load_plan_file(self, plan_file_path):#通过json载入航线信息

        try:
            with open(plan_file_path, 'r') as f:
                plan = json.load(f)
            if 'mission' not in plan or 'items' not in plan['mission']:
                raise ValueError('Invalid plan file format')
            items = plan['mission']['items']
            waypoints = []
            for i, item in enumerate(items):
                # 从params数组中提取坐标数据（QGroundControl格式）
                params = item.get('params', [])
                # params格式: [param1, param2, param3, param4, latitude, longitude, altitude]
                lat = params[4] if len(params) > 4 else 0.0
                lon = params[5] if len(params) > 5 else 0.0  
                alt = params[6] if len(params) > 6 else 0.0
                
                command = item.get('command', 16)
                frame = item.get('frame', 3)

                waypoint = self.create_waypoint(
                    lat,
                    lon,
                    alt,
                    frame,
                    command,
                )
                if i == 0:
                    waypoint.is_current = True
                else:
                    waypoint.is_current = False
                waypoints.append(waypoint)
            if waypoints:
                last_waypoint = waypoints[-1]
                last_waypoint.command = 21
                   
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from plan file')
            return waypoints
        
        except FileNotFoundError:
            self.get_logger().error(f'Plan file {plan_file_path} not found')
            return []
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON in plan file {plan_file_path}')
            return []
        except Exception as e:
            self.get_logger().error(f'Error loading plan file {plan_file_path}: {e}')
            return []

    def set_mode(self, mode: str = 'AUTO') -> bool:
        # """
        # Set the flight mode.

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
    ) -> Waypoint:    #手动导出航线
        # """
        # Create a MAVLink waypoint.

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
        
        if command == 22:
            wp.param1 = 15.0
        elif command == 16:
            wp.param1 = 0.0
        elif command == 21:
            wp.param1 = 0.0
        wp._param2 = 5.0
        return wp
    
    def create_sample_mission(
        self, home_lat: float = 47.397779, home_lon: float = 8.548065, home_alt: float = 0.0
    ) -> ty.List[Waypoint]:     #航点设置
        # """
        # Create a sample square mission with takeoff and landing.

        # :param home_lat: Home latitude
        # :param home_lon: Home longitude
        # :param home_alt: Home altitude
        # :returns: List of waypoints
        # """
        waypoints = []

        # # Waypoint 0: Home position (required)
        # wp_home = self.create_waypoint(
        #     home_lat,
        #     home_lon,
        #     home_alt,
        #     frame=0,  # GLOBAL
        #     command=16,  # WAYPOINT
        #     is_current=True,
        # )
        # waypoints.append(wp_home)

        # Waypoint 1: Takeoff
        wp_takeoff = self.create_waypoint(
            home_lat,
            home_lon,
            25.0,
            frame=3,  # GLOBAL_RELATIVE_ALT
            command=22,  # TAKEOFF
            param1=15.0,  # Pitch angle
            is_current=True,
        )
        waypoints.append(wp_takeoff)

        waypoints.append(self.create_waypoint(47.397687, 8.551507, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.396993, 8.552460, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.396066, 8.552197, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.395481, 8.551405, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.395752, 8.546942, 25.0, frame=3,command = 16,param2 = 5.0))

        waypoints.append(self.create_waypoint(47.396365, 8.546299, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.397481, 8.546483, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.398106, 8.547498, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.397985, 8.550824, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.397551, 8.551515, 25.0, frame=3,command = 16,param2 = 5.0))
        waypoints.append(self.create_waypoint(47.394671, 8.551008, 25.0, frame=3,command = 16,param2 = 5.0))
        

        #wp_approch，着陆进场点
        approach_lat = 47.394665
        approach_lon = 8.548873
        wp_approach = self.create_waypoint(
            approach_lat,approach_lon,15.0,
            frame=3,command=16
        )
        waypoints.append(wp_approach)

        # Waypoint 6: Land
        wp_land = self.create_waypoint(
            47.397506,
            8.548638,
            0.0,
            frame=3,
            command=21,  # LAND
        )
        waypoints.append(wp_land)

        return waypoints
    
    def push_waypoints(self, waypoints: ty.List[Waypoint]) -> bool:
        # """
        # Upload waypoints to the flight controller.

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
        import time

        time.sleep(2)

        manager.get_logger().info('=== Example 2: Basic Waypoint Mission ===')
        manager.get_logger().info('\n--- Clearing Existing Waypoints ---')
        manager.clear_waypoints()

        time.sleep(1)

        # Example 2: Create and push a sample mission
        manager.get_logger().info('\n--- Creating and Pushing Sample Mission ---')
        sample_mission = manager.create_sample_mission()
        manager.print_waypoint_info(sample_mission)
        manager.push_waypoints(sample_mission)
        #解析文件方式
        #sample_mission1 = manager.load_plan_file('/home/proxima/plan/plan.plan2')
        #manager.print_waypoint_info(sample_mission1)
        #manager.push_waypoints(sample_mission1)
        time.sleep(1)

        # Example 3: Pull waypoints from FCU
        manager.get_logger().info('\n--- Pulling Waypoints from FCU ---')
        pulled_waypoints = manager.pull_waypoints()
        if pulled_waypoints:
            manager.print_waypoint_info(pulled_waypoints)

        time.sleep(1)

        # Example 4: Set a specific waypoint as current (skip to waypoint 2)
        manager.get_logger().info('\n--- Setting Current Waypoint ---')
        manager.set_current_waypoint(0)

        time.sleep(1)

        manager.set_mode('AUTO.MISSION')
        manager.get_logger().info('Switching to AUTO.MISSION...')
        time.sleep(1)

        manager.get_logger().info('Waiting for FCU connection...')
        
        while rclpy.ok() and not manager.current_state.connected:
            rclpy.spin_once(manager, timeout_sec=0.1)
        manager.get_logger().info('FCU connected! Starting OFFBOARD control...')
        rclpy.spin(manager)

    except KeyboardInterrupt:
        manager.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
