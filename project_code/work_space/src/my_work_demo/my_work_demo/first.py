from rclpy.node import Node
import json
import typing as ty
import rclpy
import time
from mavros_msgs.msg import Waypoint, WaypointList, State,  WaypointReached,State
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointPull, WaypointSetCurrent , SetMode, CommandBool, CommandLong 
from rclpy.qos import HistoryPolicy , QoSProfile , ReliabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import NavSatFix


class FixedWingPlanner(Node):
    def __init__(self):
        super().__init__('fixed_wing_planner')

        # QoS profile for reliable communication
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

        #初始化服务(包含：清除航线，上传航线，下载航线，设置模式，当前航点序数)
        self.clear_mission_client = self.create_client(WaypointClear,'mavros/mission/clear')
        self.push_mission_client = self.create_client(WaypointPush,'mavros/mission/push')
        self.pull_mission_client = self.create_client(WaypointPull,'mavros/mission/pull')
        self.set_current_client = self.create_client(WaypointSetCurrent,'mavros/mission/set_current')        
        self.set_mode_client = self.create_client(SetMode,'mavros/set_mode')
        self.command_client = self.create_client(CommandLong,'mavros/cmd/command')

        #初始化话题(包括：1.检查是否到达当前航点 2.查看当前航线 3.查看当前的GPS坐标)
        self.waypoints_reached_sub = self.create_subscription(WaypointReached,'mavros/mission/reached',self.waypoint_reached_callback,qos_profile )
        self.waypoints_sub = self.create_subscription(WaypointList,'mavros/mission/waypoints',self.waypoints_callback,qos_profile )
        self.global_position_sub = self.create_subscription(NavSatFix,'mavros/global_position/global',self.global_position_callback,sensor_qos_profile )
        self.state_sub = self.create_subscription(State,'/mavros/state',self.state_callback,qos_profile)
        self.secondary_waypoints_sub = self.create_subscription(WaypointList,'/secondary_waypoints',self.secondary_callback,qos_profile)

        # 三段航点文件路径
        self.primary_path = "/home/qian/project_code/work_space/src/waypoints/dacaocahang/waypoints_01_A.waypoints"
        self.secondary_path = None
        self.third_path = "/home/qian/project_code/work_space/src/waypoints/dacaocahang/waypoints_01_C.waypoints"

        # 舵机设置
        self.servo_channel= 6
        self.get_logger().info(f"程序启动，初始化舵机（通道{self.servo_channel},PWM=1000）...")
        self.set_servo_to_channel(1000)

        self.last_reach_waypoint_seq = None                     #当前航点的序号
        self.current_global_position = None
        self.current_waypoints = []                             #当前航线
        self.current_state = State()

        self.primary_waypoints = []
        self.secondary_waypoints = []
        self.third_waypoints = []

        self.primary_waypoints_completed = False
        self.secondary_waypoints_completed = False
        self.third_waypoints_completed = False

#回调函数 1.判断是否到达航点 2.读取当前航线 3.读取当前世界坐标 4.读取当前的飞行器状态 5.订阅第二航线
    def waypoint_reached_callback(self,msg:WaypointReached):
        self.last_reach_waypoint_seq = msg.wp_seq
        self.get_logger().info(f'到达第{self.last_reach_waypoint_seq}个航点')

        #补充航线更换逻辑
        if self.primary_waypoints_completed == False and self.secondary_waypoints_completed == False:
            #此处以len - 2 作为判断条件是因为在仿真中，飞控只认可有起降点的航线，因此减2来跳过降落点
            if self.last_reach_waypoint_seq == len(self.current_waypoints) - 2:
                self.primary_waypoints_completed = True

                self.safely_change_mode('loiter_unlim')
                self.current_waypoints = self.secondary_waypoints
                self.push_waypoints_retry(self.current_waypoints)
                self.set_current_waypoint(0)
                self.safely_change_mode('auto')

        if self.primary_waypoints_completed == True and self.secondary_waypoints_completed == False:
            if self.last_reach_waypoint_seq == len(self.current_waypoints) - 2:
                self.secondary_waypoints_completed = True

                self.safely_change_mode('loiter_unlim')
                self.current_waypoints = self.third_waypoints
                self.push_waypoints_retry(self.current_waypoints)
                self.set_current_waypoint(0)
                self.safely_change_mode('auto')

    def waypoints_callback(self,msg:WaypointList):
        self.current_waypoints = msg.waypoints
        self.get_logger().info(f'成功接收到{len(self.current_waypoints)}个航点')

    def global_position_callback(self,msg:NavSatFix):
        self.current_global_position = (msg.latitude,msg.longitude,msg.altitude)

    def state_callback(self,msg:State):
        self.current_state = msg

    def secondary_callback(self,msg:WaypointList):
        self.secondary_waypoints = msg
        self.get_logger().info(f'接收到第二段航线')

        #如果第一条航线完成，执行第二条航线
        # if self.primary_waypoints_completed:
        #     self.current_waypoints = self.secondary_waypoints
        #     self.last_reach_waypoint_seq = None

        #     self.get_logger().info(f'打断第一段航线，准备执行第二段航线')

        #     # 关键修复1：强制切换LOITER模式（不超时则重试，确保切换成功）
        #     if not self.safely_change_mode("LOITER_UNLIM",max_retries=5,timeout=15):
        #         self.get_logger().info(f'模式切换失败，无法安全打断航线')
        #         return
            
        #     # 关键修复2：为第二段航点设置“起始标识”（第一个航点设为当前航点）
        #     if self.secondary_waypoints:
        #         # 重置所有航点的is_current，仅第一个设为True
        #         for i, wp in enumerate(self.secondary_waypoints):
        #             wp.is_current = (i == 0)  # 只有第一个航点标记为“当前起始”
        #         self.get_logger().info("已为第二段航点设置起始标识（第一个航点is_current=True）")

        #     # 清理旧航点、上传新航点
        #     self.get_logger().info("【LOITER模式】清理第一段航点，上传第二段航点...")
        #     if not self.clear_mission(max_retries=5):  # 增加清理重试
        #         self.get_logger().errror("清理旧航点失败，终止切换")
        #         return
        #     self.current_waypoints = self.secondary_waypoints

        #     if not self.push_waypoints(self.current_waypoints, max_retries=5):
        #         self.get_logger().error("第二段航点上传失败，回退至第一段任务（若可能）")
        #         return
                
        #     # 关键修复3：切换回AUTO前，等待1-2秒确保飞控加载新航点
        #     self.get_clock().sleep_for(Duration(seconds=2))
        #     if not self.set_mode("AUTO", max_retries=3):
        #         self.get_logger().error("切换回AUTO模式失败，请手动切换！")
        #     else:
        #         self.get_logger().info(f"【第二段任务启动】共{len(self.current_waypoints)}个航点（已切回AUTO）")
                
#请求服务 1.等待所有服务链接 2.清除任务 3.上传航线 4.拉取航点 5.设置开始执行的坐标的序号 6.设置模式 7.发布mavlink的指令（详情见官网）
    def wait_for_service(self):
        """等待所有服务连接"""

        services = [
            (self.push_mission_client, 'WaypointPush'),
            (self.pull_mission_client, 'WaypointPull'),
            (self.clear_mission_client, 'WaypointClear'),
            (self.set_current_client, 'WaypointSetCurrent'),
            (self.set_mode_client, 'SetMode'),
            (self.command_client, 'CommandLong')
        ]

        for client , name in services:
            try:
                client.wait_for_service(timeout_sec=10.0)
                self.get_logger().info(f'Waiting for {name} service...')

            except Exception as e:
                self.get_logger().info(f"服务 {name} 等待超时: {e}")
                exit(1)
    
    def clear_mission(self):
        """"清空任务"""
        request = WaypointClear.Request()

        self.get_logger().info('清除所有航线...')

        future = self.clear_mission_client.call_async(request)
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
    
    def clear_mission_retry(self,max_retries = 3 , retry_delay = 3) -> bool:
        for attempt in range(max_retries):
            if self.clear_mission():  # 调用你已有的 clear_mission
                return True
            self.get_logger().warning(f"清空任务失败（第{attempt+1}/{max_retries}次），{retry_delay}秒后重试")
            self.get_clock().sleep_for(Duration(seconds=retry_delay))
        self.get_logger().error(f"清空任务失败，已重试{max_retries}次")
        return False

    def push_waypoints(self, waypoints: ty.List[Waypoint]) -> bool:
        """"上传航线到飞控"""
        """"当航线不对时，可以下载下来并打印，来检查是否是航线的问题"""
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints

        self.get_logger().info(f'Pushing {len(waypoints)} waypoints to FCU...')

        future = self.push_mission_client.call_async(request)
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
        
    def push_waypoints_retry(self,waypoints ,max_retries = 3 , retry_delay = 3) -> bool:
        for attempt in range(max_retries):
            if self.push_waypoints(waypoints):  
                return True
            self.get_logger().warning(f"上传任务失败（第{attempt+1}/{max_retries}次），{retry_delay}秒后重试")
            self.get_clock().sleep_for(Duration(seconds=retry_delay))
        self.get_logger().error(f"上传任务失败，已重试{max_retries}次")
        return False

    def pull_waypoints(self) -> ty.Optional[ty.List[Waypoint]]:
        """"从飞控中下载航线，检查航线"""
        request = WaypointPull.Request()

        self.get_logger().info('Pulling waypoints from FCU...')

        future = self.pull_mission_client.call_async(request)
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

    def set_current_waypoint(self, wp_seq: int) -> bool:
        """设置当前执行的点"""
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

    def set_mode(self, mode: str = 'AUTO') -> bool:
        """设置飞行模式"""

        # :param mode: Flight mode string (e.g., 'AUTO', 'GUIDED', 'STABILIZE', 'LOITER')
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
        
    def command(self,command_num,param1 = 0.0,param2 = 0.0,param3 = 0.0,param4 = 0.0,param5 = 0.0,param6 = 0.0,param7 = 0.0):
        """发布mavlink命令"""
        request = CommandLong.Request()
        request.command = command_num
        request.param1 = float(param1)
        request.param2 = float(param2)
        request.param3 = float(param3)
        request.param4 = float(param4)
        request.param5 = float(param5)
        request.param6 = float(param6)
        request.param7 = float(param7)

        future = self.command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'成功发送指令{request.command}')
                return True
            else:
                self.get_logger().error(f'指令{request.command}发送失败')
                return False
        else:
            self.get_logger().error(f'指令{request.command}发送失败')
            return False

        
# 实现任务
# 强制开机 -> 切换模式为自动 -> 执行航线primary -> 执行航线secondary ->调整舵机pwm -> 执行航线third 
    def force_arm(self,max_retries = 3,retry_delay = 5):
        """"强制解锁"""
        #最大尝试解锁次数；最大等待时间
        self.get_logger().info("=== 执行强制解锁预处理：允许忽略部分安全限制 ===")
        for attempt in range(max_retries):
            try:
                # MAV_CMD_COMPONENT_ARM_DISARM 命令参数说明：
                # param1: 1=解锁, 0=上锁; param2: 21196=允许强制解锁（关键参数）
                resp = self.command(400,param1=1,param2=21196)
                if resp:
                    self.get_logger().info(f"强制解锁预处理成功（第{attempt+1}次尝试）")
                    # 等待飞控处理指令（必要延迟）
                    self.get_clock().sleep_for(Duration(seconds=1.0))
                    return True
                else:
                    self.get_logger().warning(f"第{attempt+1}次强制解锁预处理失败")
            except Exception as e:
                self.get_logger().warning(f"第{attempt+1}次强制解锁预处理服务调用失败：{e}")
            
            self.get_clock().sleep_for(Duration(seconds=retry_delay))
        
        self.get_logger().error(f"连续{max_retries}次强制解锁预处理失败，无法继续强制解锁")
        return False
    
    def wait_for_remote_auto_mode(self,timeout_seconds = 60):
        """等待遥控器切换到AUTO模式（通过监听飞控状态实现）"""
        self.get_logger().info("等待遥控器切换到AUTO模式...")
        
        rate = self.create_rate(5)
        start_time = self.get_clock().now()

        last_log_time = start_time

        while rclpy.ok():
            current_time = self.get_clock().now()

        # 1. 检查超时
            if (current_time - start_time).nanoseconds / 1e9 > timeout_seconds:
                self.get_logger().error("等待遥控器切换AUTO模式超时，请检查遥控器或飞控设置")
                return False

        # 2. 检查当前模式是否已切换为 AUTO
            if self.current_state.mode == "AUTO":
                self.get_logger().info("检测到遥控器已切换至AUTO模式，开始执行任务")
                return True

        # 3. 每2秒提示一次，避免日志刷屏（更精确的节流写法）
            if (current_time - last_log_time).nanoseconds / 1e9 >= 2.0:
                self.get_logger().info("请通过遥控器将飞行模式切换至AUTO以启动任务...")
                last_log_time = current_time

        # 保持循环频率
            rate.sleep() 

        return False

#辅助函数     1.舵机控制  2.安全切换模式 3.解析json文件,下载航线 
    def set_servo_to_channel(self,pwm_value):
        """"舵机控制函数"""
        if not (1000 <= pwm_value <= 2000):
            self.get_logger().info(f'pwm值需要在1000~2000,当前值：{pwm_value}')

        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = self.command(
                    command=183,  # MAV_CMD_DO_SET_SERVO
                    param1=self.servo_channel,
                    param2=pwm_value
                )
                
                if response.success:
                    self.get_logger().info(f"[通道{self.servo_channel}] 成功设置PWM={pwm_value}（第{attempt+1}次尝试）")
                    return True
                else:
                    self.get_logger().warning(f"[通道{self.servo_channel}] 第{attempt+1}次尝试失败，错误码：{response.result}")
                
            except Exception as e:
                self.get_logger().warning(f"[通道{self.servo_channel}] 服务调用失败：{e}")
                
            self.get_clock().sleep_for(Duration(seconds=0.5))
        
        self.get_logger().error(f"[通道{self.servo_channel}] 连续{max_retries}次设置失败，请检查参数SERVO{self.servo_channel}_FUNCTION是否为0")
        return False

    def safely_change_mode(self, mode, max_retries=3, retry_delay=2, wait_for_stable=True, timeout=10):
        #尝试多次切换模式
        for attempt in range(max_retries):  #删除外层的 try...except（移到内部）
            self.get_logger().info(f'第{attempt + 1}次尝试切换至{mode}模式')
        
            try:  #只包裹 set_mode 调用
                resq = self.set_mode(mode)
            except Exception as e: 
                self.get_logger().warning(f"模式切换服务调用失败: {e}，将在 {retry_delay} 秒后重试")
                self.get_clock().sleep_for(Duration(seconds=retry_delay))
                continue  #继续下一次重试
        
            if not resq.mode_sent:
                self.get_logger().warning(f'模式切换命令发送失败（飞控未接收），将在 {retry_delay} 秒后重试')
                self.get_clock().sleep_for(Duration(seconds=retry_delay))
                continue

            if wait_for_stable:
                rate = self.create_rate(5)
                start_time = self.get_clock().now()

                while rclpy.ok():
                    elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                    if elapsed > timeout:
                        self.get_logger().warning(f"模式切换超时（{timeout}s），当前模式: {self.current_state.mode}")
                        break  #跳出 while，进入下一次重试
                
                    # 忽略大小写判断是否已切换成功
                    if self.current_state.mode.lower() == mode.lower():  #统一转为小写比较
                        self.get_logger().info(f"成功切换至 {self.current_state.mode} 模式，状态稳定")
                        #修复3：LOITER 拼写（你写成了 LOITER_UNTIME）
                        if mode.lower() == "loiter" or mode.lower() == "loiter_unlim":
                            self.get_clock().sleep_for(Duration(seconds=1))
                            self.get_logger().info(f"{self.current_state.mode} 模式已稳定，可执行后续操作")
                        return True
                    rate.sleep()
            
                # 如果 while 循环因为超时而 break，继续下一次重试
                self.get_logger().warning(f"第{attempt+1}次切换 {mode} 失败（超时或未稳定），{retry_delay}秒后重试")
                self.get_clock().sleep_for(Duration(seconds=retry_delay))
                continue  # 继续下一次重试
        
            # 如果 wait_for_stable=False，直接返回成功
            self.get_logger().info(f"{mode} 模式切换命令已发送（未等待稳定）")
            return True
    
        # 所有重试都失败
        self.get_logger().error(f"经过 {max_retries} 次尝试后，仍无法切换至 {mode} 模式")
        return False

    def load_waypoints_from_wpl(self, file_path: str) -> ty.List[Waypoint]:
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
        
            # ===== 关键修复：处理空家点（seq=0 且坐标全0） =====
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

#运行函数
    def run(self):
        self.get_logger().info(f'服务准备中.......')

        self.wait_for_service()

        self.primary_waypoints = self.load_waypoints_from_wpl(self.primary_path)

        while len(self.primary_waypoints) == 0 :
            self.get_logger().warning(f'下载primary失败，正在重新下载')
            self.primary_waypoints = self.load_waypoints_from_wpl(self.primary_path)

        self.third_waypoints = self.load_waypoints_from_wpl(self.third_path)

        while len(self.third_waypoints) == 0:
            self.get_logger().warning(f'下载third失败，正在重新下载')
            self.third_waypoints = self.load_waypoints_from_wpl(self.third_path)


        self.get_logger().info(f'primary下载成功，下一步清空任务并上传primary')

        self.clear_mission_retry()
        
        self.push_waypoints_retry(self.primary_waypoints)

        self.get_logger().info(f'准备解锁')
        self.force_arm()

        rate = self.create_rate(1)
        last_throttle_log_time = self.get_clock().now()
        while rclpy.ok():
            if self.third_waypoints_completed:
                self.get_logger().info("所有三段任务已完成")
                break

            if self.primary_waypoints_completed and not self.secondary_waypoints_completed:
                current_time = self.get_clock().now()
                if (current_time - last_throttle_log_time).nanoseconds / 1e9 >= 5.0:
                    self.get_logger().info("第一段已完成，等待外部发送第二段航点（话题：/secondary_waypoints）...")
                    last_throttle_log_time = current_time
        
            rate.sleep()
        
        self.get_logger().info("固定翼规划节点所有任务完成")    

#相关的辅助信息        
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
    manager = FixedWingPlanner()
    manager.run()
    rclpy.shutdown()










