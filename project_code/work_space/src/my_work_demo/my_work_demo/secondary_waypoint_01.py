import rclpy
import math
import json
from pathlib import Path
from collections import deque
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from tf_transformations import euler_from_quaternion
import typing as ty

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import Waypoint, WaypointList, VFR_HUD
from mavros_msgs.srv import WaypointPush, WaypointClear, CommandLong

#投弹的整体流程：获取目标点后，通过反向推理，计算在空气阻力的影响下，如何让弹药命中目标
#飞行器从起飞区方向进入打击区，借助 len_of_sight 距离对准目标

class CalculateSecondaryWaypoints01(Node):
    def __init__(self):
        super().__init__('calculate_secondary_waypoints_01')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.secondary_waypoints_pub = self.create_publisher(WaypointList, '/secondary_waypoints', qos_profile)

        #订阅传感器
        #订阅gps信息
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global',self.gps_callback, qos_profile)
        # 空速 
        self.vfr_hud_sub = self.create_subscription(VFR_HUD, '/mavros/vfr_hud',self.airspeed_callback, qos_profile)
        # 地速 (ENU坐标系下相对于地面的速度)
        self.velocity_sub = self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',self.velocity_callback, qos_profile)
        # 里程计 (位姿+姿态四元数 → 欧拉角)
        self.odom_sub = self.create_subscription( Odometry, '/mavros/local_position/odom',self.odometry_callback, qos_profile)

        # 传感器数据存储
        self.current_gps = 0.0  # 当前GPS位置
        self.current_airspeed = 0.0  # 当前空速(m/s，机体坐标系x轴)
        self.last_heading = 0.0  # 平滑后的航向角（度）

        #地面速度
        self.ground_speed_east = 0.0
        self.ground_speed_north = 0.0

        # 风速风向数据（ENU坐标系）
        self.wind_east = 0.0       # 东向风速分量（m/s，正=向东）
        self.wind_north = 0.0      # 北向风速分量（m/s，正=向北）
        self.wind_speed = 0.0      # 风速大小（m/s）
        self.wind_direction = 0.0  # 风向（风的来向，0°=正北，90°=正东）

        #飞行器姿态,滚转，俯仰，偏航
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0    # 航向角（弧度，大地坐标系：0=正北，顺时针增加）

        #存储目标位置的文件路径
        self.target_pos_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/target_pos.json"
        self.target_pos_backup_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/target_pos_backup.json" 
        self.target_pos = None
        self.target_lon = None
        self.target_lat = None

        #返航航线文件路径（QGC WPL格式，从Mission Planner导出）
        self.return_pos_path = "/home/qing/Duidi_Code_26/project_code/work_space/src/return_pos.waypoints"

        #辅助点坐标（起飞区中标定点，用于计算打击方位角）
        self.declare_parameter('fuzhu_lat', 0.0)
        self.fuzhu_lat = self.get_parameter('fuzhu_lat').value
        self.declare_parameter('fuzhu_lon', 0.0)
        self.fuzhu_lon = self.get_parameter('fuzhu_lon').value
        
        #风速历史队列
        self.declare_parameter('wind_history_size', 10)
        self.wind_history_size = self.get_parameter('wind_history_size').value  # 风速存储
        self.wind_history = deque(maxlen=self.wind_history_size)                  # deque队列是比list相似的数据结构，可以存储指定数量的元素，超出的元素会自动删除

        #目标位置历史队列
        self.declare_parameter('target_history_size', 6)
        self.target_history_size = self.get_parameter('target_history_size').value
        self.target_history = deque(maxlen=self.target_history_size)
        self.declare_parameter('target_tolerance', 0.8)
        self.target_tolerance = self.get_parameter('target_tolerance').value             #在此处用到的get_parameter_or()函数,可以做到在启动文件时修改参数

        #弹药的基本数据
        self.ammo_M = 0.5           #弹药质量
        self.ammo_CD = 0.82         #将水瓶视为一个圆柱体，圆柱体在空气中的阻力系数为0.82
        self.ammo_AREA = 0.014      #弹药迎风面面积（下落过程中，弹药大概率是翻滚下落，取侧面投影为迎风面）

        #航线参数
        self.declare_parameter('len_of_sight', 80.0)     #对准距离（米），飞行器沿打击方向反向延伸的距离，用于对准目标
        self.len_of_sight = self.get_parameter('len_of_sight').value
        self.aim_waypoint = 5                            #对准段航点数量

        #偏移量
        self.bearing_offset = 0                         #方向偏移量


#回调函数
    def gps_callback(self, msg: NavSatFix) -> None:
        self.current_gps = [msg.latitude, msg.longitude,msg.altitude]

    def airspeed_callback(self, msg: VFR_HUD) -> None:
        #空速回调
        raw_airspeed = msg.airspeed   #raw_airspeed是未过滤的airspeed

        min_airspeed = 0.0
        max_airspeed = 50.0

        if min_airspeed < raw_airspeed < max_airspeed:
            self.current_airspeed = raw_airspeed
        else:
            self.get_logger().warning(f"空速异常: {raw_airspeed:.2f}m/s，忽略此次采样",
                                      throttle_duration_sec=5.0)

    def velocity_callback(self, msg: TwistStamped) -> None:
        #地速回调
        #ground_speed_dir 地速方向 0度为正北，90度为正东，180度为正南，270度为正西
        #hypot 函数是计算三角形斜边的长度
        #atan2 函数是计算从一点到另一点的方位角
        self.ground_speed_east = msg.twist.linear.x
        self.ground_speed_north = msg.twist.linear.y
        ground_speed = math.hypot(self.ground_speed_east, self.ground_speed_north)
        ground_speed_dir = math.degrees(math.atan2(self.ground_speed_east, self.ground_speed_north)) % 360
        # self.current_airspeed().info(
            # f"地速 - 东向: {self.ground_speed_east:.2f} | 北向: {self.ground_speed_north:.2f} | 大小: {ground_speed:.2f}m/s | 方向: {ground_speed_dir:.1f}°")
        
    def odometry_callback(self, msg: Odometry) -> None:
        # 获取姿态四元数
        quaternion = msg.pose.pose.orientation
        # 欧拉角
        self.roll,self.pitch,self.yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        # 航向角（弧度，大地坐标系：0=正北，顺时针增加）
        # self.get_logger().info(f"滚转角: {math.degrees(self.roll):.2f}° | 俯仰角：{math.degrees(self.pitch):.2f}° | 偏航角：{math.degrees(self.yaw):.2f}°")

#风速计算
    #在ENU坐标系下，计算空速分向量 (方向=机头yaw)
    def calculate_airspeed_enu(self):
        airspeed = self.current_airspeed
        if airspeed < 0.1:
            return 0.0, 0.0  # 空速太小，返回零向量
    
        airspeed_east = airspeed * math.sin(self.yaw)
        airspeed_north = airspeed * math.cos(self.yaw)
    
        return airspeed_east, airspeed_north
    
    def calculate_wind(self):
        if self.current_airspeed < 0.1:
            self.get_logger().info("空速过低（<0.1m/s），无法准确计算风速")
            if not hasattr(self, 'wind_east'):
                self.wind_east = 0.0
                self.wind_north = 0.0
                self.wind_speed = 0.0
            self.wind_history.append((self.wind_east, self.wind_north, self.wind_speed))
            return

        airspeed_east, airspeed_north = self.calculate_airspeed_enu()
        current_wind_east = self.ground_speed_east - airspeed_east
        current_wind_north = self.ground_speed_north - airspeed_north
        current_wind_speed = math.hypot(current_wind_east, current_wind_north)

        max_reasonable_wind = 50.0
        if current_wind_speed > max_reasonable_wind:
            self.get_logger().warning(f"极端异常风速({current_wind_speed:.2f}m/s)，使用上一次有效值")
            if self.wind_history:
                current_wind_east, current_wind_north, current_wind_speed = self.wind_history[-1]
            else:
                current_wind_east = 0.0
                current_wind_north = 0.0
                current_wind_speed = 0.0

        self.wind_history.append((current_wind_east, current_wind_north, current_wind_speed))
        avg_east = sum(w[0] for w in self.wind_history) / len(self.wind_history)            # 东方向的平均风速
        avg_north = sum(w[1] for w in self.wind_history) / len(self.wind_history)           # 北方向的平均风速
        avg_speed = math.hypot(avg_east, avg_north)                                         # 平均风速

        #将平均风速保存为属性
        self.wind_east = avg_east
        self.wind_north = avg_north
        self.wind_speed = avg_speed

        log_msg = (
            f"瞬时风速: 东向 {current_wind_east:.4f}, 北向 {current_wind_north:.4f} → 大小 {current_wind_speed:.4f}m/s\n"
            f"{len(self.wind_history)}/{self.wind_history_size}次平均风速: 东向 {self.wind_east:.4f}, 北向 {self.wind_north:.4f} → 大小 {self.wind_speed:.4f}m/s"
        )
        self.get_logger().info(log_msg)


#辅助函数
    #从json文件中读取航点
    def load_plan_file(self, file_path: str) -> ty.List[Waypoint]:
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)

            pos = data['best_gps_position']
            self.target_lat = float(pos[0])
            self.target_lon = float(pos[1])
            self.target_pos = (self.target_lat, self.target_lon)

    #从QGC WPL文件中读取返航航点（盘旋掉头 + 降落）
    def load_return_waypoints(self) -> ty.List[Waypoint]:
        waypoints = []
        try:
            with open(self.return_pos_path, 'r', encoding='utf-8') as file:
                for line in file:
                    line = line.strip()
                    if not line or line.startswith('QGC'):
                        continue
                    parts = line.split()
                    if len(parts) < 12:
                        continue
                    wp = Waypoint()
                    wp.is_current = False
                    wp.frame = int(parts[2])
                    wp.command = int(parts[3])
                    wp.param1 = float(parts[4])
                    wp.param2 = float(parts[5])
                    wp.param3 = float(parts[6])
                    wp.param4 = float(parts[7])
                    wp.x_lat = float(parts[8])
                    wp.y_long = float(parts[9])
                    wp.z_alt = float(parts[10])
                    wp.autocontinue = bool(int(parts[11]))
                    waypoints.append(wp)
        except FileNotFoundError:
            self.get_logger().error(f"返航航线文件不存在: {self.return_pos_path}")
        return waypoints

    #判断目标是否稳定
    def is_target_stable(self, target) -> bool:
        if not target:
            self.target_history.clear()
            return False
        self.target_history.append(target)
        if len(self.target_history) < self.target_history_size:
            return False
        ref_lat, ref_lon = self.target_history[0]
        max_dist = 0.0
        for (lat, lon) in list(self.target_history)[1:]:
            dist = self.calculate_gps_distance(ref_lat, ref_lon, lat, lon)
            max_dist = max(max_dist, dist)
        return max_dist <= self.target_tolerance

    #用GPS坐标计算两点间米为单位的距离
    def calculate_gps_distance(self, lat1, lon1, lat2, lon2) -> float:
        R = 6378137.0  # 地球半径（米）
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)             #radians函数是将角度转为弧度
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        #这是 Haversine（半正矢）公式 的核心部分，用于计算球面上两点之间的距离。数学表达： a = sin²(Δφ/2) + cos(φ₁) × cos(φ₂) × sin²(Δλ/2)       
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad)*math.cos(lat2_rad)*math.sin(dlon/2)** 2        

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    #计算从一点向某方向偏移一定距离的点的GPS坐标
    def calculate_offset_point(self, lat, lon, bearing, distance):
        R = 6378137.0  # 地球半径（米）
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        bearing_rad = math.radians(bearing)
        dist_rad = distance / R
        
        new_lat_rad = math.asin(
            math.sin(lat_rad)*math.cos(dist_rad) +
            math.cos(lat_rad)*math.sin(dist_rad)*math.cos(bearing_rad)
        )
        new_lon_rad = lon_rad + math.atan2(
            math.sin(bearing_rad)*math.sin(dist_rad)*math.cos(lat_rad),
            math.cos(dist_rad) - math.sin(lat_rad)*math.sin(new_lat_rad)
        )
        new_lon_rad = (new_lon_rad + 3*math.pi) % (2*math.pi) - math.pi  # 归一化到[-π, π]
        return (math.degrees(new_lat_rad), math.degrees(new_lon_rad))    #返回目标点的GPS坐标

#计算方向角相关的数据  方向角0度是北，90度是东
    #在基础方向上加上偏移量，得到实际飞行方向
    def get_actual_bearing(self, base_bearing):
        """计算实际航线方向（基础方向+偏移量）"""
        actual_bearing = (base_bearing + self.bearing_offset) % 360
        self.get_logger().info(f"基础方向：{base_bearing:.1f}° + 偏移量：{self.bearing_offset:.1f}° → 实际方向：{actual_bearing:.1f}°")
        return actual_bearing
    
    #计算从一点到另一点的方向角
    def calculate_bearing(self,start_lat, start_lon, target_lat, target_lon):
        lat1, lon1 = math.radians(start_lat), math.radians(start_lon)
        lat2, lon2 = math.radians(target_lat), math.radians(target_lon)
        dlon = lon2 - lon1
        
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad) % 360
        return bearing_deg
    
    def calculate_release_point(self):
        """计算释放点"""
        """要考虑到瓶子的形状、下落状态、风速"""  
#需要添加判断打击目标坐标是否稳定

        k = 0.5 * 1.225 * self.ammo_CD * self.ammo_AREA / self.ammo_M           #将所有与阻力有关的常数打包成一个阻力系数

        h = 35.0
        v_z = 0.0               #竖直方向的速度
        v_north = self.ground_speed_north           #北方向的速度
        v_east = self.ground_speed_east             #东方向的速度

        pos_north = 0.0         #北方向的偏移量
        pos_east = 0.0          #东方向的偏移量


        dt = 0.02

        #计算将阻力抵消后的等效目标点
        while h > 0:
            #积分：利用当前状态求加速度
            a_z = -9.8 - k * v_z * abs (v_z)

            #danyaoxiangduiyufengdesudu
            rel_v_north = v_north - self.wind_north
            rel_v_east = v_east - self.wind_east
            rel_v = math.sqrt(rel_v_north * rel_v_north + rel_v_east * rel_v_east)

            if rel_v > 0 :
                a_north =  - k * rel_v * rel_v_north
                a_east = - k * rel_v * rel_v_east

            else:
                a_north = a_east = 0

            #RK2半步中点法
            v_z_mid = v_z + a_z * dt/2 
            v_north_mid = v_north + a_north * dt / 2
            v_east_mid = v_east + a_east * dt / 2
            
            #计算中点处a_z2 a_north2 a_east2
            a_z2 = - 9.8 - k  * v_z_mid * abs( v_z_mid )

            rel_v_north_mid = v_north_mid - self.wind_north
            rel_v_east_mid = v_east_mid - self.wind_east
            rel_v_mid = math.sqrt(rel_v_north_mid * rel_v_north_mid + rel_v_east_mid * rel_v_east_mid)

            if rel_v_mid > 0:
                a_north2 = -k * rel_v_mid * rel_v_north_mid
                a_east2 = -k * rel_v_mid * rel_v_east_mid
            else:
                a_north2 = 0.0
                a_east2 = 0.0

            #对加速度积分
            v_z += a_z2 * dt
            v_north += a_north2 * dt
            v_east += a_east2 * dt

            #对速度积分
            h += v_z * dt
            pos_north += v_north * dt
            pos_east += v_east * dt

        #atan2 是一个数学函数，用于计算从原点(0,0)到点(x,y)的方位角（即与正x轴的夹角）。
        #hypot 是 "hypotenuse"（斜边） 的缩写，用于计算直角三角形的斜边长度，即欧几里得距离。
        drift_dist = math.hypot(pos_north, pos_east)
        drift_bearing = math.degrees(math.atan2(pos_east, pos_north)) % 360
        release_bearing = (drift_bearing + 180.0) % 360
        new_target_lat ,new_target_lon=   self.calculate_offset_point(
            self.target_lat , self.target_lon, release_bearing, drift_dist)

        return new_target_lat,new_target_lon
        
    def calculate_approach_waypoints(self, release_pt):
        release_lat, release_lon = release_pt

        fly_h = 35.0

        # 打击方向：从辅助点（起飞区）指向目标，飞行器沿此方向进入打击区
        strike_bearing = self.calculate_bearing(
            self.fuzhu_lat, self.fuzhu_lon, self.target_lat, self.target_lon
        )

        # 接近方向：沿打击方向飞向释放点
        approach_bearing = strike_bearing

        # 接近起点，从释放点沿接近方向反向偏移 len_of_sight 米，保证有足够距离对准目标
        start_lat, start_lon = self.calculate_offset_point(
            release_lat, release_lon, (approach_bearing + 180.0) % 360, self.len_of_sight
        )

        waypoints = []

        # 对准段：aim_waypoint 个等距航点，从起点到释放点
        d_lat = release_lat - start_lat
        d_lon = release_lon - start_lon

        for i in range(1, self.aim_waypoint + 1):
            ratio = i / self.aim_waypoint
            wp_lat = start_lat + d_lat * ratio
            wp_lon = start_lon + d_lon * ratio

            wp = Waypoint()
            wp.frame = 3
            wp.command = 16
            wp.x_lat = wp_lat
            wp.y_long = wp_lon
            wp.z_alt = fly_h
            wp.autocontinue = True
            waypoints.append(wp)

        # 投弹舵机：通道7 PWM=1900 释放弹药
        wp_drop = Waypoint()
        wp_drop.frame = 3
        wp_drop.command = 183
        wp_drop.x_lat = release_lat
        wp_drop.y_long = release_lon
        wp_drop.z_alt = fly_h
        wp_drop.param1 = 7.0
        wp_drop.param2 = 1900.0
        wp_drop.autocontinue = True
        waypoints.append(wp_drop)

        # 收回舵机：通道7 PWM=1100
        wp_retract = Waypoint()
        wp_retract.frame = 3
        wp_retract.command = 183
        wp_retract.x_lat = release_lat
        wp_retract.y_long = release_lon
        wp_retract.z_alt = fly_h
        wp_retract.param1 = 7.0
        wp_retract.param2 = 1100.0
        wp_retract.autocontinue = True
        waypoints.append(wp_retract)

        # 加载返航航线（盘旋掉头 + 降落），拼接到打击航线末尾
        return_waypoints = self.load_return_waypoints()
        waypoints.extend(return_waypoints)

        waypoints[0].is_current = True
        return waypoints

    def generate_waypoints(self, target_lat, target_lon):
        # 计算风速
        self.calculate_wind()

        # 设置目标坐标
        self.target_lat = target_lat
        self.target_lon = target_lon

        # 计算释放点
        release_lat, release_lon = self.calculate_release_point()

        # 生成攻击航线（含返航）
        release_pt = (release_lat, release_lon)
        waypoints = self.calculate_approach_waypoints(release_pt)

        return waypoints

    def publish_waypoints(self, waypoints):
        wpl = WaypointList()
        wpl.waypoints = waypoints
        self.secondary_waypoints_pub.publish(wpl)

        self.get_logger().info(
            f'发布打击航线到 /secondary_waypoints，共 {len(waypoints)} 个航点'
        )

    def run(self):
        self.get_logger().info('--- 等待目标坐标 ... ---')
        self._task_done = False
        self._run_timer = self.create_timer(0.5, self._loop_run)

    def _loop_run(self):
        if self._task_done:
            return

        self.load_plan_file(self.target_pos_path)

        if self.target_lat is None or self.target_lon is None:
            return

        target_pt = (self.target_lat, self.target_lon)

        if not self.is_target_stable(target_pt):
            return

        self._task_done = True
        self._run_timer.cancel()

        self.get_logger().info(
            f'目标已稳定: ({self.target_lat:.7f}, {self.target_lon:.7f})，生成航线'
        )

        waypoints = self.generate_waypoints(self.target_lat, self.target_lon)
        self.publish_waypoints(waypoints)

        self.get_logger().info('--- 打击航线已发布，节点进入空闲 ---')

def main(args=None):
    rclpy.init(args=args)
    node = CalculateSecondaryWaypoints01()

    try:        
        node.run()
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')

    finally:
        node.destroy_node()
        rclpy.shutdown()
