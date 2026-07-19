#!/usr/bin/env python3
import rospy
import subprocess
import math
import json
import os
import numpy as np
from pathlib import Path
from collections import deque
from tf.transformations import euler_from_quaternion, rotation_matrix
from mavros_msgs.msg import WaypointList, Waypoint, VFR_HUD 
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped 


class StrikeWaypointsGenerator:
    def __init__(self):
        super().__init__('precision_strike_waypoint_generator')
        
        # 手动输入四个辅助点的GPS坐标（纬度lat，经度lon）
        self.auxiliary_point_1 = (30.5038134, 120.1083164)  # 第一个辅助点
        self.auxiliary_point_2 = (30.5037577, 120.1079188)  # 第二个辅助点
        self.auxiliary_point_3 = (30.5037897, 120.1073009)  # 第三个辅助点
        self.auxiliary_point_4 = (30.5041085, 120.1072686)  # 第四个辅助点
        self.auxiliary_altitude = rospy.get_param('~auxiliary_altitude', 25.0)  # 辅助点飞行高度
        
        # 发布航点话题
        # self.waypoint_pub = rospy.Publisher('/secondary_waypoints', WaypointList, queue_size=1, latch=True)

        # # 传感器数据存储
        # self.current_gps = None  # 当前GPS位置
        # self.current_airspeed = 0.0  # 当前空速(m/s，机体坐标系x轴)
        # self.last_heading = 0.0  # 平滑后的航向角（度）
        # self.yaw = 0.0  # 航向角（弧度，大地坐标系：0=正北，顺时针增加）
        
        # 风速风向数据（ENU坐标系）
        # self.wind_east = 0.0       # 东向风速分量（m/s，正=向东）
        # self.wind_north = 0.0      # 北向风速分量（m/s，正=向北）
        # self.wind_speed = 0.0      # 风速大小（m/s）
        # self.wind_direction = 0.0  # 风向（风的来向，0°=正北，90°=正东）

        # 传感器订阅
        # rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        # rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.airspeed_callback)  # 空速订阅
        # rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_callback)
        # rospy.Subscriber('/mavros/local_position/odom', Odometry, self.orientation_callback)
        
        # self.ground_speed_east = 0.0  # 东向地速（ENU）
        # self.ground_speed_north = 0.0  # 北向地速（ENU）
        # self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0  # 姿态角

        # 核心参数
        # self.json_path = Path(rospy.get_param(
        #     '~json_path', '/home/tian/catkin_ws/src/single_demo/scripts/final_position.json'
        # ))
        # self.backup_json_path = Path('/home/tian/catkin_ws/src/single_demo/scripts/random.json')  # 超时读取
        self.timeout_seconds = 90.0  # 超时时间
        self.start_monitoring_time = None 
        self.use_backup = False
        self.attack_altitude = rospy.get_param('~attack_altitude', 25.0)
        self.approach_distance = 100.0  # 预瞄准段长度
        self.safety_distance = 20.0     # 脱离段长度
        self.approach_waypoint_num = 6  # 预瞄准段航点数量
        self.ekf_ready = False 

        # 检查频率参数
        self.file_ready = False  # 标记文件是否有有效内容
        self.low_freq_interval = 5.0  # 文件未就绪时检查间隔（5秒）
        self.high_freq_interval = 0.5  # 文件就绪检查间隔（0.5秒=2Hz）
        self.last_check_time = 0.0  # 上次检查时间
        
        # 舵机与投弹参数
        self.servo_channel = 6                  # 舵机通道
        self.servo_pwm_attack = 1900            # 投弹时舵机PWM值
        self.NAV_WAYPOINT = 16                  # 航点命令
        self.SET_SERVO = 183                    # 舵机控制命令
        self.NAV_FRAME = 3                      # 航点坐标系（WGS84）
        
        # 弹药参数
        self.ammo_drag_coeff = 1      # 阻力系数
        self.ammo_mass = 0.35            # 弹药质量（kg）
        self.horizontal_decel_factor = 0.9  # 水平减速系数
        self.attitude_safety_factor = 0.98  # 姿态补偿系数
        
        # 目标稳定性校验
        # self.stable_window = 6           # 稳定校验窗口大小
        self.max_pos_error = 0.8         # 位置误差阈值（m）
        # self.target_history = deque(maxlen=self.stable_window)
        # self.wind_history_size = rospy.get_param('~wind_history_size', 10)  # 风速存储
        # self.wind_history = deque(maxlen=self.wind_history_size) 

        # 补偿参数
        self.distance_offset = -15  # 距离补偿（米），初始为0
        self.time_delay = 0.1      # 时间延迟补偿（秒），初始为0

        # 方向参数（核心可调整参数）
        self.base_waypoint_bearing = 260.0  # 基础航线方向
        self.bearing_offset = 0           # 方向偏移量（用于微调，默认0）
        
        # 初始化日志
        rospy.loginfo("="*50)
        rospy.loginfo("手动辅助点配置信息（4个点）：")
        rospy.loginfo(f"辅助点1：纬度 {self.auxiliary_point_1[0]:.7f}, 经度 {self.auxiliary_point_1[1]:.7f}")
        rospy.loginfo(f"辅助点2：纬度 {self.auxiliary_point_2[0]:.7f}, 经度 {self.auxiliary_point_2[1]:.7f}")
        rospy.loginfo(f"辅助点3：纬度 {self.auxiliary_point_3[0]:.7f}, 经度 {self.auxiliary_point_3[1]:.7f}")
        rospy.loginfo(f"辅助点4：纬度 {self.auxiliary_point_4[0]:.7f}, 经度 {self.auxiliary_point_4[1]:.7f}")
        rospy.loginfo(f"辅助点飞行高度：{self.auxiliary_altitude:.1f}m")
        rospy.loginfo("="*50)
        rospy.loginfo(f"投弹高度: {self.attack_altitude}m | 预瞄准段长度: {self.approach_distance}m")
        rospy.loginfo(f"基础航线方向: {self.base_waypoint_bearing}° | 方向偏移量: {self.bearing_offset}°")
        rospy.loginfo(f"文件检查策略：未就绪时每{self.low_freq_interval}秒一次，就绪后每{self.high_freq_interval}秒一次")
        rospy.loginfo("等待传感器数据（GPS、空速、EKF里程计）...")

    def airspeed_callback(self, msg):
        raw_airspeed = msg.airspeed
        min_valid_airspeed = 0.0  
        max_valid_airspeed = 50.0  
        if raw_airspeed < min_valid_airspeed or raw_airspeed > max_valid_airspeed:
            rospy.logwarn(f"空速异常: {raw_airspeed:.2f}m/s，使用上一时刻有效值")
            if not hasattr(self, 'current_airspeed'):
                self.current_airspeed = 10.0
            return
    
        self.current_airspeed = raw_airspeed
        rospy.loginfo_throttle(2, f"当前空速: {self.current_airspeed:.2f}m/s")

    def orientation_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        rospy.loginfo_throttle(1, 
            f"姿态角 - 滚转: {math.degrees(self.roll):.1f}° | 俯仰: {math.degrees(self.pitch):.1f}° | 航向: {math.degrees(self.yaw):.1f}°"
        )
        if not self.ekf_ready:
            self.ekf_ready = True
            rospy.loginfo("EKF数据就绪")

    def velocity_callback(self, msg):
        self.ground_speed_east = msg.twist.linear.x
        self.ground_speed_north = msg.twist.linear.y
        ground_speed = math.hypot(self.ground_speed_east, self.ground_speed_north)
        ground_speed_dir = math.degrees(math.atan2(self.ground_speed_east, self.ground_speed_north)) % 360
        rospy.loginfo_throttle(1, 
            f"地速 - 东向: {self.ground_speed_east:.2f} | 北向: {self.ground_speed_north:.2f} | 大小: {ground_speed:.2f}m/s | 方向: {ground_speed_dir:.1f}°"
        )

    def calculate_airspeed_enu(self):
        airspeed = self.current_airspeed
        if airspeed < 0.1:
            return 0.0, 0.0
        
        ground_speed_dir = math.degrees(math.atan2(self.ground_speed_east, self.ground_speed_north)) % 360
        airspeed_bearing = ground_speed_dir
        airspeed_rad = math.radians(airspeed_bearing)
        airspeed_east = airspeed * math.sin(airspeed_rad)
        airspeed_north = airspeed * math.cos(airspeed_rad)
        
        rospy.loginfo(
            f"空速校准：地速方向{ground_speed_dir:.1f}° → 空速方向{airspeed_bearing:.1f}° → "
            f"东向{airspeed_east:.2f}, 北向{airspeed_north:.2f}"
        )
        return airspeed_east, airspeed_north    

    def calculate_wind(self):
        if self.current_airspeed < 0.1:
            rospy.logwarn_throttle(2, "空速过低（<0.1m/s），无法准确计算风速")
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
            rospy.logwarn(f"极端异常风速({current_wind_speed:.2f}m/s)，使用上一次有效值")
            if self.wind_history:
                current_wind_east, current_wind_north, current_wind_speed = self.wind_history[-1]
            else:
                current_wind_east = 0.0
                current_wind_north = 0.0
                current_wind_speed = 0.0

        self.wind_history.append((current_wind_east, current_wind_north, current_wind_speed))
        avg_east = sum(w[0] for w in self.wind_history) / len(self.wind_history)
        avg_north = sum(w[1] for w in self.wind_history) / len(self.wind_history)
        avg_speed = math.hypot(avg_east, avg_north)

        self.wind_east = avg_east
        self.wind_north = avg_north
        self.wind_speed = avg_speed

        log_msg = (
            f"瞬时风速: 东向 {current_wind_east:.4f}, 北向 {current_wind_north:.4f} → 大小 {current_wind_speed:.4f}m/s\n"
            f"{len(self.wind_history)}/{self.wind_history_size}次平均风速: 东向 {self.wind_east:.4f}, 北向 {self.wind_north:.4f} → 大小 {self.wind_speed:.4f}m/s"
        )
        rospy.loginfo(log_msg) 

    def gps_callback(self, msg):
        self.current_gps = msg
        if not hasattr(self, 'gps_initialized'):
            self.gps_initialized = True
            rospy.loginfo(f"GPS数据就绪 | 初始位置: ({msg.latitude:.7f}, {msg.longitude:.7f})")
            # 计算当前位置到第一个辅助点的距离，提示飞行方向
            dist_to_aux1 = self.calculate_gps_distance(
                msg.latitude, msg.longitude,
                self.auxiliary_point_1[0], self.auxiliary_point_1[1]
            )
            bearing_to_aux1 = self.calculate_bearing(
                msg.latitude, msg.longitude,
                self.auxiliary_point_1[0], self.auxiliary_point_1[1]
            )
            rospy.loginfo(f"当前位置到辅助点1的距离：{dist_to_aux1:.2f}m，方向：{bearing_to_aux1:.1f}°")

    def parse_target_position(self, use_backup=False):
        file_path = self.backup_json_path if use_backup else self.json_path

        if not file_path.exists():
            rospy.logdebug("目标位置文件不存在（%s）", file_path)
            return None
        if file_path.stat().st_size == 0:
            rospy.loginfo("目标位置文件为空（%s），等待数据写入...", file_path)
            return None

        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            if 'lat' not in data or 'lon' not in data:
                rospy.logwarn("目标位置文件缺少'lat'或'lon'字段")
                return None

            if not use_backup and not self.file_ready:
                self.file_ready = True
                rospy.loginfo(f"文件状态变更：已就绪，检查频率切换为{self.high_freq_interval}秒/次")
            return (data['lat'], data['lon'])

        except json.JSONDecodeError as e:
            rospy.logwarn("目标位置文件格式错误：%s", str(e))
        except Exception as e:
            rospy.logwarn("读取目标位置文件失败：%s", str(e))

        if not use_backup and self.file_ready:
            self.file_ready = False
            rospy.loginfo(f"文件状态变更：未就绪，检查频率切换为{self.low_freq_interval}秒/次")
        return None

    def is_target_stable(self, target):
        if not target:
            self.target_history.clear()
            return False
        self.target_history.append(target)
        if len(self.target_history) < self.stable_window:
            return False
        ref_lat, ref_lon = self.target_history[0]
        max_dist = 0.0
        for (lat, lon) in list(self.target_history)[1:]:
            dist = self.calculate_gps_distance(ref_lat, ref_lon, lat, lon)
            max_dist = max(max_dist, dist)
        return max_dist <= self.max_pos_error

    def calculate_gps_distance(self, lat1, lon1, lat2, lon2):
        R = 6378137.0  # 地球半径（米）
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad)*math.cos(lat2_rad)*math.sin(dlon/2)** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

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
        return (math.degrees(new_lat_rad), math.degrees(new_lon_rad))

    def get_actual_bearing(self, base_bearing):
        """计算实际航线方向（基础方向+偏移量）"""
        actual = (base_bearing + self.bearing_offset) % 360
        rospy.loginfo(f"基础方向：{base_bearing:.1f}° + 偏移量：{self.bearing_offset:.1f}° → 实际方向：{actual:.1f}°")
        return actual

    def calculate_release_point(self, target):
        """计算投弹释放点（沿航线反方向从打击点偏移）"""
        # 弹药下落时间计算
        free_fall_time = math.sqrt(2 * self.attack_altitude / 9.8)
        vertical_drag_factor = 0.2
        fall_time = free_fall_time * (1 + vertical_drag_factor * self.ammo_drag_coeff) + self.time_delay
        
        # 获取实际航线方向
        actual_waypoint_bearing = self.get_actual_bearing(self.base_waypoint_bearing)
        # 投弹点应沿航线反方向偏移（无人机飞行方向的前方）
        release_bearing = (actual_waypoint_bearing + 180) % 360  # 关键修正：反向
        
        # 计算目标方位角和航向平滑
        target_bearing = self.calculate_bearing(
            self.current_gps.latitude, self.current_gps.longitude,
            target[0], target[1]
        )
        if not hasattr(self, 'last_heading'):
            self.last_heading = target_bearing
        heading_diff = (target_bearing - self.last_heading) % 360
        if heading_diff > 180:
            heading_diff -= 360
        self.last_heading = (self.last_heading + 0.2 * heading_diff) % 360
        
        # 风速分解（基于实际航线方向）
        heading_rad = math.radians(self.last_heading)
        wind_longitudinal = self.wind_north * math.cos(heading_rad) + self.wind_east * math.sin(heading_rad)
        wind_lateral = -self.wind_north * math.sin(heading_rad) + self.wind_east * math.cos(heading_rad)
        
        # 提前量计算
        plane_advance = self.current_airspeed * fall_time
        wind_advance = wind_longitudinal * fall_time
        total_advance = (plane_advance + wind_advance) * self.horizontal_decel_factor * self.attitude_safety_factor + self.distance_offset

        # 投弹点：从打击点沿修正后的方向（release_bearing）偏移提前量
        release_point = self.calculate_offset_point(
            target[0], target[1], 
            release_bearing,  # 改用反向后的方向
            total_advance
        )
        
        # 侧向风补偿
        wind_lateral_offset = wind_lateral * fall_time
        if abs(wind_lateral_offset) > 0.3:
            lateral_bearing = (self.last_heading + 90) % 360 if wind_lateral_offset > 0 else (self.last_heading - 90) % 360
            release_point = self.calculate_offset_point(
                release_point[0], release_point[1], 
                lateral_bearing, 
                abs(wind_lateral_offset)
            )
        
        # 验证投弹点与打击点的方位关系
        release_to_target_bearing = self.calculate_bearing(
            release_point[0], release_point[1],
            target[0], target[1]
        )
        rospy.loginfo(f"投弹点→打击点方位角：{release_to_target_bearing:.1f}°（应接近 {actual_waypoint_bearing:.1f}°，即航线方向）")
        rospy.loginfo(f"下落时间: {fall_time:.2f}s | 总提前量: {total_advance:.1f}m")
        return release_point

    def calculate_bearing(self, start_lat, start_lon, target_lat, target_lon):
        lat1, lon1 = math.radians(start_lat), math.radians(start_lon)
        lat2, lon2 = math.radians(target_lat), math.radians(target_lon)
        dlon = lon2 - lon1
        
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad) % 360
        return bearing_deg

    def generate_approach_waypoints(self, target):
        """生成预瞄准段和打击段的航点"""
        if not self.current_gps:
            rospy.logerr("无法生成航点：GPS数据缺失")
            return [], None, None
        
        # 计算关键点位
        release_point = self.calculate_release_point(target)
        actual_waypoint_bearing = self.get_actual_bearing(self.base_waypoint_bearing)
        
        # 预瞄准段起点：沿航线反方向偏移
        approach_bearing = (actual_waypoint_bearing + 180) % 360  # 航线反方向
        approach_start = self.calculate_offset_point(
            release_point[0], release_point[1], 
            approach_bearing,
            self.approach_distance
        )
        
        # 验证预瞄准起点与投弹点的方向
        start_to_release_bearing = self.calculate_bearing(
            approach_start[0], approach_start[1],
            release_point[0], release_point[1]
        )
        rospy.loginfo(f"预瞄准起点→投弹点方位角：{start_to_release_bearing:.1f}°（应与航线方向一致）")
        
        # 生成预瞄准段航点
        waypoints = []
        for i in range(self.approach_waypoint_num):
            ratio = i / (self.approach_waypoint_num - 1)
            distance_from_start = ratio * self.approach_distance
            lat, lon = self.calculate_offset_point(
                approach_start[0], approach_start[1],
                actual_waypoint_bearing,  # 沿实际航线方向
                distance_from_start
            )
            
            wp = Waypoint()
            wp.command = self.NAV_WAYPOINT
            wp.frame = self.NAV_FRAME
            wp.x_lat, wp.y_long, wp.z_alt = lat, lon, self.attack_altitude
            wp.autocontinue = True
            wp.is_current = False  # 初始不设为当前航点，由辅助点引导
            waypoints.append(wp)
            rospy.logdebug(f"预瞄准航点{i+1}：({lat:.7f}, {lon:.7f})")
        
        # 投弹点（舵机控制）
        servo_wp = Waypoint()
        servo_wp.command = self.SET_SERVO
        servo_wp.param1 = self.servo_channel
        servo_wp.param2 = self.servo_pwm_attack
        servo_wp.x_lat, servo_wp.y_long, servo_wp.z_alt = release_point[0], release_point[1], self.attack_altitude
        servo_wp.autocontinue = True
        waypoints.append(servo_wp)
        rospy.loginfo(f"投弹点：({release_point[0]:.7f}, {release_point[1]:.7f})")
        
        # 脱离点
        exit_point = self.calculate_offset_point(
            release_point[0], release_point[1], 
            actual_waypoint_bearing, 
            self.safety_distance
        )
        exit_wp = Waypoint()
        exit_wp.command = self.NAV_WAYPOINT
        exit_wp.x_lat, exit_wp.y_long, exit_wp.z_alt = exit_point[0], exit_point[1], self.attack_altitude
        exit_wp.autocontinue = True
        waypoints.append(exit_wp)
        rospy.loginfo(f"脱离点：({exit_point[0]:.7f}, {exit_point[1]:.7f})")
        
        # 打印关键点位距离
        rospy.loginfo(f"投弹点与打击点距离：{self.calculate_gps_distance(release_point[0], release_point[1], target[0], target[1]):.2f}m")
        return waypoints, approach_start, actual_waypoint_bearing

    def generate_auxiliary_waypoints(self):
        """生成四个手动配置的辅助点航点"""
        auxiliary_waypoints = []
        
        # 创建辅助点1航点（设为当前起始航点）
        wp1 = Waypoint()
        wp1.command = self.NAV_WAYPOINT
        wp1.frame = self.NAV_FRAME
        wp1.x_lat, wp1.y_long, wp1.z_alt = self.auxiliary_point_1[0], self.auxiliary_point_1[1], self.auxiliary_altitude
        wp1.autocontinue = True
        wp1.is_current = True  # 第一个辅助点设为当前航点
        auxiliary_waypoints.append(wp1)
        rospy.logdebug(f"辅助点1航点已创建，是否当前：{wp1.is_current}")
        
        # 创建辅助点2航点
        wp2 = Waypoint()
        wp2.command = self.NAV_WAYPOINT
        wp2.frame = self.NAV_FRAME
        wp2.x_lat, wp2.y_long, wp2.z_alt = self.auxiliary_point_2[0], self.auxiliary_point_2[1], self.auxiliary_altitude
        wp2.autocontinue = True
        wp2.is_current = False
        auxiliary_waypoints.append(wp2)
        rospy.logdebug(f"辅助点2航点已创建")
        
        # 创建辅助点3航点
        wp3 = Waypoint()
        wp3.command = self.NAV_WAYPOINT
        wp3.frame = self.NAV_FRAME
        wp3.x_lat, wp3.y_long, wp3.z_alt = self.auxiliary_point_3[0], self.auxiliary_point_3[1], self.auxiliary_altitude
        wp3.autocontinue = True
        wp3.is_current = False
        auxiliary_waypoints.append(wp3)
        rospy.logdebug(f"辅助点3航点已创建")
        
        # 创建辅助点4航点
        wp4 = Waypoint()
        wp4.command = self.NAV_WAYPOINT
        wp4.frame = self.NAV_FRAME
        wp4.x_lat, wp4.y_long, wp4.z_alt = self.auxiliary_point_4[0], self.auxiliary_point_4[1], self.auxiliary_altitude
        wp4.autocontinue = True
        wp4.is_current = False
        auxiliary_waypoints.append(wp4)
        rospy.logdebug(f"辅助点4航点已创建")
        
        # 计算辅助点之间的距离和方向，用于验证
        dist_1_to_2 = self.calculate_gps_distance(
            self.auxiliary_point_1[0], self.auxiliary_point_1[1],
            self.auxiliary_point_2[0], self.auxiliary_point_2[1]
        )
        dist_2_to_3 = self.calculate_gps_distance(
            self.auxiliary_point_2[0], self.auxiliary_point_2[1],
            self.auxiliary_point_3[0], self.auxiliary_point_3[1]
        )
        dist_3_to_4 = self.calculate_gps_distance(
            self.auxiliary_point_3[0], self.auxiliary_point_3[1],
            self.auxiliary_point_4[0], self.auxiliary_point_4[1]
        )
        rospy.loginfo(f"辅助点1到2的距离：{dist_1_to_2:.2f}m | 辅助点2到3：{dist_2_to_3:.2f}m | 辅助点3到4：{dist_3_to_4:.2f}m")
        
        return auxiliary_waypoints

    def generate_waypoints(self, target):
        """生成完整的航点序列：4个辅助点 + 预瞄准段 + 投弹点 + 脱离点"""
        # 先生成打击航线的航点
        strike_waypoints, approach_start, actual_bearing = self.generate_approach_waypoints(target)
        if not strike_waypoints:
            return []
            
        # 生成辅助点航点
        auxiliary_waypoints = self.generate_auxiliary_waypoints()
        
        # 组合所有航点：辅助点 -> 预瞄准段 -> 投弹点 -> 脱离点
        all_waypoints = auxiliary_waypoints + strike_waypoints
        
        # 确保只有第一个航点为当前航点
        for i, wp in enumerate(all_waypoints):
            wp.is_current = (i == 0)
            rospy.logdebug(f"最终航点 {i+1}：({wp.x_lat:.7f}, {wp.y_long:.7f})，是否当前：{wp.is_current}")
        
        return all_waypoints
        
    def publish_waypoints(self, target):
        waypoints = self.generate_waypoints(target)
        if not waypoints:
            rospy.logerr("无法发布航点：航点列表为空")
            return
        
        waypoint_list = WaypointList()
        waypoint_list.waypoints = waypoints
        self.waypoint_pub.publish(waypoint_list)
        rospy.loginfo(f"成功发布 {len(waypoints)} 个航点到 /secondary_waypoints 话题")
        rospy.loginfo(f"航点构成：4个辅助点 + {self.approach_waypoint_num}个预瞄准点 + 1个投弹点 + 1个脱离点")
        
        self.waypoints_published = True

    def run(self):
        rate = rospy.Rate(10)  # 10Hz基础循环
        while not rospy.is_shutdown():
            if not self.current_gps:
                rospy.loginfo("等待GPS信号...")
                rate.sleep()
                continue
            if not self.ekf_ready:
                rospy.loginfo("等待EKF数据...")
                rate.sleep()
                continue

            if self.start_monitoring_time is None:
                self.start_monitoring_time = rospy.get_time()
                rospy.loginfo(f"开始监控目标文件，超时时间{self.timeout_seconds}秒")
            self.calculate_wind()
            
            current_time = rospy.get_time()
            check_interval = self.high_freq_interval if self.file_ready else self.low_freq_interval
            
            if current_time - self.last_check_time >= check_interval:
                self.last_check_time = current_time

            if not self.use_backup and (current_time - self.start_monitoring_time) >= self.timeout_seconds:
                self.use_backup = True
                rospy.logwarn(f"已超时{self.timeout_seconds}秒，切换至备用文件：{self.backup_json_path}")

            target = self.parse_target_position(use_backup=self.use_backup)

            if target:
                if len(self.wind_history) < self.wind_history_size:
                    rospy.loginfo(f"等待风速缓存填满（当前{len(self.wind_history)}/{self.wind_history_size}条）...")
                    continue
                if self.is_target_stable(target):
                    rospy.loginfo("目标已稳定，开始生成航点")
                    self.publish_waypoints(target)
                    return
                else:
                    rospy.loginfo(f"目标未稳定（需连续{self.stable_window}次位置稳定）...")
            else:
                rospy.loginfo(f"文件未就绪，当前检查频率：每{check_interval}秒一次")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        generator = StrikeWaypointsGenerator()
        generator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断，退出程序")
    except Exception as e:
        rospy.logerr(f"程序异常终止：{str(e)}")
        import traceback
        traceback.print_exc()
    