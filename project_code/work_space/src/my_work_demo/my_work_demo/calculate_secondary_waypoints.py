#!/usr/bin/env python3
import rclpy
import math
import json
from pathlib import Path
from collections import deque
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import Waypoint, WaypointList, VFR_HUD
from mavros_msgs.srv import WaypointPush, WaypointClear, CommandLong


# ======================================================================
# 物理常量和环境参数
# ======================================================================

# 重力加速度 (m/s^2)
G = 9.81
# 地球半径 (米), WGS84
EARTH_RADIUS = 6378137.0
# 海平面标准空气密度 (kg/m^3)
RHO = 1.225
# 度转弧度系数
DEG_TO_RAD = math.pi / 180.0
# 纬度1度对应的米数 (约111320m/deg)
ONE_DEG_LAT_M = EARTH_RADIUS * DEG_TO_RAD

# ======================================================================
# 弹药物理参数 (500g水瓶)
# ======================================================================

# 弹药质量 (kg)
BOTTLE_MASS = 0.5
# 阻力系数 (圆柱体/瓶状)
BOTTLE_CD = 0.82
# 迎风面积 (m^2), 约Φ6cm瓶的截面积
BOTTLE_AREA = 0.014

# ======================================================================
# 任务参数
# ======================================================================

# 全程飞行高度 (米, 相对起飞点)
FLIGHT_ALTITUDE = 35.0

# 安全区: 60x60米方形, 天井在其内20x20米方形四角
# 安全区与天井几何中心重合
SAFE_ZONE_SIDE = 60.0              # 安全区边长 (米)
SAFE_ZONE_HALF_SIDE = SAFE_ZONE_SIDE / 2.0    # 半边长 = 30m
# 安全区角点距中心距离 = 半边长 * sqrt(2) ≈ 42.43m
SAFE_ZONE_CORNER_DIST = SAFE_ZONE_HALF_SIDE * math.sqrt(2.0)
SAFE_MARGIN = 3.0                  # 安全区内缩余量 (防止飞出边界)

# 天井: 20x20米方形, 位于安全区中心
TIANJING_HALF = 10.0               # 天井半边长 (米)

# 目标获取超时: 90秒后视觉未给出有效值则用备用文件
TARGET_TIMEOUT = 90.0

# ======================================================================
# 风速/目标滤波参数
# ======================================================================

# 风速滑动窗口大小
WIND_HISTORY_SIZE = 10
# 目标稳定窗口: 连续6个点与第一个误差≤1m则认为稳定
STABLE_WINDOW = 6
MAX_POS_ERROR = 1.0                # 目标点间最大允许误差 (米)

# ======================================================================
# MAVLink 命令常量
# ======================================================================

# 航点命令: 悬停/直线飞行至指定坐标
MAV_CMD_NAV_WAYPOINT = 16
# 舵机控制命令: 通道7 → 投弹舵机
MAV_CMD_DO_SET_SERVO = 183
# 坐标参考系: 全球经纬度 + 相对于起飞点的高度
MAV_FRAME_GLOBAL_REL_ALT = 3


class CalculateSecondaryWaypoints(Node):
    """
    第二航线节点: 打击航线生成与发布

    职责:
      1. 从JSON文件读取天井坐标, 反推60x60m安全区边界
      2. 从JSON文件获取打击目标 (高价值目标), 6点滑动窗口滤波
      3. 根据空速/地速估算风速, 建立弹药下落物理模型
      4. 计算投弹提前量, 生成接近→投弹→脱离的打击航线
      5. 以/secondary_waypoints话题发布航点, 供auto_mission模式执行

    状态机: WAIT_EKF → READ_SAFE_ZONE → READ_TARGET → COMPUTE → DONE
    """

    def __init__(self):
        super().__init__('calculate_secondary_waypoints')

        # --- QoS: BEST_EFFORT 容忍丢包, KEEP_LAST 只用最新数据 ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- 发布器: 将生成的打击航点发布到 /secondary_waypoints ---
        self.waypoint_pub = self.create_publisher(
            WaypointList, '/secondary_waypoints', qos
        )

        # --- 传感器订阅 ---
        # GPS全局位置 (经纬度+海拔)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.global_position_callback, qos
        )
        # 空速 (皮托管测得的相对气流速度)
        self.vfr_hud_sub = self.create_subscription(
            VFR_HUD, '/mavros/vfr_hud',
            self.airspeed_callback, qos
        )
        # 地速 (ENU坐标系下相对于地面的速度)
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self.velocity_callback, qos
        )
        # 里程计 (位姿+姿态四元数 → 欧拉角)
        self.odom_sub = self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self.odometry_callback, qos
        )

        # --- MAVROS 服务客户端 (预留, 可用于手动推送/清除航点) ---
        self.wp_push_cli = self.create_client(WaypointPush, '/mavros/mission/push')
        self.wp_clear_cli = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.cmd_long_cli = self.create_client(CommandLong, '/mavros/cmd/command')

        # --- ROS2 参数声明 (可在launch文件中覆写) ---
        self.declare_parameter('stable_window', STABLE_WINDOW)
        self.declare_parameter('max_pos_error', MAX_POS_ERROR)
        self.declare_parameter('wind_history_size', WIND_HISTORY_SIZE)
        # 打击目标JSON路径 (主文件, 由视觉程序写入)
        self.declare_parameter('json_path',
            '/home/qing/project_code/work_space/src/target_pos/target_pos.json')
        # 打击目标备用JSON路径 (90s超时后使用)
        self.declare_parameter('backup_json_path',
            '/home/qing/project_code/work_space/src/target_pos/target_pos_backup.json')
        # 天井角点JSON路径 (主文件)
        self.declare_parameter('safe_zone_json',
            '/home/qing/project_code/work_space/src/tianjing/tianjing_pos.json')
        # 天井角点备用JSON路径
        self.declare_parameter('safe_zone_backup_json',
            '/home/qing/project_code/work_space/src/tianjing/tianjing_pos_backup.json')

        self.stable_window = self.get_parameter('stable_window').value
        self.max_pos_error = self.get_parameter('max_pos_error').value
        self.wind_history_size = self.get_parameter('wind_history_size').value
        self.target_json = Path(self.get_parameter('json_path').value)
        self.backup_target_json = Path(self.get_parameter('backup_json_path').value)
        self.safe_zone_json = Path(self.get_parameter('safe_zone_json').value)
        self.safe_zone_backup_json = Path(self.get_parameter('safe_zone_backup_json').value)

        # --- 传感器数据缓存 ---
        # 当前GPS位置: (lat, lon, alt)
        self.global_pos = None
        # 当前空速 (m/s), 经合法性过滤
        self.airspeed = 0.0
        # 地速东向/北向分量 (m/s), 来自 /mavros/local_position/velocity_local
        self.ground_vel_e = 0.0
        self.ground_vel_n = 0.0
        # 姿态角 (弧度): roll, pitch, yaw
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # EKF标志: Odom首次回调后置True
        self.ekf_ok = False

        # --- 风速估算 ---
        # 滑动平均后的风速 (东向, 北向, 合速度) (m/s)
        self.wind_e = 0.0
        self.wind_n = 0.0
        self.wind_spd = 0.0
        # 瞬时风速采样队列 (maxlen=WIND_HISTORY_SIZE)
        self.wind_history = deque(maxlen=self.wind_history_size)

        # --- 目标锁定 ---
        # 目标坐标采样队列 (maxlen=STABLE_WINDOW=6)
        self.target_history = deque(maxlen=self.stable_window)
        # 滤波后的目标位置: (lat, lon)
        self.filtered_target = None

        # --- 安全区 ---
        # 安全区四个角点: [(lat,lon), ...]
        self.zone_corners = None
        # 安全区几何中心: (lat, lon)
        self.zone_center = None
        # 安全区就绪标志
        self.zone_ready = False

        # --- 状态机 ---
        # WAIT_EKF → READ_SAFE_ZONE → READ_TARGET → COMPUTE → DONE
        self.state = 'WAIT_EKF'
        # 节点启动时刻, 用于90s超时判定
        self.node_start_time = self.get_clock().now()
        # 防止重复发布
        self.mission_published = False

        # 主循环定时器: 5Hz (0.2s间隔)
        self.timer = self.create_timer(0.2, self.main_loop)

        self.get_logger().info('=== 第二航线节点已启动 ===')
        self.get_logger().info(f'飞行高度: {FLIGHT_ALTITUDE}m')
        self.get_logger().info(f'目标超时: {TARGET_TIMEOUT}s')
        self.get_logger().info(f'稳定窗口: {self.stable_window}次, 误差阈值: {self.max_pos_error}m')

    # ============================================================
    # 传感器回调 — 仅做数据缓存, 不做计算
    # ============================================================

    def global_position_callback(self, msg: NavSatFix):
        self.global_pos = (msg.latitude, msg.longitude, msg.altitude)

    def airspeed_callback(self, msg: VFR_HUD):
        raw = msg.airspeed
        if 0.0 < raw < 50.0:
            self.airspeed = raw

    def velocity_callback(self, msg: TwistStamped):
        self.ground_vel_e = msg.twist.linear.x
        self.ground_vel_n = msg.twist.linear.y

    def odometry_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        if not self.ekf_ok:
            self.ekf_ok = True
            self.get_logger().info('EKF 就绪')

    # ============================================================
    # 主循环 — 状态机驱动 (5Hz)
    #
    # WAIT_EKF       等待EKF/GPS数据就绪 → 立即转下一状态
    # READ_SAFE_ZONE 读取天井坐标, 计算60m安全区边界
    # READ_TARGET    读取打击目标, 6点滤波稳定后锁定
    #                超过90s未锁定则自动切换备用文件
    # COMPUTE        物理仿真 + 生成航线 + 发布
    # DONE           任务完成, 空闲
    # ============================================================

    def main_loop(self):
        if self.global_pos is None or not self.ekf_ok:
            return

        self._update_wind()

        if self.state == 'WAIT_EKF':
            self.state = 'READ_SAFE_ZONE'

        elif self.state == 'READ_SAFE_ZONE':
            self._compute_safe_zone()
            if self.zone_ready:
                self.state = 'READ_TARGET'
                self.get_logger().info(
                    f'安全区就绪，中心: ({self.zone_center[0]:.7f}, {self.zone_center[1]:.7f})'
                )

        elif self.state == 'READ_TARGET':
            elapsed = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9
            use_backup = elapsed > TARGET_TIMEOUT
            self._read_and_filter_target(use_backup)
            if self.filtered_target is not None:
                self.state = 'COMPUTE'
                self.get_logger().info(
                    f'目标锁定: ({self.filtered_target[0]:.7f}, {self.filtered_target[1]:.7f})'
                )

        elif self.state == 'COMPUTE':
            waypoints = self._generate_waypoints()
            if waypoints is not None:
                self._publish(waypoints)
                self.state = 'DONE'
                self.mission_published = True

        elif self.state == 'DONE':
            pass

    # ============================================================
    # 风速估算
    #
    # 原理: 地速 = 空速 + 风速 (矢量)
    #   - ground_vel 来自 /mavros/local_position/velocity_local (ENU)
    #   - airspeed 来自 /mavros/vfr_hud, 方向 = 机头朝向 (yaw)
    #   - 风速 = 地速 - 空速投影
    # 异常值检测: >50m/s 丢弃, 使用滑动平均平滑
    # ============================================================

    def _update_wind(self):
        if self.airspeed < 0.5:
            return

        air_east = self.airspeed * math.sin(self.yaw)
        air_north = self.airspeed * math.cos(self.yaw)

        inst_wind_e = self.ground_vel_e - air_east
        inst_wind_n = self.ground_vel_n - air_north
        inst_wind_spd = math.hypot(inst_wind_e, inst_wind_n)

        if inst_wind_spd > 50.0:
            self.get_logger().warn('风速异常，丢弃本次采样', throttle_duration_sec=5.0)
            return

        self.wind_history.append((inst_wind_e, inst_wind_n, inst_wind_spd))
        if len(self.wind_history) == 0:
            return
        self.wind_e = sum(w[0] for w in self.wind_history) / len(self.wind_history)
        self.wind_n = sum(w[1] for w in self.wind_history) / len(self.wind_history)
        self.wind_spd = math.hypot(self.wind_e, self.wind_n)

    # ============================================================
    # 安全区计算
    #
    # 1. 从JSON读取4个天井角点GPS坐标
    # 2. 去重 (距离<0.5m视为同一点)
    # 3. 缺角补全 (支持2/3个角点推完整矩形)
    # 4. 以天井几何中心 → 安全区几何中心, 外扩至60m方形的四角
    # 5. 主文件无效时自动切换备用文件
    # ============================================================

    def _compute_safe_zone(self):
        corners = self._read_corners(self.safe_zone_json)
        if corners is None:
            self.get_logger().warn('主安全区文件无效，尝试备份')
            corners = self._read_corners(self.safe_zone_backup_json)
        if corners is None:
            return

        corners = self._dedup_corners(corners, tolerance=0.5)
        if len(corners) < 2:
            self.get_logger().error(f'去重后有效角点不足 ({len(corners)}), 无法确定安全区')
            return

        if len(corners) < 4:
            corners = self._complete_rect(corners)
            if corners is None:
                return

        lat_c = sum(p[0] for p in corners) / 4.0
        lon_c = sum(p[1] for p in corners) / 4.0
        self.zone_center = (lat_c, lon_c)

        safe_pts = []
        for lat, lon in corners:
            bearing = self._bearing(lat_c, lon_c, lat, lon)
            pt = self._offset(lat_c, lon_c, bearing, SAFE_ZONE_CORNER_DIST)
            safe_pts.append(pt)
        self.zone_corners = safe_pts
        self.zone_ready = True

        self.get_logger().info(f'安全区中心 ({lat_c:.7f}, {lon_c:.7f})')
        for i, pt in enumerate(safe_pts):
            self.get_logger().info(f'  角{i+1}: ({pt[0]:.7f}, {pt[1]:.7f})')

    def _read_corners(self, path):
        """
        读取天井角点JSON文件.
        支持格式:
          [{"lat": xx, "lon": xx}, ...]        (list of dicts)
          [[lat, lon], ...]                     (list of lists)
          {"corners": [...]}                    (dict with 'corners' key)
          {"corner_1": ..., "corner_2": ...}    (dict with numbered keys)
        """
        if not path.exists():
            return None
        try:
            with open(path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().warn(f'读取角点文件异常: {e}')
            return None

        corners = []
        if isinstance(data, list):
            for item in data:
                if isinstance(item, (list, tuple)) and len(item) >= 2:
                    corners.append((float(item[0]), float(item[1])))
                elif isinstance(item, dict) and 'lat' in item and 'lon' in item:
                    corners.append((float(item['lat']), float(item['lon'])))
        elif isinstance(data, dict):
            if 'corners' in data:
                for c in data['corners']:
                    if isinstance(c, (list, tuple)) and len(c) >= 2:
                        corners.append((float(c[0]), float(c[1])))
                    elif isinstance(c, dict) and 'lat' in c and 'lon' in c:
                        corners.append((float(c['lat']), float(c['lon'])))
            else:
                for key in ['corner_1', 'corner_2', 'corner_3', 'corner_4',
                            'c1', 'c2', 'c3', 'c4', '1', '2', '3', '4']:
                    if key in data:
                        c = data[key]
                        if isinstance(c, (list, tuple)) and len(c) >= 2:
                            corners.append((float(c[0]), float(c[1])))
                        elif isinstance(c, dict) and 'lat' in c and 'lon' in c:
                            corners.append((float(c['lat']), float(c['lon'])))

        if len(corners) < 2:
            return None
        return corners

    def _dedup_corners(self, corners, tolerance=0.5):
        unique = []
        for p in corners:
            dup = False
            for q in unique:
                if self._gps_dist(p[0], p[1], q[0], q[1]) < tolerance:
                    dup = True
                    break
            if not dup:
                unique.append(p)
        if len(unique) < len(corners):
            self.get_logger().info(f'天井去重: {len(corners)} -> {len(unique)}')
        return unique

    def _complete_rect(self, points):
        """
        根据已有角点补全20x20m天井矩形的4个角点.
        - 3点: 通过最远距离识别对角顶点, 用平行四边形法则求第4点
        - 2点: 判定为对角或邻角 (距离>24m为对角), 推算其余2点
        """
        if len(points) >= 4:
            return points[:4]

        if len(points) == 3:
            p1, p2, p3 = points[0], points[1], points[2]
            d12 = self._gps_dist(p1[0], p1[1], p2[0], p2[1])
            d23 = self._gps_dist(p2[0], p2[1], p3[0], p3[1])
            d13 = self._gps_dist(p1[0], p1[1], p3[0], p3[1])

            if d13 >= d12 and d13 >= d23:
                p4 = (p1[0] + p3[0] - p2[0], p1[1] + p3[1] - p2[1])
            elif d12 >= d23 and d12 >= d13:
                p4 = (p1[0] + p2[0] - p3[0], p1[1] + p2[1] - p3[1])
            else:
                p4 = (p2[0] + p3[0] - p1[0], p2[1] + p3[1] - p1[1])

            self.get_logger().info(f'补全第四角点: ({p4[0]:.7f}, {p4[1]:.7f})')
            return [p1, p2, p3, p4]

        if len(points) == 2:
            p1, p2 = points
            d = self._gps_dist(p1[0], p1[1], p2[0], p2[1])
            side_est = TIANJING_HALF * 2.0
            diag_est = side_est * math.sqrt(2.0)

            if d > (side_est + diag_est) / 2.0:
                self.get_logger().info('2点推定为对角点，计算另两个角点')
                center = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
                b_p1 = self._bearing(center[0], center[1], p1[0], p1[1])
                half_diag = d / 2.0
                p3 = self._offset(center[0], center[1], (b_p1 + 90.0) % 360.0, half_diag)
                p4 = self._offset(center[0], center[1], (b_p1 + 270.0) % 360.0, half_diag)
            else:
                self.get_logger().info('2点推定为相邻角点，计算另两个角点')
                b12 = self._bearing(p1[0], p1[1], p2[0], p2[1])
                perp = (b12 - 90.0) % 360.0
                p3 = self._offset(p1[0], p1[1], perp, d)
                p4 = self._offset(p2[0], p2[1], perp, d)

            self.get_logger().info(
                f'推断角点: ({p3[0]:.7f},{p3[1]:.7f}) ({p4[0]:.7f},{p4[1]:.7f})')
            return [p1, p2, p3, p4]

        self.get_logger().error(f'角点数量不足 ({len(points)}), 无法补全矩形')
        return None

    # ============================================================
    # 目标获取与过滤
    #
    # 滤波策略 (6点窗口, 1m阈值):
    #   1. 连续采集6个目标坐标
    #   2. 后5个与第1个的距离均 ≤1m → 取6点平均值作为锁定目标
    #   3. 任一点超1m → 剔除异常点, 仅保留有效数据, 重新采集补足
    #   4. 超过90s仍未锁定 → 自动使用备用JSON文件
    # ============================================================

    def _read_and_filter_target(self, use_backup=False):
        path = self.backup_target_json if use_backup else self.target_json
        target = self._parse_target(path)

        if target is None:
            return

        if use_backup:
            self.filtered_target = target
            self.get_logger().info(f'使用备用目标: ({target[0]:.7f}, {target[1]:.7f})')
            return

        self.target_history.append(target)
        hlen = len(self.target_history)

        if hlen < self.stable_window:
            self.get_logger().info(
                f'目标收集中... {hlen}/{self.stable_window}',
                throttle_duration_sec=1.0
            )
            return

        stable = self._check_stable()
        if stable:
            self.filtered_target = self._avg_target()
            self.get_logger().info('目标稳定，取平均值')
        else:
            self._remove_outliers()
            self.get_logger().info('检测到异常值，已剔除')

    def _parse_target(self, path):
        """解析目标JSON: 支持 {"lat":xx,"lon":xx} 或 [lat,lon] 格式"""
        if not path.exists():
            return None
        if path.stat().st_size == 0:
            return None
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            if 'lat' in data and 'lon' in data:
                return (float(data['lat']), float(data['lon']))
            if isinstance(data, list) and len(data) >= 2:
                return (float(data[0]), float(data[1]))
        except Exception as e:
            self.get_logger().warn(f'解析目标文件失败: {e}')
        return None

    def _check_stable(self):
        if len(self.target_history) < self.stable_window:
            return False
        ref = self.target_history[0]
        for pt in list(self.target_history)[1:]:
            if self._gps_dist(ref[0], ref[1], pt[0], pt[1]) > self.max_pos_error:
                return False
        return True

    def _avg_target(self):
        pts = list(self.target_history)
        return (
            sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts),
        )

    def _remove_outliers(self):
        ref = self.target_history[0]
        valid = [ref]
        for pt in list(self.target_history)[1:]:
            d = self._gps_dist(ref[0], ref[1], pt[0], pt[1])
            if d <= self.max_pos_error:
                valid.append(pt)
        self.target_history.clear()
        for v in valid:
            self.target_history.append(v)

    # ============================================================
    # 物理仿真 — 弹药下落轨迹 (RK2中点法)
    #
    # 考虑因素:
    #   - 重力 G=9.81
    #   - 空气阻力 Fd = 0.5*ρ*Cd*A*v^2
    #   - 风对水平漂移的影响 (wind_e, wind_n)
    #
    # 返回: (下落时间, 北向漂移量, 东向漂移量)  单位: 秒/米
    # ============================================================

    def _simulate_drop(self):
        dt = 0.02
        z = FLIGHT_ALTITUDE
        vz = 0.0
        vn = 0.0
        ve = 0.0
        pos_n = 0.0
        pos_e = 0.0
        t = 0.0

        k = 0.5 * RHO * BOTTLE_CD * BOTTLE_AREA / BOTTLE_MASS

        while z > 0.0:
            drag_v = k * vz * abs(vz)
            az = -G + drag_v

            rel_vn = vn - self.wind_n
            rel_ve = ve - self.wind_e
            rel_v = math.sqrt(rel_vn**2 + rel_ve**2)
            if rel_v > 1e-6:
                drag_h = k * rel_v**2
                an = -drag_h * rel_vn / rel_v
                ae = -drag_h * rel_ve / rel_v
            else:
                an = 0.0
                ae = 0.0

            vz_mid = vz + az * dt * 0.5
            vn_mid = vn + an * dt * 0.5
            ve_mid = ve + ae * dt * 0.5
            z_mid = z + vz_mid * dt * 0.5

            if z_mid > 0:
                drag_v_mid = k * vz_mid * abs(vz_mid)
                az_mid = -G + drag_v_mid
                rel_vn_mid = vn_mid - self.wind_n
                rel_ve_mid = ve_mid - self.wind_e
                rel_v_mid = math.sqrt(rel_vn_mid**2 + rel_ve_mid**2)
                if rel_v_mid > 1e-6:
                    drag_h_mid = k * rel_v_mid**2
                    an_mid = -drag_h_mid * rel_vn_mid / rel_v_mid
                    ae_mid = -drag_h_mid * rel_ve_mid / rel_v_mid
                else:
                    an_mid = 0.0
                    ae_mid = 0.0

                vz += az_mid * dt
                vn += an_mid * dt
                ve += ae_mid * dt
            else:
                vz += az * dt
                vn += an * dt
                ve += ae * dt

            z += vz * dt
            pos_n += vn * dt
            pos_e += ve * dt
            z = max(z, 0.0)
            t += dt

            if t > 60.0:
                break

        return t, pos_n, pos_e

    # ============================================================
    # 航线生成
    #
    # 航线结构 (共9个航点):
    #   [0] 接近起点    — 安全区边界上, 正对目标方向
    #   [1-5] 预瞄准段  — 6个等距航点, 线性插值至投弹点
    #   [6] DO_SET_SERVO — 通道7 PWM=1900, 释放弹药
    #   [7] DO_SET_SERVO — 通道7 PWM=1100, 收回舵机
    #   [8] 脱离点      — 朝向安全区中心飞行, 转交返航航线
    #
    # 入口方向: 从安全区边界 (远离中心侧) 飞向目标
    # 出口方向: 投弹后飞向安全区中心 (返回安全位置)
    # 所有航点均经 _clamp_to_zone 约束在60m方形边界内
    # ============================================================

    def _generate_waypoints(self):
        if self.filtered_target is None or self.zone_center is None:
            return None
        if self.global_pos is None:
            return None

        tgt_lat, tgt_lon = self.filtered_target

        # --- 步骤1: 物理仿真, 计算弹药漂移量 ---
        fall_time, drift_n, drift_e = self._simulate_drop()

        # --- 步骤2: 逆向计算投弹释放点 ---
        # 目标 + 反方向偏移 (漂移量) = 释放点
        rls_lat, rls_lon = self._offset(
            tgt_lat, tgt_lon,
            (self._bearing_from_delta(drift_n, drift_e) + 180.0) % 360.0,
            math.hypot(drift_n, drift_e)
        )

        # --- 步骤3: 确定航线方向 ---
        # bearing: 从安全区中心指向释放点的方向
        bearing = self._bearing(self.zone_center[0], self.zone_center[1],
                                rls_lat, rls_lon)

        # --- 步骤4: 释放点约束至安全区内 ---
        rls_lat, rls_lon = self._clamp_to_zone(rls_lat, rls_lon)

        rls_dn = self._lat_diff_m(rls_lat, self.zone_center[0])
        rls_de = self._lon_diff_m(rls_lon, self.zone_center[1], self.zone_center[0])
        rls_dist = math.hypot(rls_dn, rls_de)

        # --- 步骤5: 计算接近起点 ---
        # 从安全区中心沿接近方向 (bearing+180°) 投影至边界附近
        approach_brg = (bearing + 180.0) % 360.0
        approach_start = self._offset(self.zone_center[0], self.zone_center[1],
                                      approach_brg, SAFE_ZONE_CORNER_DIST)
        approach_start = self._clamp_to_zone(approach_start[0], approach_start[1])

        # --- 步骤6: 计算脱离点 ---
        # 投弹后飞向安全区中心方向
        exit_brg = self._bearing(rls_lat, rls_lon,
                                 self.zone_center[0], self.zone_center[1])
        exit_dist = min(30.0, max(rls_dist - SAFE_MARGIN, 5.0))
        exit_pt = self._offset(rls_lat, rls_lon, exit_brg, exit_dist)
        exit_pt = self._clamp_to_zone(exit_pt[0], exit_pt[1])

        self.get_logger().info(
            f'下落时间={fall_time:.2f}s, 漂移(N={drift_n:.2f}m, E={drift_e:.2f}m), '
            f'投放点({rls_lat:.7f}, {rls_lon:.7f})'
        )

        waypoints = []

        # --- 预瞄准段: 6个等距航点 (从边界匀速飞至投弹点) ---
        approach_wps = 6
        for i in range(approach_wps):
            ratio = (i + 1) / approach_wps
            lat = approach_start[0] + (rls_lat - approach_start[0]) * ratio
            lon = approach_start[1] + (rls_lon - approach_start[1]) * ratio
            pt = self._clamp_to_zone(lat, lon)
            wp = self._make_wp(MAV_CMD_NAV_WAYPOINT, pt[0], pt[1], FLIGHT_ALTITUDE)
            waypoints.append(wp)

        # --- 投弹: 通道7舵机, PWM=1900 释放弹药 ---
        servo_drop = self._make_wp(MAV_CMD_DO_SET_SERVO, rls_lat, rls_lon, FLIGHT_ALTITUDE)
        servo_drop.param1 = 7.0      # AUX通道7
        servo_drop.param2 = 1900.0   # PWM高电平 → 释放
        waypoints.append(servo_drop)

        # --- 收回舵机: PWM=1100 ---
        servo_retract = self._make_wp(MAV_CMD_DO_SET_SERVO, rls_lat, rls_lon, FLIGHT_ALTITUDE)
        servo_retract.param1 = 7.0
        servo_retract.param2 = 1100.0
        waypoints.append(servo_retract)

        # --- 脱离: 飞向安全区中心 ---
        exit_wp = self._make_wp(MAV_CMD_NAV_WAYPOINT, exit_pt[0], exit_pt[1], FLIGHT_ALTITUDE)
        waypoints.append(exit_wp)

        if waypoints:
            waypoints[0].is_current = True

        self.get_logger().info(f'生成 {len(waypoints)} 个打击航点')
        return waypoints

    # ============================================================
    # 安全区边界约束
    #
    # _is_in_zone:  判定点是否在60m方形安全区内 (含SAFE_MARGIN内缩)
    # _clamp_to_zone: 若点在区外, 沿"中心→点"方向裁剪至方形边界
    #
    # 方形约束原理:
    #   1. 通过zone_corners确定方形朝向 (角点方向 -45° = 边方向)
    #   2. 旋转坐标系使方形轴对齐 (x', y')
    #   3. 检查 |x'| <= half_side 且 |y'| <= half_side
    #   4. 裁剪时等比缩放 (x', y') 至边界内
    # ============================================================

    def _is_in_zone(self, lat, lon):
        if self.zone_center is None:
            return True
        if not self.zone_ready or self.zone_corners is None or len(self.zone_corners) < 4:
            max_dist = SAFE_ZONE_HALF_SIDE - SAFE_MARGIN
            d = self._gps_dist(lat, lon, self.zone_center[0], self.zone_center[1])
            return d <= max_dist

        dn = self._lat_diff_m(lat, self.zone_center[0])
        de = self._lon_diff_m(lon, self.zone_center[1], self.zone_center[0])

        b_first = self._bearing(self.zone_center[0], self.zone_center[1],
                                self.zone_corners[0][0], self.zone_corners[0][1])
        axis_rad = math.radians(b_first - 45.0)

        cos_a = math.cos(axis_rad)
        sin_a = math.sin(axis_rad)
        x_rot = de * cos_a - dn * sin_a
        y_rot = de * sin_a + dn * cos_a

        limit = SAFE_ZONE_HALF_SIDE - SAFE_MARGIN
        return abs(x_rot) <= limit and abs(y_rot) <= limit

    def _clamp_to_zone(self, lat, lon):
        if self.zone_center is None:
            return (lat, lon)

        if not self.zone_ready or self.zone_corners is None or len(self.zone_corners) < 4:
            max_dist = SAFE_ZONE_HALF_SIDE - SAFE_MARGIN
            d = self._gps_dist(lat, lon, self.zone_center[0], self.zone_center[1])
            if d <= max_dist:
                return (lat, lon)
            bearing = self._bearing(self.zone_center[0], self.zone_center[1], lat, lon)
            pt = self._offset(self.zone_center[0], self.zone_center[1], bearing, max_dist)
            self.get_logger().warn(f'航点超出安全区{d:.1f}m, 已限制至{max_dist:.1f}m',
                                  throttle_duration_sec=2.0)
            return pt

        dn = self._lat_diff_m(lat, self.zone_center[0])
        de = self._lon_diff_m(lon, self.zone_center[1], self.zone_center[0])

        b_first = self._bearing(self.zone_center[0], self.zone_center[1],
                                self.zone_corners[0][0], self.zone_corners[0][1])
        axis_rad = math.radians(b_first - 45.0)

        cos_a = math.cos(axis_rad)
        sin_a = math.sin(axis_rad)
        x_rot = de * cos_a - dn * sin_a
        y_rot = de * sin_a + dn * cos_a

        limit = SAFE_ZONE_HALF_SIDE - SAFE_MARGIN
        if abs(x_rot) <= limit and abs(y_rot) <= limit:
            return (lat, lon)

        scale = min(limit / max(abs(x_rot), 1e-9),
                    limit / max(abs(y_rot), 1e-9))
        x_clamped = x_rot * scale
        y_clamped = y_rot * scale

        de_clamped = x_clamped * cos_a + y_clamped * sin_a
        dn_clamped = -x_clamped * sin_a + y_clamped * cos_a

        new_lat = self.zone_center[0] + self._m_to_lat(dn_clamped)
        new_lon = self.zone_center[1] + self._m_to_lon(de_clamped, self.zone_center[0])

        self.get_logger().warn(
            f'航点已裁剪至安全区方形边界内 (原距中心 {math.hypot(dn, de):.1f}m)',
            throttle_duration_sec=2.0)
        return (new_lat, new_lon)

    def _make_wp(self, cmd, lat, lon, alt):
        wp = Waypoint()
        wp.frame = MAV_FRAME_GLOBAL_REL_ALT
        wp.command = cmd
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        wp.autocontinue = True
        wp.is_current = False
        return wp

    # ============================================================
    # 航点发布
    #
    # 将生成的Waypoint列表封入WaypointList, 发布至 /secondary_waypoints
    # 外部节点 (如 task_manager) 订阅此话题, 在auto_mission模式下
    # 将航点推送到飞控
    # ============================================================

    def _publish(self, waypoints):
        wpl = WaypointList()
        wpl.waypoints = waypoints
        self.waypoint_pub.publish(wpl)
        self.get_logger().info(f'已发布 {len(waypoints)} 个航点至 /secondary_waypoints')

    # ============================================================
    # 地理计算工具
    #
    # _gps_dist:   Haversine公式 — 两点GPS距离 (米)
    # _bearing:   起点 → 终点的方位角 (0°=北, 顺时针)
    # _bearing_from_delta: 北/东向偏移量 → 方位角
    # _lat_diff_m: 纬度差 (度) → 米
    # _lon_diff_m: 经度差 (度) → 米 (含纬度余弦修正)
    # _m_to_lat:  米 → 纬度差 (度)
    # _m_to_lon:  米 → 经度差 (度)
    # _offset:    起点 + 方位角 + 距离 → 目标GPS坐标
    # ============================================================

    def _gps_dist(self, lat1, lon1, lat2, lon2):
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlon / 2) ** 2)
        return 2.0 * EARTH_RADIUS * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    def _bearing(self, lat1, lon1, lat2, lon2):
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        x = (math.cos(lat1_r) * math.sin(lat2_r) -
             math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))
        y = math.sin(dlon) * math.cos(lat2_r)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    def _bearing_from_delta(self, dn, de):
        return (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0

    def _lat_diff_m(self, lat, ref_lat):
        return (lat - ref_lat) * ONE_DEG_LAT_M

    def _lon_diff_m(self, lon, ref_lon, ref_lat):
        return (lon - ref_lon) * ONE_DEG_LAT_M * math.cos(math.radians(ref_lat))

    def _m_to_lat(self, dm):
        return dm / ONE_DEG_LAT_M

    def _m_to_lon(self, dm, ref_lat):
        return dm / (ONE_DEG_LAT_M * math.cos(math.radians(ref_lat)))

    def _offset(self, lat, lon, bearing, dist):
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)
        brg_r = math.radians(bearing)
        dr = dist / EARTH_RADIUS
        new_lat_r = math.asin(
            math.sin(lat_r) * math.cos(dr) +
            math.cos(lat_r) * math.sin(dr) * math.cos(brg_r)
        )
        new_lon_r = lon_r + math.atan2(
            math.sin(brg_r) * math.sin(dr) * math.cos(lat_r),
            math.cos(dr) - math.sin(lat_r) * math.sin(new_lat_r)
        )
        return (math.degrees(new_lat_r), math.degrees(new_lon_r))


def main(args=None):
    rclpy.init(args=args)
    node = CalculateSecondaryWaypoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
