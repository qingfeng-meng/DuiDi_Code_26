#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandLong, CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
from vision_msgs.msg import TargetCoord

class OffboardServoSwing(Node):
    def __init__(self):
        super().__init__("offboard_servo_swing_node")
        # ====================== 用户配置区（按需修改） ======================
        self.home_lat = 28.2345678    # 安全悬停点纬度
        self.home_lon = 113.4567890   # 安全悬停点经度
        self.safe_rel_alt = 12.0      # 相对起飞高度(m)
        # 舵机参数
        self.servo_ch = 7             # 舵机硬件通道号
        self.pwm_left = 1100          # 左摆PWM
        self.pwm_mid = 1500           # 中间中立位
        self.pwm_right = 1900         # 右摆PWM
        self.swing_wait = 0.7         # 每个姿态停留时长(秒)
        # ==================================================================
        # 状态缓存
        self.fc_state = State()
        self.cur_gps = NavSatFix()
        self.offboard_ready = False
        self.swing_state = 0  # 0左 1中 2右
        self.send_target_count = 0

        # 1. Offboard位置目标发布器
        self.pos_pub = self.create_publisher(
            PositionTarget,
            "/mavros/setpoint_raw/local",
            10
        )
        # 2. 服务客户端
        # 舵机指令服务
        self.servo_cli = self.create_client(CommandLong, "/mavros/cmd/command_long")
        # 解锁服务
        self.arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        # 模式切换服务
        self.mode_cli = self.create_client(SetMode, "/mavros/set_mode")

        # 3. 话题订阅
        self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_cb, 10)

        # 主循环定时器 20Hz
        self.timer = self.create_timer(0.05, self.main_loop)
        self.get_logger().info("ROS2 Offboard舵机摆动测试节点启动完成")

        self.vis_pb = self.create_publisher(TargetCoord,"/vision/custom_target",10)

    def vision_pb(self):
        msg = TargetCoord()
        msg.p_x = 210.01
        msg.p_y = 302.02
        msg.conf = 0.9
        msg.class_id = 0
        self.vis_pb.publish(msg)
    def state_cb(self, msg: State):
        """飞控状态回调"""
        self.fc_state = msg

    def gps_cb(self, msg: NavSatFix):
        """GPS定位回调"""
        self.cur_gps = msg

    def wait_all_services(self):
        """阻塞等待所有mavros服务就绪"""
        while not self.servo_cli.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn("等待 command_long 舵机服务...")
        while not self.arm_cli.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn("等待 arming 解锁服务...")
        while not self.mode_cli.wait_for_service(timeout_sec=0.3):
            self.get_logger().warn("等待 set_mode 模式服务...")
        self.get_logger().info("全部MAVROS服务连接成功")

    def send_servo(self, channel: int, pwm: int):
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

    def publish_safe_target(self):
        """持续发布安全悬停目标点"""
        target = PositionTarget()
        target.coordinate_frame = PositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = 0b111111000111  # 仅位置控制，忽略速度/加速度
        target.latitude = self.home_lat
        target.longitude = self.home_lon
        target.altitude = self.safe_rel_alt
        self.pos_pub.publish(target)
        self.send_target_count += 1

    def switch_offboard(self):
        """切换至OFFBOARD模式并解锁"""
        # 先持续下发目标点20帧，满足PX4切换Offboard前置条件
        if self.send_target_count < 20:
            return
        # 已连接飞控
        if not self.fc_state.connected:
            self.get_logger().warn("飞控未连接，跳过模式切换")
            return
        # 已经是Offboard直接返回
        if self.fc_state.mode == "OFFBOARD" and self.fc_state.armed:
            self.offboard_ready = True
            return

        # 1. 切换OFFBOARD模式
        mode_req = SetMode.Request()
        mode_req.custom_mode = "OFFBOARD"
        mode_future = self.mode_cli.call_async(mode_req)
        rclpy.spin_until_future_complete(self, mode_future)
        mode_resp = mode_future.result()
        if mode_resp.mode_sent:
            self.get_logger().info("OFFBOARD模式切换成功")
        else:
            self.get_logger().warn("OFFBOARD切换失败，持续重试")
            return

        # 2. 解锁电机
        arm_req = CommandBool.Request()
        arm_req.value = True
        arm_future = self.arm_cli.call_async(arm_req)
        rclpy.spin_until_future_complete(self, arm_future)
        arm_resp = arm_future.result()
        if arm_resp.success:
            self.get_logger().info("电机解锁完成，进入定点悬停")
            self.offboard_ready = True
        else:
            self.get_logger().error("电机解锁失败，请检查飞控安全开关")

    def servo_swing_logic(self):
        """舵机往复摆动逻辑"""
        if self.swing_state == 0:
            self.send_servo(self.servo_ch, self.pwm_left)
            self.swing_state = 1
        elif self.swing_state == 1:
            self.send_servo(self.servo_ch, self.pwm_mid)
            self.swing_state = 2
        elif self.swing_state == 2:
            self.send_servo(self.servo_ch, self.pwm_right)
            self.swing_state = 0
        # 延时等待机械动作到位
        self.get_clock().sleep_for(Duration(seconds=self.swing_wait))

    def main_loop(self):
        """主循环定时器回调"""
        # 1. 持续下发安全定点（维持Offboard必备）
        self.publish_safe_target()

        # 2. 未进入Offboard时执行模式切换流程
        if not self.offboard_ready:
            self.switch_offboard()
            return

        # 3. Offboard就绪后执行舵机左右摆动
        self.servo_swing_logic()

    def shutdown_reset_servo(self):
        """程序退出前舵机回中安全处理"""
        self.get_logger().warn("程序退出，舵机复位至中立位")
        self.send_servo(self.servo_ch, self.pwm_mid)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardServoSwing()
    node.vision_pb()
    # 等待服务就绪
    node.wait_all_services()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown_reset_servo()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()