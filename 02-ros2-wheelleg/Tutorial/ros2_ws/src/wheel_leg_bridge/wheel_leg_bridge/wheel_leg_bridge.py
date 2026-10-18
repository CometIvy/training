"""Wheel_Leg MuJoCo 仿真接入 ROS2 的桥接节点。

功能：
- 加载 MuJoCo 模型（robot.xml + obstacles.xml）
- 定时步进仿真
- 发布 /imu (sensor_msgs/Imu) 与 /joint_states (sensor_msgs/JointState)
- 订阅 /cmd_vel (geometry_msgs/Twist) 驱动左右轮
"""
import os
import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


class WheelLegBridge(Node):
    def __init__(self):
        super().__init__('wheel_leg_bridge')

        # 模型路径参数（默认取包内 robots/robot.xml）
        self.declare_parameter('robot_xml', '')
        xml = self.get_parameter('robot_xml').value
        if not xml:
            xml = os.path.join(
                get_package_share_directory('wheel_leg_bridge'),
                'robots', 'robot.xml')

        # 加载 MuJoCo 模型与数据
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep  # 0.002 s
        self.get_logger().info(f'已加载模型: {xml} (dt={self.dt}s)')

        # 发布者：IMU、关节状态
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 订阅者：速度指令
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.left_torque = 0.0
        self.right_torque = 0.0
        self.track = 0.183  # 左右轮间距 (m)

        # 控制循环：50 Hz，每次步进 10 步（10 * 0.002 = 0.02 s）
        self.timer = self.create_timer(0.02, self.step)

    def cmd_vel_cb(self, msg):
        """把 /cmd_vel 映射为左右轮力矩（差速）。

        ⚠️ 关键：模型里左右轮是**镜像安装**的 —— 两个轮关节的本地轴都是 (0,0,1)，
        但世界坐标下的转轴是 right=(0,-1,0)、left=(0,+1,0)，点积 −1。

        所以同一个力矩会让两轮在世界上**反向**旋转。想让它「共同前进」，
        两轮的关节力矩必须**反号**；而「原地转向」才是**同号**。

        这也是原版 robot_control_main.py 写 left = -wheel_T、right = +wheel_T 的原因。
        """
        k = 1.0
        forward = k * msg.linear.x                          # 共同前进分量
        yaw = k * msg.angular.z * self.track / 2.0          # 差速分量
        self.right_torque = forward - yaw
        self.left_torque = -forward - yaw

    def step(self):
        # 执行器顺序：0-3 为腿关节，4=右轮，5=左轮（对应 XML 中 motor 定义顺序）
        self.data.ctrl[0:4] = 0.0
        self.data.ctrl[4] = self.right_torque
        self.data.ctrl[5] = self.left_torque

        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        self.publish_imu()
        self.publish_joints()

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'

        # MuJoCo framequat 返回 [w, x, y, z]；ROS 四元数为 [x, y, z, w]
        w, x, y, z = self.data.sensor('orientation').data
        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)
        msg.orientation.w = float(w)

        gx, gy, gz = self.data.sensor('angular_vel').data
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        self.imu_pub.publish(msg)

    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = ['right1', 'right2', 'left1', 'left2', 'rightwheel', 'leftwheel']
        for n in names:
            msg.name.append(n)
            msg.position.append(float(self.data.sensor(n + '_pos').data[0]))
            msg.velocity.append(float(self.data.sensor(n + '_vel').data[0]))
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
