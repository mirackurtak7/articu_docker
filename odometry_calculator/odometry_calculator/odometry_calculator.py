import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import numpy as np
import tf_transformations
import tf2_ros

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_calculator')

        # Parametreler (robot fiziksel özellikleri)
        self.R = 0.1016  # Tekerlek yarıçapı (m)
        self.L = 0.35    # Dingil mesafesi (m)
        self.dt = 0.005    # Zaman adımı (s)

        # Başlangıç pozisyonu
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # RPM topiclerine abone olma
        self.subscription_left = self.create_subscription(
            Float64, '/left_wheel_rpm', self.left_rpm_callback, 10)
        self.subscription_right = self.create_subscription(
            Float64, '/right_wheel_rpm', self.right_rpm_callback, 10)

        # Odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Son okunan RPM değerleri
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # Zamanlayıcı
        self.timer = self.create_timer(self.dt, self.compute_odometry)

        # TF broadcaster'ı oluştur (odom → base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def left_rpm_callback(self, msg):
        """Sol tekerlek RPM güncellemesi"""
        self.left_rpm = msg.data

    def right_rpm_callback(self, msg):
        """Sağ tekerlek RPM güncellemesi"""
        self.right_rpm = msg.data

    def compute_odometry(self):
        """Odometri hesaplayarak /odom topicine ve tf yayınlar"""
        # RPM'den doğrusal hız hesaplama
        v_left = (self.left_rpm / 60) * 2 * np.pi * self.R
        v_right = (self.right_rpm / 60) * 2 * np.pi * self.R

        # Lineer ve açısal hız hesaplama
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.L

        # Pozisyon güncelleme
        dx = v * np.cos(self.theta) * self.dt
        dy = v * np.sin(self.theta) * self.dt
        dtheta = omega * self.dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        # Euler açıdan quaternion dönüşümü
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # Odometry mesajı oluşturma
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pozisyon bilgileri
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0], y=quaternion[1],
            z=quaternion[2], w=quaternion[3]
        )

        # Hız bilgileri
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        # Odometry mesajını yayınla
        self.odom_publisher.publish(odom_msg)

        # TF yayınlamak için TransformStamped oluşturma
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Transformu yayınla
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"Published Odometry: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometryCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
