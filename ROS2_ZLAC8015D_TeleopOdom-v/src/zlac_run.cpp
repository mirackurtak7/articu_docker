#include <math.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "zlac8015d.h"

#define TWIST_SUB_TOPIC "cmd_vel"
#define JOINT_PUB_TOPIC "joint_states"
#define LEFT_RPM_TOPIC "left_wheel_rpm"
#define RIGHT_RPM_TOPIC "right_wheel_rpm"
#define JOINT_NAME_WHEEL_L "left_wheel_joint"
#define JOINT_NAME_WHEEL_R "right_wheel_joint"
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200
#define MODBUS_ID 0x01
#define DEBUG_ENABLE false
#define WHEEL_RADIUS 0.1016  // m
#define RPM_TO_RAD_S 0.1047  // (2 * M_PI) / 60

class ZLACController : public rclcpp::Node {
public:
    ZLACController() : Node("zlac8015d_node"), joint_positions_{0.0, 0.0} {
        motor_.init(SERIAL_PORT, BAUDRATE, MODBUS_ID, DEBUG_ENABLE);
        
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINT_PUB_TOPIC, 10);
        left_rpm_pub_ = create_publisher<std_msgs::msg::Float64>(LEFT_RPM_TOPIC, 10);
        right_rpm_pub_ = create_publisher<std_msgs::msg::Float64>(RIGHT_RPM_TOPIC, 10);

        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            TWIST_SUB_TOPIC, 10, std::bind(&ZLACController::cmdVelCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&ZLACController::publishJointStates, this));
    }

    ~ZLACController() {
        motor_.terminate();
    }

private:
    ZLAC motor_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rpm_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Eklem konumlarını tutacak vektör (başlangıçta sıfır)
    std::vector<double> joint_positions_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        const double wheel_separation = 0.35;
        const double left_rpm = (msg->linear.x - msg->angular.z * wheel_separation / 2.0) * (60.0 / (2 * M_PI * WHEEL_RADIUS));
        const double right_rpm = (msg->linear.x + msg->angular.z * wheel_separation / 2.0) * (60.0 / (2 * M_PI * WHEEL_RADIUS));

        motor_.set_double_rpm(left_rpm, right_rpm);
    }

    void publishJointStates() {
        auto rpm_stat = motor_.get_rpm();

        // Joint States Yayını
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = now();
        joint_msg.name = {JOINT_NAME_WHEEL_L, JOINT_NAME_WHEEL_R};

        // RPM'lerden rad/s dönüşümünü hesapla
        double left_rad_per_sec = rpm_stat.rpm_L * RPM_TO_RAD_S;
        double right_rad_per_sec = rpm_stat.rpm_R * RPM_TO_RAD_S;
        joint_msg.velocity = {left_rad_per_sec, right_rad_per_sec};

        // 50 ms (0.05 s) periyot kullanılarak konumları entegre et
        const double dt = 0.05;
        joint_positions_[0] += left_rad_per_sec * dt;
        joint_positions_[1] += right_rad_per_sec * dt;
        joint_msg.position = joint_positions_;

        joint_pub_->publish(joint_msg);

        // RPM'leri Ayrı Topic'lerde Yayınla
        std_msgs::msg::Float64 left_rpm_msg;
        left_rpm_msg.data = rpm_stat.rpm_L;
        left_rpm_pub_->publish(left_rpm_msg);

        std_msgs::msg::Float64 right_rpm_msg;
        right_rpm_msg.data = rpm_stat.rpm_R;
        right_rpm_pub_->publish(right_rpm_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZLACController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
