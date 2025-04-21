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
#define VOLTAGE_TOPIC "battery_voltage"
#define JOINT_NAME_WHEEL_L "left_wheel_joint"
#define JOINT_NAME_WHEEL_R "right_wheel_joint"
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200
#define MODBUS_ID 0x01
#define DEBUG_ENABLE false
#define WHEEL_RADIUS 0.08255  // m
#define RPM_TO_RAD_S 0.1047  // (2 * M_PI) / 60

class ZLACController : public rclcpp::Node {
public:
    ZLACController() : Node("zlac8015d_node"), joint_positions_{0.0, 0.0} {
        motor_.init(SERIAL_PORT, BAUDRATE, MODBUS_ID, DEBUG_ENABLE);
        
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINT_PUB_TOPIC, 10);
        left_rpm_pub_ = create_publisher<std_msgs::msg::Float64>(LEFT_RPM_TOPIC, 10);
        right_rpm_pub_ = create_publisher<std_msgs::msg::Float64>(RIGHT_RPM_TOPIC, 10);
        voltage_pub_ = create_publisher<std_msgs::msg::Float64>(VOLTAGE_TOPIC, 10);

        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            TWIST_SUB_TOPIC, 10, std::bind(&ZLACController::cmdVelCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
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
        
        // Publish joint states
        auto joint_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_msg->header.stamp = now();
        joint_msg->name = {JOINT_NAME_WHEEL_L, JOINT_NAME_WHEEL_R};
        joint_msg->position = joint_positions_;
        joint_msg->velocity = {
            rpm_stat.rpm_L * RPM_TO_RAD_S,
            rpm_stat.rpm_R * RPM_TO_RAD_S
        };
        joint_pub_->publish(std::move(joint_msg));

        // Publish RPM values
        std_msgs::msg::Float64 left_rpm_msg;
        left_rpm_msg.data = rpm_stat.rpm_L;
        left_rpm_pub_->publish(left_rpm_msg);

        std_msgs::msg::Float64 right_rpm_msg;
        right_rpm_msg.data = rpm_stat.rpm_R;
        right_rpm_pub_->publish(right_rpm_msg);

        // Publish voltage
        std_msgs::msg::Float64 voltage_msg;
        float voltage = motor_.read_voltage();
        if (voltage >= 0) {
            voltage_msg.data = voltage;
            voltage_pub_->publish(voltage_msg);
        } else {
            RCLCPP_WARN(get_logger(), "Failed to read voltage from motor driver");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZLACController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
