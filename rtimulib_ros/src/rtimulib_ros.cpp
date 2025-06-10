#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <RTIMULib.h>

class ImuDriverNode : public rclcpp::Node
{
public:
    ImuDriverNode() : Node("rtimulib_node")
    {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

        this->declare_parameter("calibration_file_path");
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("update_rate", 100.0);

        this->get_parameter("calibration_file_path", path_calib_file_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("update_rate", update_rate_);

        settings_ = std::make_unique<RTIMUSettings>(path_calib_file_.c_str(), "RTIMULib");
        imu_ = RTIMU::createIMU(settings_.get());

        if (!imu_ || imu_->IMUType() == RTIMU_TYPE_NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "No Imu found");
            rclcpp::shutdown();
        }

        imu_->IMUInit();
        imu_->setSlerpPower(0.02);
        imu_->setGyroEnable(true);
        imu_->setAccelEnable(true);
        imu_->setCompassEnable(false);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            std::bind(&ImuDriverNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        sensor_msgs::msg::Imu imu_msg;

        if (imu_->IMURead())
        {
            RTIMU_DATA imu_data = imu_->getIMUData();

            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = frame_id_;
            imu_msg.orientation.x = imu_data.fusionQPose.x();
            imu_msg.orientation.y = imu_data.fusionQPose.y();
            imu_msg.orientation.z = imu_data.fusionQPose.z();
            imu_msg.orientation.w = imu_data.fusionQPose.scalar();
            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();
            imu_msg.linear_acceleration.x = imu_data.accel.x();
            imu_msg.linear_acceleration.y = imu_data.accel.y();
            imu_msg.linear_acceleration.z = imu_data.accel.z();
            imu_msg.orientation_covariance = {
                0.01, 0,    0, 
                0,    0.01, 0, 
                0,    0,    0.01
            };

            imu_msg.angular_velocity_covariance = {
                0.0001, 0,     0, 
                0,      0.0001, 0, 
                0,      0,     0.0001
            };

            imu_msg.linear_acceleration_covariance = {
                0.05, 0,   0, 
                0,    0.05, 0, 
                0,    0,   0.05
            };
            imu_publisher_->publish(imu_msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<RTIMUSettings> settings_;
    RTIMU *imu_;
    std::string path_calib_file_;
    std::string frame_id_;
    double update_rate_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuDriverNode>());
    rclcpp::shutdown();
    return 0;
}
