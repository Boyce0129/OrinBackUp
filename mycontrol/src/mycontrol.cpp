// ============================ CHANGELOG =====================================
// [A] 採用ENU坐標系，IMU和GPS皆在本程式轉換
// ============================================================================

#include <iostream>
#include <sstream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <mutex>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <csignal>
#include <sys/select.h>
#include <fcntl.h>
#include <cmath>
#include <iomanip>
#include <vector>
#include <stdexcept>
#include <utility>
#include <fstream>
#include <limits>
#include <deque>
#include <atomic>
#include <termios.h>

extern "C" {
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

// PCA9685 Registers
#define MODE1 0x00
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

// I2C Address of PCA9685
#define PCA9685_ADDRESS 0x40

// ====== 共享指令狀態 ======
std::atomic<bool> running{true};
std::atomic<bool> sbus_running{true};

void signal_handler(int signal) {
    if (signal == SIGINT) {
        running = false;
        sbus_running = false;
        rclcpp::shutdown();
    }
}

// ====== ROS2: 訂閱 IMU 與 RTK Odom ======
class MultiTopicSubscriber : public rclcpp::Node {
public:
    MultiTopicSubscriber()
    : Node("multi_topic_subscriber") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&MultiTopicSubscriber::imuCallback, this, std::placeholders::_1));
        gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "rtk/odom", 10, std::bind(&MultiTopicSubscriber::gpsCallback, this, std::placeholders::_1));

        last_imu_data_.orientation.w = 1.0;
        last_imu_data_.orientation.x = 0.0;
        last_imu_data_.orientation.y = 0.0;
        last_imu_data_.orientation.z = 0.0;
        last_imu_data_.angular_velocity.x = 0.0;
        last_imu_data_.angular_velocity.y = 0.0;
        last_imu_data_.angular_velocity.z = 0.0;

        last_gps_data_.pose.pose.position.x = 0.0;
        last_gps_data_.pose.pose.position.y = 0.0;
        last_gps_data_.pose.pose.position.z = 0.0;
        last_gps_data_.twist.twist.linear.x = 0.0;
        last_gps_data_.twist.twist.linear.y = 0.0;
        last_gps_data_.twist.twist.linear.z = 0.0;

        last_imu_timestamp_ = 0.0;
        last_gps_timestamp_ = 0.0;
    }

    double get_last_imu_timestamp() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_imu_timestamp_;
    }

    sensor_msgs::msg::Imu get_last_imu_data() {
        std::lock_guard<std::mutex> lock(mutex_);
        new_imu_data_available_ = false;
        return last_imu_data_;
    }

    double get_last_gps_timestamp() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_gps_timestamp_;
    }

    nav_msgs::msg::Odometry get_last_gps_data() {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_gps_data_;
    }

    bool new_imu_data_available() const {
        return new_imu_data_available_;
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        new_imu_data_available_ = true;
        last_imu_data_ = *msg;
        last_imu_timestamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    }

    void gpsCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_gps_data_ = *msg;
        last_gps_timestamp_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    mutable std::mutex mutex_;
    sensor_msgs::msg::Imu last_imu_data_;
    nav_msgs::msg::Odometry last_gps_data_;
    double last_imu_timestamp_;
    double last_gps_timestamp_;
    bool new_imu_data_available_ = false;
};

// ====== FOHP 一階＋半帶寬差分器 ======
class FOHPDerivative {
private:
    double y[3];
    double dt;
    double alpha;
public:
    FOHPDerivative(double dt_init, double alpha_init) : dt(dt_init), alpha(alpha_init) {
        y[0] = y[1] = y[2] = 0;
    }
    void updateDt(double new_dt) {
        if (new_dt > 0) dt = new_dt;
    }
    double update(double new_value) {
        y[0] = y[1];
        y[1] = y[2];
        y[2] = new_value;
        return (1 - alpha) * (y[2] - y[1]) / dt + alpha * (y[2] - y[0]) / (2 * dt);
    }
};

// ====== PCA9685 基本函式 ======
void writeRegister(int file, int registerAddress, int value) {
    char buffer[2];
    buffer[0] = registerAddress;
    buffer[1] = value;
    if (write(file, buffer, 2) != 2) {
        std::cerr << "Failed to write to the i2c bus.\n";
    }
}

void setPWMFreq(int file, int freq) {
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= float(freq);
    prescaleval -= 1.0;
    int prescale = int(prescaleval + 0.5);

    int oldmode = i2c_smbus_read_byte_data(file, MODE1);
    int newmode = (oldmode & 0x7F) | 0x10;
    writeRegister(file, MODE1, newmode);
    writeRegister(file, PRESCALE, prescale);
    writeRegister(file, MODE1, oldmode);
    usleep(5000);
    writeRegister(file, MODE1, oldmode | 0x80);
}

void setPWM(int file, int channel, int on, int off) {
    writeRegister(file, LED0_ON_L + 4 * channel, on & 0xFF);
    writeRegister(file, LED0_ON_H + 4 * channel, on >> 8);
    writeRegister(file, LED0_OFF_L + 4 * channel, off & 0xFF);
    writeRegister(file, LED0_OFF_H + 4 * channel, off >> 8);
}

// ====== 姿態工具 ======
void quaternionToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw) {
    roll  = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    pitch = (-1) * (std::asin(2.0 * (qw * qy - qz * qx))) + 0.022; // 你的偏置保留
    yaw   = (-1) * std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

static inline double wrapToPi(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}

// prev_unwrapped 是連續角；new_wrapped 是 (-pi, pi] 的角
static inline double unwrapAngle(double prev_unwrapped, double new_wrapped) {
    // 用 prev 的「當前 wrap」來算最短角差
    double prev_wrapped = wrapToPi(prev_unwrapped);
    double delta = wrapToPi(new_wrapped - prev_wrapped);
    return prev_unwrapped + delta;
}

// ====== NDOB ======
void ndob_x(double x_Dot, double phi, double theta, double psi, double dt, double U1, double z_x_old, double& d_x, double& z_x, double l_x) {
    const double m = 1.82, g = 9.807, Cv = 0.01;
    double p_x = l_x * x_Dot;
    double z_x_Dot = l_x * (((-cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * U1 / m) + Cv * (x_Dot / m)) - l_x * z_x_old - l_x * p_x;
    z_x = z_x_Dot * dt;
    d_x = z_x + p_x;
}
void ndob_y(double y_Dot, double phi, double theta, double psi, double dt, double U1, double z_y_old, double& d_y, double& z_y, double l_y) {
    const double m = 1.82, g = 9.807, Cv = 0.01;
    double p_y = l_y * y_Dot;
    double z_y_Dot = l_y * (((-cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * U1 / m) + Cv * (y_Dot / m)) - l_y * z_y_old - l_y * p_y;
    z_y = z_y_Dot * dt;
    d_y = z_y + p_y;
}
void ndob_z(double z_Dot, double phi, double theta, double dt, double U1, double z_z_old, double& d_z, double& z_z, double l_z) {
    const double m = 1.82, g = 9.807, Cv = 0.01;
    double p_z = l_z * z_Dot;
    double z_z_Dot = l_z * ((g + -(cos(phi) * cos(theta) * U1 / m)) + Cv * (z_Dot / m)) - l_z * z_z_old - l_z * p_z;
    z_z = z_z_Dot * dt;
    d_z = z_z + p_z;
}
void ndob_phi(double phi_Dot, double theta_Dot, double psi_Dot, double dt, double u_phi, double omega, double z_phi_old, double& d_phi, double& z_phi, double l_phi) {
    const double Ixx = 0.015326, Iyy = 0.014583, Izz = 0.017339, Jtp = 4.103e-4;
    double p_phi = l_phi * phi_Dot;
    double z_phi_Dot = (l_phi * (((-(Iyy - Izz) * theta_Dot * psi_Dot) / Ixx) + ((Jtp / Ixx) * theta_Dot * omega) - (u_phi))) - (l_phi * z_phi_old) - (l_phi * p_phi);
    z_phi = z_phi_Dot * dt;
    d_phi = z_phi + p_phi;
}
void ndob_theta(double phi_Dot, double theta_Dot, double psi_Dot, double dt, double u_theta, double omega, double z_theta_old, double& d_theta, double& z_theta, double l_theta) {
    const double Ixx = 0.015326, Iyy = 0.014583, Izz = 0.017339, Jtp = 4.103e-4;
    double p_theta = l_theta * theta_Dot;
    double z_theta_Dot = (l_theta * (((-(Izz - Ixx) * phi_Dot * psi_Dot) / Iyy) - ((Jtp / Iyy) * phi_Dot * omega) - (u_theta))) - (l_theta * z_theta_old) - (l_theta * p_theta);
    z_theta = z_theta_Dot * dt;
    d_theta = z_theta + p_theta;
}
void ndob_psi(double phi_Dot, double theta_Dot, double psi_Dot, double dt, double u_psi, double omega, double z_psi_old, double& d_psi, double& z_psi, double l_psi) {
    const double Ixx = 0.015326, Iyy = 0.014583, Izz = 0.017339, Jtp = 4.103e-4;
    double p_psi = l_psi * psi_Dot;
    double z_psi_Dot = (l_psi * (((-(Ixx - Iyy) * theta_Dot * phi_Dot) / Izz) - (u_psi))) - (l_psi * z_psi_old) - (l_psi * p_psi);
    z_psi = z_psi_Dot * dt;
    d_psi = z_psi + p_psi;
}

std::atomic<float> ch_norm[5];

static inline float apply_deadband(float x, float db = 0.05f) { // 設定遙控器的deadband
    if (db <= 0.0f) return x;
    float ax = std::fabs(x);
    if (ax <= db) return 0.0f;

    float sign = (x >= 0.0f) ? 1.0f : -1.0f; // 有做 rescale ，+-0.2範圍內還是有值
    return sign * ((ax - db) / (1.0f - db));
}

void init_sbus_vars() {
    for (int i = 0; i < 5; ++i) {
        ch_norm[i].store(0.0f);
    }
}

void sbusThread() {
    const char* PORT_PATH = "/dev/ttyUSB0";
    constexpr size_t FRAME_SIZE = 35;
    constexpr uint8_t START_BYTE = 0x0F;
    constexpr int BAUD = B115200;

    auto open_serial = [&]() {
        int fd = open(PORT_PATH, O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) { 
            perror("SBUS open");
            std::cerr << "⚠️ 無法開啟 " << PORT_PATH << std::endl;
            return -1; 
        }

        termios tio{};
        cfmakeraw(&tio);
        cfsetispeed(&tio, BAUD);
        cfsetospeed(&tio, BAUD);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tio);
        tcflush(fd, TCIFLUSH);

        // 設置 DTR 和 RTS 信號（關鍵！）
        int mstat;
        if (ioctl(fd, TIOCMGET, &mstat) == 0) {
            mstat |= TIOCM_DTR;   // DTR 拉高
            mstat &= ~TIOCM_RTS;  // RTS 拉低
            ioctl(fd, TIOCMSET, &mstat);
        }
        
        // 等待串口穩定
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        std::cout << "✓ SBUS 串口已開啟" << std::endl;
        return fd;
    };

    std::cout << "啟動 SBUS 執行緒..." << std::endl;
    int fd = open_serial();
    if (fd < 0) {
        std::cerr << "❌ SBUS 執行緒無法啟動" << std::endl;
        sbus_running = false;
        return;
    }

    std::vector<uint8_t> buf;
    buf.reserve(1024);  // 增加緩衝區大小

    constexpr int raw_min[5] = {311, 306, 308, 306, 306};
    constexpr int raw_max[5] = {1693, 1688, 1730, 1693, 1694};

    uint8_t frame[FRAME_SIZE];
    auto last_rx = std::chrono::steady_clock::now();
    bool first_frame = true;
    int reconnect_count = 0;

    std::cout << "等待 SBUS 訊號同步..." << std::flush;

    while (sbus_running) {
        uint8_t tmp[256];  // 增加讀取緩衝區
        ssize_t n = read(fd, tmp, sizeof(tmp));
        
        if (n > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
            last_rx = std::chrono::steady_clock::now();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        // === 自動重連機制：0.5秒沒資料就重開串口 ===
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_rx).count() > 0.5) {
            if (!first_frame) {
                std::cout << "\n⚠️ SBUS 訊號中斷，重新連接中..." << std::flush;
            } else {
                std::cout << " 重試中..." << std::flush;
            }
            
            close(fd);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            fd = open_serial();
            
            if (fd < 0) {
                reconnect_count++;
                if (reconnect_count > 10) {
                    std::cerr << "\n❌ SBUS 重連失敗次數過多，執行緒終止" << std::endl;
                    sbus_running = false;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            
            buf.clear();
            last_rx = std::chrono::steady_clock::now();
            reconnect_count = 0;
            continue;
        }

        if (buf.size() < FRAME_SIZE) continue;

        auto it = std::find(buf.begin(), buf.end(), START_BYTE);
        if (it == buf.end()) { 
            buf.clear(); 
            continue; 
        }

        size_t idx = it - buf.begin();
        if (buf.size() - idx < FRAME_SIZE) continue;

        memcpy(frame, &buf[idx], FRAME_SIZE);
        buf.erase(buf.begin(), buf.begin() + idx + FRAME_SIZE);

        if (frame[0] != START_BYTE) continue;

        // XOR 校驗
        uint8_t xor_val = 0;
        for (int i = 1; i <= 33; ++i) xor_val ^= frame[i];
        if (xor_val != frame[34]) continue;

        // 解析並正規化通道數據 + deadband（只對 CH0/1/2）
        for (int i = 0; i < 5; ++i) {
            uint16_t raw = (uint16_t(frame[1 + 2 * i]) << 8) | frame[2 + 2 * i];
            float scale = 2.0f / float(raw_max[i] - raw_min[i]);

            float v = (raw - raw_min[i]) * scale - 1.0f;   // 原本的正規化

            // CH0/1/2：加 deadband = 0.2；CH4（switching）不動，避免破壞你的門檻判斷
            if (i == 0 || i == 1 || i == 2) {
                v = apply_deadband(v, 0.2f);
            }

            ch_norm[i].store(v);
        }

        // 首次接收到有效幀
        if (first_frame) {
            std::cout << " 完成！" << std::endl;
            std::cout << "✓ SBUS 開始接收數據 (CH5 = " 
                      << std::fixed << std::setprecision(3) 
                      << ch_norm[4].load() << ")" << std::endl;
            first_frame = false;
        }

        // 檢查 flag 狀態（failsafe 等）
        uint8_t flags = frame[33];
        if (flags & 0x20) {
            static auto last_warning = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_warning).count() > 2.0) {
                std::cerr << "⚠️ SBUS 訊號丟失" << std::endl;
                last_warning = now;
            }
        }
        if (flags & 0x10) {
            static auto last_warning = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_warning).count() > 2.0) {
                std::cerr << "⚠️ SBUS Failsafe 觸發" << std::endl;
                last_warning = now;
            }
        }
    }

    close(fd);
    std::cout << "SBUS 執行緒已停止" << std::endl;
}


// ====== 主控制執行緒 ======
struct VariablesGroup {
    std::vector<std::string> names;
    std::vector<double> group;
    VariablesGroup(const std::vector<std::string>& n, const std::vector<double>& g) : names(n), group(g) {}
    VariablesGroup() = default;
};

void droneControlThread(int file, const std::shared_ptr<MultiTopicSubscriber>& node, std::ofstream& outFile1, std::ofstream& outFile2) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    static auto program_start = std::chrono::high_resolution_clock::now();

    std::chrono::time_point<std::chrono::high_resolution_clock> last_position_update = program_start;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_attitude_update = program_start;

    const std::chrono::milliseconds position_update_interval(100); // 10Hz
    const std::chrono::milliseconds attitude_update_interval(10);  // 100Hz
    const std::chrono::milliseconds gps_timeout(1000);
    const std::chrono::seconds imu_detection_delay(5);
    const std::chrono::milliseconds imu_timeout(100);

    const float max_angle = 15.0f * M_PI / 180.0f;
    const double XY_RADIUS_LIMIT = 0.6;   // 若有使用 XY 超界（僅關機保護）
    const double Z_RADIUS_LIMIT  = 0.35;   // 0.15 m
    const double Z_VELOCITY_LIMIT = 0.4;   // m/s


    std::mutex dataMutex;

    // 保留：IMU相關
    std::atomic<bool> imu_loss_detected{false};
    std::atomic<bool> attitude_invalid{false};

    // （僅保護用）XYZ 超界
    std::atomic<bool> xy_exceeded{false};
    std::atomic<bool> z_exceeded{false};
    std::atomic<bool> z_vel_exceeded{false};  // Z 速度是否超界


    // 位置狀態（只需 z，x/y 讀回但不控制）
    double current_x = 0.0, current_y = 0.0, current_z = 0.0;

    // 姿態量測：roll/pitch 用原本；yaw 改用 psi（連續角）
    float roll = 0.0f, pitch = 0.0f;
    double psi = 0.0;     // 連續角（相對 psi0）
    double psi0 = 0.0;    // 初始化 100 筆平均得到的偏移（wrapped）

    // GPS 新/舊時間戳追蹤（新的 GPS 丟失偵測法）
    double last_gps_timestamp_seen = 0.0;
    auto   last_gps_wallclock = program_start;

    static double last_position_time = 0.0;
    static double last_attitude_time = 0.0;

    bool control_mode_on = false, motors_on = false;

    // ====== 目標與命令 ======
    double x_target = 0.0, y_target = 0.0, z_target = 0.2;
    double x_command = 0.0, y_command = 0.0, z_command = 0.0;

    static double stored_uz = 0.0;

    // FOHP 導數器（僅 z + 姿態）
    FOHPDerivative x_command_Derivative(0.1, 0.5);
    FOHPDerivative y_command_Derivative(0.1, 0.5);
    FOHPDerivative z_command_Derivative(0.1, 0.5);
    FOHPDerivative x_command_sec_Derivative(0.1, 0.5);
    FOHPDerivative y_command_sec_Derivative(0.1, 0.5);
    FOHPDerivative z_command_sec_Derivative(0.1, 0.5);
    FOHPDerivative phi_command_Derivative(0.01, 0.5);
    FOHPDerivative theta_command_Derivative(0.01, 0.5);
    FOHPDerivative psi_command_Derivative(0.01, 0.5);
    FOHPDerivative phi_command_sec_Derivative(0.01, 0.5);
    FOHPDerivative theta_command_sec_Derivative(0.01, 0.5);
    FOHPDerivative psi_command_sec_Derivative(0.01, 0.5);

    // FOHPDerivative vz_e_derivative(0.1, 0.5);  // 初始化微分計算器，dt_position 和 alpha 可以根據需要調整


    // 參數
    const double Ixx = 0.015326, Iyy = 0.014583, Izz = 0.017339;
    const double m = 1.82, g = 9.807, Jtp = 4.103e-4, Cv = 0.01;
    const double WEIGHT_FORCE = m * g; // N（GPS fallback 用）
    const double bound = 0.1;
    const double position_command_bound_derivative = 0.1;  // m/s
    const double position_command_bound_sec_derivative = 1.0; // m/s^2
    const double attitude_command_bound_derivative = 3.1416;
    const double attitude_command_bound_sec_derivative = 31.4159;

    // SMC gains
    std::vector<double> C = {1.5, 1.5, 1, 4, 4, 4};
    std::vector<double> K = {2, 2, 1, 6, 6, 3};
    std::vector<double> Eta = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    std::vector<double> L = {1, 1, 1, 7, 7, 3};
    std::vector<double> PID = {1.5, 1.0, 0.3, 0.1, 0.5, -0.5};
    double I_vz_e = 0.0; // 積分項初始化

    // 位置控制比例增益
    // 速度控制比例增益
    // 速度控制積分增益
    // 速度控制微分增益
    // 積分項最大值（根據需求調整）
    // 積分項最小值（根據需求調整）


    outFile1 << "滑模參數C: ";
    for (double param : C) {
        outFile1 << param << " ";
    }
    outFile1 << "\n";

    outFile1 << "滑模參數K: ";
    for (double param : K) {
        outFile1 << param << " ";
    }
    outFile1 << "\n";

    outFile1 << "滑模參數Eta: ";
    for (double param : Eta) {
        outFile1 << param << " ";
    }
    outFile1 << "\n";

    outFile1 << "NDOB參數L: ";
    for (double param : L) {
        outFile1 << param << " ";
    }
    outFile1 << "\n";

    outFile1 << "PID參數: ";
    for (double param : PID) {
        outFile1 << param << " ";
    }
    outFile1 << "\n";

    outFile1 << "------------------------------------------------------------";
    outFile1 << "\n";

    outFile1.flush();

    // log 緩衝
    std::stringstream buffer_outFile1, buffer_outFile2;
    int buffer_flush_counter = 0;
    const int buffer_flush_frequency = 10;

    // 姿態命令（固定水平 + 固定初始 yaw）
    double psi_command_initial = 0.0;
    double theta_command = 0.0, phi_command = 0.0, psi_command = 0.0;

    // 控制輸出 / 擾動
    double U1 = 0.0, U2 = 0.0, U3 = 0.0, U4 = 0.0, omega = 0.0;
    double d_x = 0.0, z_x = 0.0, z_x_new = 0.0;
    double d_y = 0.0, z_y = 0.0, z_y_new = 0.0;
    double d_z = 0.0, z_z = 0.0, z_z_new = 0.0;
    double d_phi = 0.0, z_phi = 0.0, z_phi_new = 0.0;
    double d_theta = 0.0, z_theta = 0.0, z_theta_new = 0.0;
    double d_psi = 0.0, z_psi = 0.0, z_psi_new = 0.0;

    double max_force = 9.0, min_force = 2.5;
    double max_sign = 800.0, min_sign = 600.0;
    int index_position = 1, index_attitude = 1;

    // IMU 偵測
    double last_checked_imu_timestamp = 0.0;
    int imu_loss_counter = 0;
    const int imu_loss_threshold = 10;
    std::deque<double> imu_intervals;
    const size_t max_intervals = 10;

    // SUBS相關參數
    float xy_bound = 0.5; //m
    float z_bound = 3.0; //m


    while (ch_norm[4].load() >= -0.8f && running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // 可選：每隔一段時間打印當前值
        static int wait_counter = 0;
        wait_counter++;
        if (wait_counter % 10 == 0) {  // 每秒打印一次
            std::cout << "請將撥桿向下撥,當前通道值: "<< ch_norm[4].load() << std::endl;
        }
    }

    // 初始化 psi0：取 100 筆 IMU yaw 的 circular mean，避免跨 ±pi 平均錯
    float initial_roll, initial_pitch, initial_yaw;
    double sum_sin = 0.0, sum_cos = 0.0;

    for (int i = 0; i < 100; i++) {
        while (!node->new_imu_data_available())
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto initial_imu_data = node->get_last_imu_data();
        quaternionToEuler(initial_imu_data.orientation.x, initial_imu_data.orientation.y,
                          initial_imu_data.orientation.z, initial_imu_data.orientation.w,
                          initial_roll, initial_pitch, initial_yaw);

        sum_sin += std::sin((double)initial_yaw);
        sum_cos += std::cos((double)initial_yaw);
    }

    // psi0 是「初始機頭方向」（wrapped）
    psi0 = std::atan2(sum_sin, sum_cos);

    // 三大目標 #2：初始 psi_command 平移到 0 度
    psi_command_initial = 0.0;
    psi_command = psi_command_initial;

    // 三大目標 #3：IMU 量到的 psi 以相對 psi_command_initial 表示（所以一開始 psi=0）
    psi = 0.0;

    
    // 等待第一筆 GPS
    while (node->get_last_gps_timestamp() == 0.0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    last_gps_timestamp_seen = node->get_last_gps_timestamp();
    last_gps_wallclock     = std::chrono::high_resolution_clock::now();

    std::cout << "Init successful. psi0(IMU avg) = " << psi0 * 180 / M_PI
              << " deg, psi_command_initial = 0 deg\n";
    auto writeGroup = [](std::stringstream& buffer, const VariablesGroup& group) {
        for (const auto& name : group.names) buffer << name << " ";
        for (const auto& var : group.group) buffer << var << " ";
        buffer << "\n";
    };

    // === 新增：GPS fallback 狀態 ===
    bool gps_fallback_active = false;

    while (running) {
        // === 固定主迴圈頻率為 100 Hz，並量測迴圈間隔 ===
        static auto last_loop_time = std::chrono::steady_clock::now();
        auto loop_start = std::chrono::steady_clock::now();

        // 計算與上一圈的時間差（秒）
        double loop_interval_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(loop_start - last_loop_time).count();

        // 每 50 圈印一次間隔時間（理論值應該接近 0.01 秒）
        static int loop_counter = 0;
        loop_counter++;
        // if (loop_counter % 50 == 0) {
        //    std::cout << "[Loop Interval] " << std::fixed << std::setprecision(6)
        //              << loop_interval_sec << " sec" << std::endl;
        // }
        last_loop_time = loop_start;

        auto now = std::chrono::high_resolution_clock::now();

        // SUBS相關參數
        static std::string last_mode = "off";  // 記錄上一次狀態，避免重複觸發
        float switching = ch_norm[4].load();
        

        if (switching >= -0.5f && switching <= -0.4f && last_mode != "motor_on") {
            if (!motors_on) {
                motors_on = true;
                last_mode = "motor_on";
                setPWM(file, 1, 0, 600);
                setPWM(file, 2, 0, 600);
                setPWM(file, 3, 0, 600);
                setPWM(file, 4, 0, 600);
                std::cout << "Motors ON.\n";
            }
        }
        else if (switching >= -0.25f && switching <= -0.15f && last_mode != "control_mode") {
            if (motors_on) {
                control_mode_on = true;
                psi_command = psi_command_initial;
                last_mode = "control_mode";
                std::cout << "🟢 Enter control mode" << std::endl;
            } else {
                std::cout << "⚠️ Cannot enter control mode — motors are OFF" << std::endl;
            }
        }
        else if (switching > 0.0f && last_mode != "motor_off") {
            motors_on = false;
            control_mode_on = false;
            last_mode = "motor_off";

            setPWM(file, 1, 0, 400);
            setPWM(file, 2, 0, 400);
            setPWM(file, 3, 0, 400);
            setPWM(file, 4, 0, 400);

            std::cout << "🔴 Motors OFF" << std::endl;
        }

        // IMU 狀態監看
        double current_imu_timestamp = node->get_last_imu_timestamp();
        static auto start_time = program_start;
        if (current_imu_timestamp > last_checked_imu_timestamp) {
            double interval = current_imu_timestamp - last_checked_imu_timestamp;
            last_checked_imu_timestamp = current_imu_timestamp;
            imu_intervals.push_back(interval * 1000.0);
            if (imu_intervals.size() > max_intervals) imu_intervals.pop_front();
            imu_loss_counter = 0; imu_loss_detected = false;
        }
        auto elapsed_since_start = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        if (elapsed_since_start > imu_detection_delay) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - std::chrono::high_resolution_clock::time_point(
                std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                std::chrono::duration<double>(current_imu_timestamp))));
            if (elapsed > imu_timeout) {
                imu_loss_counter++;
                if (imu_loss_counter >= imu_loss_threshold) {
                    std::cerr << "Warning: IMU data lost > " << imu_timeout.count()
                              << "ms, counter=" << imu_loss_counter << "\n";
                    std::cerr << "Recent IMU intervals (ms): ";
                    for (const auto& itv : imu_intervals) std::cerr << itv << " ";
                    std::cerr << "\n";
                    imu_loss_detected = true;
                }
            }
        }

        // ====================== 位置回路（10 Hz, 只 z）======================
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_position_update) >= position_update_interval) {

            // --- 檢查 GPS 是否有新資料，並以「最後看到新時間戳」+ timeout 判定可用性 ---
            bool gps_ok_now = false;
            double gps_ts = node->get_last_gps_timestamp();
            if (gps_ts > last_gps_timestamp_seen) {
                // 收到新 GPS
                last_gps_timestamp_seen = gps_ts;
                last_gps_wallclock = now;

                auto gps_data = node->get_last_gps_data();
                {
                    std::lock_guard<std::mutex> lock(dataMutex);
                    current_x = gps_data.pose.pose.position.x;
                    current_y = gps_data.pose.pose.position.y;
                    current_z = gps_data.pose.pose.position.z;
                }
                gps_ok_now = true;
            } else {
                // 沒有新資料 → 看 timeout
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_gps_wallclock);
                gps_ok_now = (elapsed <= gps_timeout);
            }

            // 若有用到 XY 超界（保護）
            if (gps_ok_now) {
                double x, y;
                { std::lock_guard<std::mutex> lock(dataMutex); x = current_x; y = current_y; }
                double dist_xy = std::hypot(x, y);
                xy_exceeded = (dist_xy > XY_RADIUS_LIMIT);
                z_exceeded = (std::fabs(current_z) > Z_RADIUS_LIMIT);
            }

            // --- 依狀態切換 GPS fallback ---
            if (control_mode_on) {
                if (!gps_ok_now && !gps_fallback_active) {
                    // 進入 fallback
                    gps_fallback_active = true;
                    psi_command = psi;
                    std::cout << "GPS LOST → Fallback: set U1=17.65N, keep level, z_target unchanged.\n";
                } else if (gps_ok_now && gps_fallback_active) {
                    // 離開 fallback，恢復 Z 控制器
                    gps_fallback_active = false;
                    // 以「當前高度」為新起點
                    double cz;
                    { std::lock_guard<std::mutex> lock(dataMutex); cz = current_z; }
                    z_target = cz;
                    z_command = z_target;
                }
            } else {
                // 非控制模式 → 確保 fallback 關閉
                gps_fallback_active = false;
            }

            // --- 只有 GPS 正常且在控制模式才更新 Z 控制器 ---
            if (gps_ok_now && control_mode_on) {
                auto imu_data = node->get_last_imu_data();
                float phi, theta, psi_raw;
                quaternionToEuler(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w, phi, theta, psi_raw);

                // 用 GPS header timestamp 估 dt（與舊版一致）
                auto gps_data = node->get_last_gps_data();
                double current_time = gps_data.header.stamp.sec + gps_data.header.stamp.nanosec * 1e-9;
                double dt_position = (last_position_time == 0.0) ? 0.1 : (current_time - last_position_time);
                last_position_time = current_time;
                if (dt_position <= 0) dt_position = 0.1;
                else if (dt_position > 0.2) dt_position = 0.2;

                x_command = x_target;
                y_command = y_target;
                z_command = z_target;

                // -------- z 命令導數與夾限 --------
                z_command_Derivative.updateDt(dt_position);
                z_command_sec_Derivative.updateDt(dt_position);

                double x_command_Dot = x_command_Derivative.update(x_command);
                double y_command_Dot = y_command_Derivative.update(y_command);
                double z_command_Dot = z_command_Derivative.update(z_command);
                if (x_command_Dot > position_command_bound_derivative) x_command_Dot = position_command_bound_derivative;
                else if (x_command_Dot < -position_command_bound_derivative) x_command_Dot = -position_command_bound_derivative;
                if (y_command_Dot > position_command_bound_derivative) y_command_Dot = position_command_bound_derivative;
                else if (y_command_Dot < -position_command_bound_derivative) y_command_Dot = -position_command_bound_derivative;
                if (z_command_Dot > position_command_bound_derivative) z_command_Dot = position_command_bound_derivative;
                else if (z_command_Dot < -position_command_bound_derivative) z_command_Dot = -position_command_bound_derivative;

                double x_command_sec_Dot = x_command_sec_Derivative.update(x_command_Dot);
                double y_command_sec_Dot = y_command_sec_Derivative.update(y_command_Dot);
                double z_command_sec_Dot = z_command_sec_Derivative.update(z_command_Dot);
                if (x_command_sec_Dot > position_command_bound_sec_derivative) x_command_sec_Dot = position_command_bound_sec_derivative;
                else if (x_command_sec_Dot < -position_command_bound_sec_derivative) x_command_sec_Dot = -position_command_bound_sec_derivative;
                if (y_command_sec_Dot > position_command_bound_sec_derivative) y_command_sec_Dot = position_command_bound_sec_derivative;
                else if (y_command_sec_Dot < -position_command_bound_sec_derivative) y_command_sec_Dot = -position_command_bound_sec_derivative;
                if (z_command_sec_Dot > position_command_bound_sec_derivative) z_command_sec_Dot = position_command_bound_sec_derivative;
                else if (z_command_sec_Dot < -position_command_bound_sec_derivative) z_command_sec_Dot = -position_command_bound_sec_derivative;

                // 目前速度（與你原本重映射一致）
                double current_x_Dot = gps_data.twist.twist.linear.x;
                double current_y_Dot = gps_data.twist.twist.linear.y;
                double current_z_Dot = gps_data.twist.twist.linear.z;
                z_vel_exceeded = (std::fabs(current_z_Dot) > Z_VELOCITY_LIMIT);


                // 誤差與滑模面（僅 z）
                double x_e = current_x - x_command;
                double y_e = current_y - y_command;
                double z_e = current_z - z_command;

                double x_e_Dot = current_x_Dot - x_command_Dot;
                double y_e_Dot = current_y_Dot - y_command_Dot;
                double z_e_Dot = current_z_Dot - z_command_Dot;

                // 滑模面
                double sx = C[0] * x_e + x_e_Dot;
                double sy = C[1] * y_e + y_e_Dot;
                double sz = C[2] * z_e + z_e_Dot;

                double sx_sat = (std::abs(sx) > bound) ? (sx > 0 ? 1 : -1) : sx / bound;
                double sy_sat = (std::abs(sy) > bound) ? (sy > 0 ? 1 : -1) : sy / bound;
                double sz_sat = (std::abs(sz) > bound) ? (sz > 0 ? 1 : -1) : sz / bound;

                // NDOB（用上一次 U1）
                ndob_x(current_x_Dot, phi, theta, psi, dt_position, U1, z_x, d_x, z_x_new, L[0]);
                ndob_y(current_y_Dot, phi, theta, psi, dt_position, U1, z_y, d_y, z_y_new, L[1]);
                ndob_z(current_z_Dot, phi, theta, dt_position, U1, z_z, d_z, z_z_new, L[2]);
                z_x = z_x_new; z_y = z_y_new; z_z = z_z_new;

                // 控制律
                double ux = -C[0] * x_e_Dot + Cv * (current_x_Dot / m) + x_command_sec_Dot - K[0] * sx - Eta[0] * sx_sat - d_x;
                double uy = -C[1] * y_e_Dot + Cv * (current_y_Dot / m) + y_command_sec_Dot - K[1] * sy - Eta[1] * sy_sat - d_y;
                double uz = -C[2] * z_e_Dot + Cv * (current_z_Dot / m) + z_command_sec_Dot - K[2] * sz - Eta[2] * sz_sat + g - d_z;
                
                if (std::abs(uz) < 1e-6) uz = 1e-6;

                // 由 ux/uy/uz 轉姿態命令
                phi_command   = atan2(ux*sin(psi_command) - uy*cos(psi_command), uz);
                theta_command = atan2(ux*cos(psi_command) + uy*sin(psi_command), uz);


                stored_uz = uz;

                // ====== 位置記錄（僅 z） ======
                VariablesGroup g1 = {{"index : "}, {static_cast<double>(index_position)}};
                VariablesGroup g2 = {{"current position : "}, {current_x, current_y, current_z}};
                VariablesGroup g3 = {{"current velocity : "}, {current_x_Dot, current_y_Dot, current_z_Dot}};
                VariablesGroup g4 = {{"position command : "}, {x_command, y_command, z_command}};
                VariablesGroup g5 = {{"command_Dot : "}, {x_command_Dot, y_command_Dot, z_command_Dot}};
                VariablesGroup g6 = {{"command_sec_Dot : "}, {x_command_sec_Dot, y_command_sec_Dot, z_command_sec_Dot}};
                VariablesGroup g7 = {{"error : "}, {x_e, y_e, z_e}};
                VariablesGroup g8 = {{"sliding surface : "}, {sx, sy, sz}};
                VariablesGroup g9 = {{"ux uy uz : "}, {ux, uy, uz}};
                VariablesGroup g10= {{"target : "}, {x_target, y_target, z_target}};
                VariablesGroup g11= {{"attitude cmd(deg) : "}, {phi_command * 180.0 / M_PI, theta_command * 180.0 / M_PI, psi_command * 180.0 / M_PI}};
                VariablesGroup g12= {{"------------------------------------------------------------"}, {}};

                writeGroup(buffer_outFile1, g1);
                writeGroup(buffer_outFile1, g2);
                writeGroup(buffer_outFile1, g3);
                writeGroup(buffer_outFile1, g4);
                writeGroup(buffer_outFile1, g5);
                writeGroup(buffer_outFile1, g6);
                writeGroup(buffer_outFile1, g7);
                writeGroup(buffer_outFile1, g8);
                writeGroup(buffer_outFile1, g9);
                writeGroup(buffer_outFile1, g10);
                writeGroup(buffer_outFile1, g11);
                writeGroup(buffer_outFile1, g12);


                index_position++;
            }

            last_position_update = now;
        }

        // ====================== 姿態回路（100 Hz）======================
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_attitude_update) >= attitude_update_interval) {
            auto imu_data = node->get_last_imu_data();
            auto elapsed_imu = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - std::chrono::high_resolution_clock::time_point(
                    std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                        std::chrono::duration<double>(node->get_last_imu_timestamp()))));

            if (elapsed_imu <= imu_timeout) {
                double current_time = imu_data.header.stamp.sec + imu_data.header.stamp.nanosec * 1e-9;
                double dt_attitude = (last_attitude_time == 0.0) ? 0.01 : (current_time - last_attitude_time);
                last_attitude_time = current_time;
                if (dt_attitude <= 0) dt_attitude = 0.01;
                else if (dt_attitude > 0.05) dt_attitude = 0.05;

                float phi = 0.0f, theta = 0.0f, psi_raw = 0.0f;
                quaternionToEuler(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w, phi, theta, psi_raw);
                roll = phi; pitch = theta;
                // 相對化 + 連續角（存回 psi）
                double psi_wrapped_rel = wrapToPi((double)psi_raw - psi0);
                psi = unwrapAngle(psi, psi_wrapped_rel);

                {
                    std::lock_guard<std::mutex> lock(dataMutex);
                    bool att_invalid = (std::abs(roll) > max_angle || std::abs(pitch) > max_angle);
                    if (att_invalid) {
                        std::cerr << "Warning: Attitude invalid, roll=" << roll * 180.0 / M_PI
                                  << " deg, pitch=" << pitch * 180.0 / M_PI << " deg\n";
                    }
                    attitude_invalid = att_invalid;
                }

                if (control_mode_on) {
                    // 命令導數（姿態）
                    phi_command_Derivative.updateDt(dt_attitude);
                    theta_command_Derivative.updateDt(dt_attitude);
                    psi_command_Derivative.updateDt(dt_attitude);
                    phi_command_sec_Derivative.updateDt(dt_attitude);
                    theta_command_sec_Derivative.updateDt(dt_attitude);
                    psi_command_sec_Derivative.updateDt(dt_attitude);

                    double phi_command_Dot = phi_command_Derivative.update(phi_command);
                    double theta_command_Dot = theta_command_Derivative.update(theta_command);
                    double psi_command_Dot = psi_command_Derivative.update(psi_command);

                    if (phi_command_Dot > attitude_command_bound_derivative) phi_command_Dot = attitude_command_bound_derivative;
                    else if (phi_command_Dot < -attitude_command_bound_derivative) phi_command_Dot = -attitude_command_bound_derivative;
                    if (theta_command_Dot > attitude_command_bound_derivative) theta_command_Dot = attitude_command_bound_derivative;
                    else if (theta_command_Dot < -attitude_command_bound_derivative) theta_command_Dot = -attitude_command_bound_derivative;
                    if (psi_command_Dot > attitude_command_bound_derivative) psi_command_Dot = attitude_command_bound_derivative;
                    else if (psi_command_Dot < -attitude_command_bound_derivative) psi_command_Dot = -attitude_command_bound_derivative;

                    double phi_command_sec_Dot   = phi_command_sec_Derivative.update(phi_command_Dot);
                    double theta_command_sec_Dot = theta_command_sec_Derivative.update(theta_command_Dot);
                    double psi_command_sec_Dot   = psi_command_sec_Derivative.update(psi_command_Dot);

                    if (phi_command_sec_Dot > attitude_command_bound_sec_derivative) phi_command_sec_Dot = attitude_command_bound_sec_derivative;
                    else if (phi_command_sec_Dot < -attitude_command_bound_sec_derivative) phi_command_sec_Dot = -attitude_command_bound_sec_derivative;
                    if (theta_command_sec_Dot > attitude_command_bound_sec_derivative) theta_command_sec_Dot = attitude_command_bound_sec_derivative;
                    else if (theta_command_sec_Dot < -attitude_command_bound_sec_derivative) theta_command_sec_Dot = -attitude_command_bound_sec_derivative;
                    if (psi_command_sec_Dot > attitude_command_bound_sec_derivative) psi_command_sec_Dot = attitude_command_bound_sec_derivative;
                    else if (psi_command_sec_Dot < -attitude_command_bound_sec_derivative) psi_command_sec_Dot = -attitude_command_bound_sec_derivative;

                    // IMU 角速度（與你原本重映射一致）
                    double phi_Dot   = imu_data.angular_velocity.x;
                    double theta_Dot = -imu_data.angular_velocity.y;
                    double psi_Dot   = -imu_data.angular_velocity.z;

                    // 姿態誤差與滑模面
                    double phi_e   = phi - phi_command;
                    double theta_e = theta - theta_command;
                    double psi_e   = psi - psi_command;

                    double phi_e_Dot   = phi_Dot   - phi_command_Dot;
                    double theta_e_Dot = theta_Dot - theta_command_Dot;
                    double psi_e_Dot   = psi_Dot   - psi_command_Dot;

                    double sphi   = C[3] * phi_e   + phi_e_Dot;
                    double stheta = C[4] * theta_e + theta_e_Dot;
                    double spsi   = C[5] * psi_e   + psi_e_Dot;

                    double sphi_sat   = (std::abs(sphi) > bound)   ? (sphi > 0 ? 1 : -1)   : sphi / bound;
                    double stheta_sat = (std::abs(stheta) > bound) ? (stheta > 0 ? 1 : -1) : stheta / bound;
                    double spsi_sat   = (std::abs(spsi) > bound)   ? (spsi > 0 ? 1 : -1)   : spsi / bound;

                    // 姿態控制律
                    double uphi   = ((-(Iyy - Izz) / Ixx) * theta_Dot * psi_Dot - C[3] * phi_e_Dot - K[3] * sphi - Eta[3] * sphi_sat
                                    + (Jtp / Ixx) * theta_Dot * omega + phi_command_sec_Dot) - d_phi;
                    double utheta = ((-(Izz - Ixx) / Iyy) * phi_Dot   * psi_Dot - C[4] * theta_e_Dot - K[4] * stheta - Eta[4] * stheta_sat
                                    - (Jtp / Iyy) * phi_Dot   * omega + theta_command_sec_Dot) - d_theta;
                    double upsi   = ((-(Ixx - Iyy) / Izz) * phi_Dot   * theta_Dot - C[5] * psi_e_Dot - K[5] * spsi - Eta[5] * spsi_sat
                                    + psi_command_sec_Dot) - d_psi;

                    // 推力與力矩
                    double cos_product = cos(phi) * cos(theta);
                    if (std::abs(cos_product) < 1e-6) cos_product = (cos_product >= 0) ? 1e-6 : -1e-6;

                    // === 關鍵：U1 在 GPS fallback 下改為 17.65 N，否則用 z 控制器的 stored_uz ===
                    if (gps_fallback_active) {
                        U1 = WEIGHT_FORCE + ch_norm[2].load() * z_bound;                 // 直接給總推力
                        // 同時維持水平命令
                        theta_command = 0.0;
                        phi_command   = 0.0;
                    } else {
                        U1 = (stored_uz * m) / cos_product + ch_norm[2].load() * z_bound; // 正常情況
                    }

                    U2 = uphi   * Ixx  + ch_norm[1].load() * xy_bound;
                    U3 = utheta * Iyy  + ch_norm[0].load() * xy_bound;
                    U4 = upsi   * Izz;

                    // 馬達混控
                    double vec[4] = {U1, U2, U3, U4};
                    double res[4] = {0.0, 0.0, 0.0, 0.0};
                    double mixM[4][4] = {
                        {0.25,  1.866, -1.866, -14.434},
                        {0.25,  1.866,  1.866,  14.434},
                        {0.25, -1.866,  1.866, -14.434},
                        {0.25, -1.866, -1.866,  14.434}
                    };
                    for (int i = 0; i < 4; i++)
                        for (int j = 0; j < 4; j++)
                            res[i] += mixM[i][j] * vec[j];

                    double force_1 = res[0], force_2 = res[1], force_3 = res[2], force_4 = res[3];

                    // 力限制
                    if (force_1 > max_force) force_1 = max_force; else if (force_1 < min_force) force_1 = min_force;
                    if (force_2 > max_force) force_2 = max_force; else if (force_2 < min_force) force_2 = min_force;
                    if (force_3 > max_force) force_3 = max_force; else if (force_3 < min_force) force_3 = min_force;
                    if (force_4 > max_force) force_4 = max_force; else if (force_4 < min_force) force_4 = min_force;

                    // 力→PWM
                    auto force_to_pwm = [](double f) {
                        return 33.8543 * f + 491.7733;
                    };

                    double rotor_sign_1 = force_to_pwm(force_1);
                    double rotor_sign_2 = force_to_pwm(force_2);
                    double rotor_sign_3 = force_to_pwm(force_3);
                    double rotor_sign_4 = force_to_pwm(force_4);

                    // PWM 限制
                    if (rotor_sign_1 > max_sign) rotor_sign_1 = max_sign; else if (rotor_sign_1 < min_sign) rotor_sign_1 = min_sign;
                    if (rotor_sign_2 > max_sign) rotor_sign_2 = max_sign; else if (rotor_sign_2 < min_sign) rotor_sign_2 = min_sign;
                    if (rotor_sign_3 > max_sign) rotor_sign_3 = max_sign; else if (rotor_sign_3 < min_sign) rotor_sign_3 = min_sign;
                    if (rotor_sign_4 > max_sign) rotor_sign_4 = max_sign; else if (rotor_sign_4 < min_sign) rotor_sign_4 = min_sign;

                    // 估計轉速、合成 omega
                    auto pwm_to_speed = [](double pwm) {
                        return (pwm - 343.464) / 0.3699;
                    };

                    double rotor_speed_1 = pwm_to_speed(rotor_sign_1);
                    double rotor_speed_2 = pwm_to_speed(rotor_sign_2);
                    double rotor_speed_3 = pwm_to_speed(rotor_sign_3);
                    double rotor_speed_4 = pwm_to_speed(rotor_sign_4);

                    omega = -rotor_speed_1 + rotor_speed_2 - rotor_speed_3 + rotor_speed_4;

                    // 更新 NDOB（姿態）
                    ndob_phi(phi_Dot, theta_Dot, psi_Dot, dt_attitude, uphi, omega, z_phi, d_phi, z_phi_new, L[3]);
                    ndob_theta(phi_Dot, theta_Dot, psi_Dot, dt_attitude, utheta, omega, z_theta, d_theta, z_theta_new, L[4]);
                    ndob_psi(phi_Dot, theta_Dot, psi_Dot, dt_attitude, upsi, omega, z_psi, d_psi, z_psi_new, L[5]);
                    z_phi = z_phi_new; z_theta = z_theta_new; z_psi = z_psi_new;

                    // 輸出 PWM
                    int pwm1 = std::round(rotor_sign_1);
                    int pwm2 = std::round(rotor_sign_2);
                    int pwm3 = std::round(rotor_sign_3);
                    int pwm4 = std::round(rotor_sign_4);
                    setPWM(file, 1, 0, pwm1);
                    setPWM(file, 2, 0, pwm2);
                    setPWM(file, 3, 0, pwm3);
                    setPWM(file, 4, 0, pwm4);

                    // ====== 姿態記錄（每 5 次一筆）======
                    static int log_counter = 0;
                    const int log_frequency = 5;
                    log_counter++;
                    if (log_counter >= log_frequency) {
                        log_counter = 0;
                        VariablesGroup a1 = {{"index : "}, {static_cast<double>(index_attitude)}};
                        VariablesGroup a2 = {{"att(deg) :"}, {phi * 180/M_PI, theta * 180/M_PI, psi * 180/M_PI}};
                        VariablesGroup a3 = {{"att_dot : "}, {phi_Dot, theta_Dot, psi_Dot}};
                        VariablesGroup a4 = {{"dist phi theta psi : "}, {d_phi, d_theta, d_psi}};
                        VariablesGroup a5 = {{"err(deg) : "}, {(phi - phi_command) * 180/M_PI, (theta - theta_command)*180/M_PI, (psi - psi_command)*180/M_PI}};
                        VariablesGroup a6 = {{"dt : "}, {dt_attitude}};
                        VariablesGroup a7 = {{"U1 U2 U3 U4 : "}, {U1, U2, U3, U4}};
                        VariablesGroup a8 = {{"cmd_dot : "}, {phi_command_Dot, theta_command_Dot, psi_command_Dot}};
                        VariablesGroup a9 = {{"cmd_ddot : "}, {phi_command_sec_Dot, theta_command_sec_Dot, psi_command_sec_Dot}};
                        VariablesGroup a10 = {{"pwm sign : "}, {pwm1, pwm2, pwm3, pwm4}};
                        VariablesGroup aA = {{"------------------------------------------------------------"}, {}};
                        writeGroup(buffer_outFile2, a1);
                        writeGroup(buffer_outFile2, a2);
                        writeGroup(buffer_outFile2, a3);
                        writeGroup(buffer_outFile2, a4);
                        writeGroup(buffer_outFile2, a5);
                        writeGroup(buffer_outFile2, a6);
                        writeGroup(buffer_outFile2, a7);
                        writeGroup(buffer_outFile2, a8);
                        writeGroup(buffer_outFile2, a9);
                        writeGroup(buffer_outFile2, a10);
                        writeGroup(buffer_outFile2, aA);

                        index_attitude++;
                    }
                }
            }

            last_attitude_update = now;
        }

        // Emergency（IMU/姿態/XY 超界）
        if (motors_on && (imu_loss_detected || attitude_invalid || xy_exceeded || z_exceeded || z_vel_exceeded)) {
            std::cerr << "Emergency: ";
            if (imu_loss_detected)  std::cerr << "[IMU lost] ";
            if (attitude_invalid)   std::cerr << "[Attitude invalid] ";
            if (xy_exceeded)        std::cerr << "[XY drift > " << XY_RADIUS_LIMIT << " m] ";
            if (z_exceeded)         std::cerr << "[Z drift > "  << Z_RADIUS_LIMIT  << " m] ";
            if (z_vel_exceeded)      std::cerr << "[Z velocity > " << Z_VELOCITY_LIMIT << " m/s] ";
            std::cerr << "Shutting down motors.\n";
            setPWM(file, 1, 0, 400);
            setPWM(file, 2, 0, 400);
            setPWM(file, 3, 0, 400);
            setPWM(file, 4, 0, 400);
            motors_on = false;
            control_mode_on = false;
            gps_fallback_active = false;
        }

        buffer_flush_counter++;
        if (buffer_flush_counter >= buffer_flush_frequency) {
            if (buffer_outFile1.tellp() > 0) {
                outFile1 << buffer_outFile1.str(); outFile1.flush();
                buffer_outFile1.str(""); buffer_outFile1.clear();
            }
            if (buffer_outFile2.tellp() > 0) {
                outFile2 << buffer_outFile2.str(); outFile2.flush();
                buffer_outFile2.str(""); buffer_outFile2.clear();
            }
            buffer_flush_counter = 0;
        }

        // === 補足至 10ms 週期（100 Hz）===
        auto loop_end = std::chrono::steady_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_start);
        auto sleep_time = std::chrono::milliseconds(10) - loop_duration; // 在這裡更改值以調整主迴圈頻率
        if (sleep_time > std::chrono::microseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        } else {
            // 若本圈耗時超過10ms，偶爾印出警告
            static int warning_counter = 0;
            warning_counter++;
            if (warning_counter % 50 == 0) {
                std::cerr << "⚠️ Loop overrun! Took "
                          << loop_duration.count() / 1000.0 << " ms\n";
            }
        }
    }

    if (buffer_outFile1.tellp() > 0) { outFile1 << buffer_outFile1.str(); outFile1.flush(); }
    if (buffer_outFile2.tellp() > 0) { outFile2 << buffer_outFile2.str(); outFile2.flush(); }
    outFile1.close(); outFile2.close();
}

void spinThread(std::shared_ptr<MultiTopicSubscriber> node) {
    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signal_handler);

    // I2C init
    const char *filename = "/dev/i2c-7";
    int file = open(filename, O_RDWR);
    if (file < 0) { std::cerr << "Failed to open the i2c bus\n"; return 1; }
    if (ioctl(file, I2C_SLAVE, PCA9685_ADDRESS) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave.\n"; return 1;
    }
    writeRegister(file, MODE1, 0x00);
    setPWMFreq(file, 100);
    setPWM(file, 0, 0, 400);
    setPWM(file, 1, 0, 400);
    setPWM(file, 2, 0, 400);
    setPWM(file, 3, 0, 400);
    setPWM(file, 4, 0, 400);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // log 檔名
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = ss.str();
    std::string filePath1 = "/home/dodo/UAV_log/position_log " + timestamp + ".txt";
    std::ofstream outFile1(filePath1);
    if (!outFile1) { std::cerr << "Unable to open " << filePath1 << "\n"; return 1; }
    std::string filePath2 = "/home/dodo/UAV_log/attitude_log " + timestamp + ".txt";
    std::ofstream outFile2(filePath2);
    if (!outFile2) { std::cerr << "Unable to open " << filePath2 << "\n"; return 1; }

    // ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiTopicSubscriber>();
    std::thread spin_thread(spinThread, node);
    std::thread sbusThreadObj(sbusThread);
    std::thread controlThread(droneControlThread, file, node, std::ref(outFile1), std::ref(outFile2));

    if (spin_thread.joinable()) spin_thread.join();
    if (sbusThreadObj.joinable()) sbusThreadObj.join();
    if (controlThread.joinable()) controlThread.join();    

    setPWM(file, 0, 0, 0);
    setPWM(file, 1, 0, 0);
    setPWM(file, 2, 0, 0);
    setPWM(file, 3, 0, 0);
    setPWM(file, 4, 0, 0);
    usleep(10000);
    std::cout << "stop\n";
    close(file);
    return 0;
}