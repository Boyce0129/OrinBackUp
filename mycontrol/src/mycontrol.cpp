#include <iostream>
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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <csignal> 
#include <sys/select.h>
#include <fcntl.h>
#include <cmath>
#include <iomanip>
#include <vector>
#include <stdexcept>
#include <utility>
#include "my_msg/msg/bmp280.hpp"
#include <fstream>


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

std::string command = "";
std::mutex commandMutex;
std::atomic<bool> running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        running = false;
        rclcpp::shutdown();  // 关闭 ROS 2
    }
}

class MultiTopicSubscriber : public rclcpp::Node {
public:
    MultiTopicSubscriber()
    : Node("multi_topic_subscriber") {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&MultiTopicSubscriber::imuCallback, this, std::placeholders::_1));
    

    }


    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        new_imu_data_available_ = true;
        last_imu_data_ = *msg;
    }

    sensor_msgs::msg::Imu get_last_imu_data() {
        std::lock_guard<std::mutex> lock(mutex_);
        new_imu_data_available_ = false;
        return last_imu_data_;
    }

    bool new_imu_data_available() const {
        return new_imu_data_available_;
    }


    private:

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    mutable std::mutex mutex_;
    sensor_msgs::msg::Imu last_imu_data_;
    bool new_imu_data_available_ = false;

};


class FOHPDerivative {
private:
    double dt;
    double alpha;
    std::vector<double> y;

public:
    FOHPDerivative(double alpha) : alpha(alpha) {
        y = std::vector<double>(3, 0.0); // 初始化为三个零
    }

    void updateDt(double newdt) {
        dt = newdt;
    }

    double compute(double new_input) {
        // 更新历史数据
        y[0] = y[1];
        y[1] = y[2];
        y[2] = new_input;

        // 计算微分
        return (1 - alpha) * (y[2] - y[1]) / dt + alpha * (y[2] - y[0]) / (2 * dt);
    }
};


class FusionKalmanFilter {
public:
    double estimate;
    double cov_estimate;

    FusionKalmanFilter(double init_estimate, double init_cov)
        : estimate(init_estimate), cov_estimate(init_cov) {}

    void update(double gps_estimate, double gps_cov, double baro_estimate, double baro_cov) {
        // First update with GPS data
        double K_gps = cov_estimate / (cov_estimate + gps_cov);
        estimate = estimate + K_gps * (gps_estimate - estimate);
        cov_estimate = (1 - K_gps) * cov_estimate;

        // Second update with Barometer data
        double K_baro = cov_estimate / (cov_estimate + baro_cov);
        estimate = estimate + K_baro * (baro_estimate - estimate);
        cov_estimate = (1 - K_baro) * cov_estimate;
    }
};

void writeRegister(int file, int registerAddress, int value) {
    char buffer[2];
    buffer[0] = registerAddress;
    buffer[1] = value;
    if (write(file, buffer, 2) != 2) {
        std::cerr << "Failed to write to the i2c bus.\n";
    }
}

// Function to set PWM frequency
void setPWMFreq(int file, int freq) {
    float prescaleval = 25000000.0; // 25MHz
    prescaleval /= 4096.0;          // 12-bit
    prescaleval /= float(freq);
    prescaleval -= 1.0;
    int prescale = int(prescaleval + 0.5);

    int oldmode = i2c_smbus_read_byte_data(file, MODE1);
    int newmode = (oldmode & 0x7F) | 0x10; // sleep
    writeRegister(file, MODE1, newmode);    // go to sleep
    writeRegister(file, PRESCALE, prescale);
    writeRegister(file, MODE1, oldmode);
    usleep(5000);
    writeRegister(file, MODE1, oldmode | 0x80);
}

// Function to set PWM output
void setPWM(int file, int channel, int on, int off) {
    writeRegister(file, LED0_ON_L + 4 * channel, on & 0xFF);
    writeRegister(file, LED0_ON_H + 4 * channel, on >> 8);
    writeRegister(file, LED0_OFF_L + 4 * channel, off & 0xFF);
    writeRegister(file, LED0_OFF_H + 4 * channel, off >> 8);
}

void quaternionToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw) {
    roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    pitch = (-1) *(std::asin(2.0 * (qw * qy - qz * qx)));
    yaw = (-1) *std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

// void quaternionToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw) {
//      roll = std::asin(2.0 * (qw * qy - qz * qx));
//      pitch = (-1) * std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
//      yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
// }

// void quaternionToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw) {
//     // 計算原始歐拉角
//     roll = std::asin(2.0 * (qw * qy - qz * qx));
//     pitch = (-1) * std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
//     yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

//     // 補償 X 方向 (Roll) 1.4507 度的安裝誤差
//     const float roll_error_degrees = 1.07f; // X 方向誤差 (度)
//     const float roll_error_radians = roll_error_degrees * M_PI / 180.0f; // 轉換為弧度
//     roll -= roll_error_radians;

//     // 補償 Y 方向 (Pitch) -0.2764 度的安裝誤差
//     const float pitch_error_degrees = -0.07f; // Y 方向誤差 (度)
//     const float pitch_error_radians = pitch_error_degrees * M_PI / 180.0f; // 轉換為弧度
//     pitch -= pitch_error_radians; // 注意這裡使用 -=，因為 pitch 已取反

//     // 確保 roll、pitch 和 yaw 在 [-π, π] 範圍內
//     if (roll > M_PI) {
//         roll -= 2.0f * M_PI;
//     } else if (roll < -M_PI) {
//         roll += 2.0f * M_PI;
//     }
//     if (pitch > M_PI) {
//         pitch -= 2.0f * M_PI;
//     } else if (pitch < -M_PI) {
//         pitch += 2.0f * M_PI;
//     }
//     if (yaw > M_PI) {
//         yaw -= 2.0f * M_PI;
//     } else if (yaw < -M_PI) {
//         yaw += 2.0f * M_PI;
//     }
// }
const double EARTH_RADIUS = 6371000.0;// 地球半径，单位：米

double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}// 将度转换为弧度


void calculateDistance(double lat1, double lon1, double alt1, 
                       double lat2, double lon2, double alt2,
                       double& distanceEast, double& distanceNorth, double& altitudeDifference) {
    double lat1Rad = toRadians(lat1);
    double lat2Rad = toRadians(lat2);
    double deltaLat = toRadians(lat2 - lat1);
    double deltaLon = toRadians(lon2 - lon1);

    // 计算北向和东向距离
    distanceNorth = deltaLat * EARTH_RADIUS;
    distanceEast = deltaLon * EARTH_RADIUS * cos((lat1Rad + lat2Rad) / 2);

    // 计算高度差
    altitudeDifference = alt2 - alt1;
}

typedef std::pair<double, double> DataPoint;
double linearInterpolate(double x, const DataPoint& p1, const DataPoint& p2) {
    double slope = (p2.second - p1.second) / (p2.first - p1.first);
    return p1.second + slope * (x - p1.first);
} 


double interpolate(double x, const std::vector<DataPoint>& dataPoints) {
    for (size_t i = 0; i < dataPoints.size() - 1; ++i) {
        if (x >= dataPoints[i].first && x <= dataPoints[i + 1].first) {
            return linearInterpolate(x, dataPoints[i], dataPoints[i + 1]);
        }
    }
    // 如果 x 超出了数据点的范围，返回一个错误值
    return -1; // 或者可以抛出一个异常
}

struct VariablesGroup {
    std::vector<double> group; // 使用vector来存储每组的变量
};


void ndob_phi(double phi_Dot,double theta_Dot,double psi_Dot,double dt,double u_phi,double omega,double z_phi_old,double& d_phi,double& z_phi)
{
    const double Ixx = 0.01885; // kg*m^2
    const double Iyy = 0.01837; // kg*m^2
    const double Izz = 0.01774; // kg*m^2
    const double m = 1.995;     // kg
    const double g = 9.81;      // m/s^2
    const double Jtp = 4.103e-4;
    const double l_phi = 6;
    double p_phi=l_phi*phi_Dot;
    double z_phi_Dot=(l_phi*(((-(Iyy-Izz)*theta_Dot*psi_Dot)/Ixx)+((Jtp/Ixx)*theta_Dot*omega)-(u_phi)))-(l_phi*z_phi_old)-(l_phi*p_phi);
    z_phi=z_phi_Dot*dt;
    d_phi=z_phi+p_phi;

}


void ndob_theta(double phi_Dot,double theta_Dot,double psi_Dot,double dt,double u_theta,double omega,double z_theta_old,double& d_theta,double& z_theta)
{
    const double Ixx = 0.01885; // kg*m^2
    const double Iyy = 0.01837; // kg*m^2
    const double Izz = 0.01774; // kg*m^2
    const double m = 1.995;     // kg
    const double g = 9.81;      // m/s^2
    const double Jtp = 4.103e-4;
    const double l_theta = 6;
    double p_theta=l_theta*theta_Dot;
    double z_theta_Dot=(l_theta*(((-(Izz-Ixx)*phi_Dot*psi_Dot)/Iyy)-((Jtp/Iyy)*phi_Dot*omega)-(u_theta)))-(l_theta*z_theta_old)-(l_theta*p_theta);
    z_theta=z_theta_Dot*dt;
    d_theta=z_theta+p_theta;

}

void ndob_psi(double phi_Dot,double theta_Dot,double psi_Dot,double dt,double u_psi,double omega,double z_psi_old,double& d_psi,double& z_psi)
{
    const double Ixx = 0.01885; // kg*m^2
    const double Iyy = 0.01837; // kg*m^2
    const double Izz = 0.01774; // kg*m^2
    const double m = 1.995;     // kg
    const double g = 9.81;      // m/s^2
    const double Jtp = 4.103e-4;
    const double l_psi = 3;
    double p_psi=l_psi*psi_Dot;
    double z_psi_Dot=(l_psi*(((-(Ixx-Iyy)*theta_Dot*phi_Dot)/Izz)-(u_psi)))-(l_psi*z_psi_old)-(l_psi*p_psi);
    z_psi=z_psi_Dot*dt;
    d_psi=z_psi+p_psi;

}

void commandInputThread() {
    fd_set readfds;
    struct timeval tv;
    int retval;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100 毫秒

    while (running){
        FD_SET(STDIN_FILENO, &readfds);

        retval = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);

        if (retval == -1) {
            perror("select()");
            break;
        } else if (retval) {
            std::string newCommand;
            std::cin >> newCommand;
            {
                std::lock_guard<std::mutex> lock(commandMutex);
                command = newCommand;
            }
        } else {
           
        }

        if (!running) {
            
            break;
        }
    }
}



void droneControlThread(int file,const std::shared_ptr<MultiTopicSubscriber>& node,std::ofstream& outFile1,std::ofstream& outFile2) {
   
    FOHPDerivative theta_command_Derivative(0.5); // 假设 alpha = 0.5
    FOHPDerivative phi_command_Derivative(0.5);
    FOHPDerivative psi_command_Derivative(0.5);
    FOHPDerivative theta_command_sec_Derivative(0.5);
    FOHPDerivative phi_command_sec_Derivative(0.5);
    FOHPDerivative psi_command_sec_Derivative(0.5);
    FOHPDerivative x_command_Derivative(0.5);
    FOHPDerivative y_command_Derivative(0.5);
    FOHPDerivative z_command_Derivative(0.5);
    FOHPDerivative x_command_sec_Derivative(0.5);
    FOHPDerivative y_command_sec_Derivative(0.5);
    FOHPDerivative z_command_sec_Derivative(0.5);
    // FOHPDerivative theta_Derivative(0.5);
    // FOHPDerivative phi_Derivative(0.5);
    // FOHPDerivative psi_Derivative(0.5);

    FOHPDerivative x_Derivative(0.65);
    FOHPDerivative y_Derivative(0.65);
    FOHPDerivative z_Derivative(0.65);


    double psi_command_initial;
    double theta_command=0;
    double phi_command=0;
    double psi_command=0;
    double x_command=0;
    double y_command=0;
    double z_command=0;


    const double Ixx = 0.01885; // kg*m^2
    const double Iyy = 0.01837; // kg*m^2
    const double Izz = 0.01774; // kg*m^2
    const double m = 1.995;     // kg
    const double g = 9.81;      // m/s^2
    const double Jtp = 4.103e-4;
    const double l = 0.19;
    const double Cv=0.01;
    const double bound=0.1;
    const double position_command_bound_derivative=0.05;
    const double position_command_bound_sec_derivative=0.5;
    
    const double attitude_command_bound=15;
    const double attitude_command_bound_derivative=0.5;
    const double attitude_command_bound_sec_derivative=5;


    const double position_bound_derivative=0.5;


    const double CEP=0.8;

    double position_dt=0.07;
    double attuide_dt=0.0105;

    double U1=0;
    double U2=0;
    double U3=0;
    double U4=0;
    double omega=0;

    double ux=0;
    double uy=0;
    double uz=0;


    double d_phi=0;
    double z_phi=0;
    double z_phi_new=0;
    double d_theta=0;
    double z_theta=0;
    double z_theta_new=0;
    double d_psi=0;
    double z_psi=0;
    double z_psi_new=0;


    std::vector<double> C = {0.5,0.5,1,10,10,5};
    std::vector<double> K = {1,1,1,35,35,2};
    std::vector<double> Eta = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};



    std::vector<DataPoint> rotorspeed_change = {
    {600,630.38},{610,660.15},{620,694.08},{630,725.81},{640,756.50},{650,801.00},{660,817.76},{670,846.87},{680,876.82},
    {690,919.96},{700,932.42},{710,959.97},{720,984.68},{730,1012.12},{740,1038.51},{750,1068.35},{760,1082.91},
    {770,1105.53},{780,1127.83},{790,1167.52},{800,1194.96},{810,1219.04},{820,1241.77}

    };

    std::vector<DataPoint> rotorforce_change = {
    {2.410,600}, {2.626,610}, {2.940,620}, {3.234,630}, {3.498,640}, {3.792,650}, {4.194,660}, {4.508,670},
    {4.998,680}, {5.292,690}, {5.605,700}, {5.997,710}, {6.321,720}, {6.713,730}, {6.997,740}, {7.487,750}, 
    {7.869,760}, {8.143,770}, {8.447,780}, {8.712,790}, {8.957,800}, {9.359,810}, {9.751,820}
    };

    bool flag=true;

    double max_force=9;
    double min_force=2.5;
    double max_sign=800;
    double min_sign=600;
    int i=0;


    int count =0;
    int index2=1;
    int index1=1;
    double initial_x_gps=0;
    double initial_y_gps=0;
    double initial_z_gps=0;

    double initial_cov = 1.0;     // 初始協方差
    double gps_cov = 5.0;         // GPS 的測量協方差
    double barometer_cov = 1.5;   // 氣壓計的測量協方差
    FusionKalmanFilter kf_fusion(0, 1.0);


    float initial_roll;float initial_pitch; float initial_yaw;
    for(int i=0;i<100;i++)
    { 
    while (!node->new_imu_data_available()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto initial_imu_data = node->get_last_imu_data(); 
    float initial_qx = initial_imu_data.orientation.x;
    float initial_qy = initial_imu_data.orientation.y;
    float initial_qz = initial_imu_data.orientation.z;
    float initial_qw = initial_imu_data.orientation.w;
    quaternionToEuler(initial_qx, initial_qy, initial_qz, initial_qw, initial_roll, initial_pitch, initial_yaw);
    psi_command+=initial_yaw;
    }
    psi_command_initial=psi_command/100;

    std::cout << "Init successful"  << std::endl;

        while (running) {
        std::string currentCommand;
        {
            std::lock_guard<std::mutex> lock(commandMutex);
            currentCommand = command;
        }
                    if (currentCommand == "v1") {
                std::cout << "v1go "  << std::endl;
                sleep(1);

                if (node->new_imu_data_available()) {
                    auto imu_data = node->get_last_imu_data();

                std::cout << "IMU Data: " << std::endl;
                std::cout << "Orientation - x: " << imu_data.orientation.x << ", y: " << imu_data.orientation.y
                          << ", z: " << imu_data.orientation.z << ", w: " << imu_data.orientation.w << std::endl;
                std::cout << "Angular Velocity - x: " << imu_data.angular_velocity.x << ", y: " << imu_data.angular_velocity.y
                          << ", z: " << imu_data.angular_velocity.z << std::endl;
                std::cout << "Linear Acceleration - x: " << imu_data.linear_acceleration.x << ", y: " << imu_data.linear_acceleration.y
                          << ", z: " << imu_data.linear_acceleration.z << std::endl;
                float qx = imu_data.orientation.x;
                float qy = imu_data.orientation.y;
                float qz = imu_data.orientation.z;
                float qw = imu_data.orientation.w;
                float roll;float pitch; float yaw;

                quaternionToEuler(qx, qy, qz, qw, roll, pitch, yaw);

                std::cout << "Roll: " << roll*180/3.14 << ", Pitch: " << pitch*180/3.14 << ", Yaw: " << yaw*180/3.14<< std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

            }


            
            
            else if (currentCommand == "v4") {
                
                

                std::lock_guard<std::mutex> lock(commandMutex);

          
                if (node->new_imu_data_available()) {

                auto start_time = std::chrono::steady_clock::now(); // 開始時間
                i++;
                auto imu_data = node->get_last_imu_data();
                float qx = imu_data.orientation.x;
                float qy = imu_data.orientation.y;
                float qz = imu_data.orientation.z;
                float qw = imu_data.orientation.w;
                float roll;float pitch; float yaw;

                quaternionToEuler(qx, qy, qz, qw, roll, pitch, yaw);
                double phi=roll;
                double theta=pitch;
                double psi=yaw;

                theta_command=0.0;
                phi_command=0.0;
                psi_command=psi_command_initial;
                U1=23;
                // std::cout << "v4go  "  << std::endl;

                phi_command_Derivative.updateDt(attuide_dt);
                theta_command_Derivative.updateDt(attuide_dt);
                psi_command_Derivative.updateDt(attuide_dt);
                phi_command_sec_Derivative.updateDt(attuide_dt);
                theta_command_sec_Derivative.updateDt(attuide_dt);
                psi_command_sec_Derivative.updateDt(attuide_dt);
                


                double phi_command_Dot = phi_command_Derivative.compute(phi_command);
                double theta_command_Dot = theta_command_Derivative.compute(theta_command);
                double psi_command_Dot = psi_command_Derivative.compute(psi_command);
                
                if (phi_command_Dot> attitude_command_bound_derivative) {
                        phi_command_Dot = attitude_command_bound_derivative;
                } else if ( phi_command_Dot < -attitude_command_bound_derivative) {
                        phi_command_Dot = -attitude_command_bound_derivative;
                    }
                
                if (theta_command_Dot> attitude_command_bound_derivative) {
                        theta_command_Dot = attitude_command_bound_derivative;
                } else if ( theta_command_Dot < -attitude_command_bound_derivative) {
                       theta_command_Dot = -attitude_command_bound_derivative;
                    }
                
                if ( psi_command_Dot> attitude_command_bound_derivative) {
                        psi_command_Dot = attitude_command_bound_derivative;
                } else if ( psi_command_Dot < -attitude_command_bound_derivative) {
                        psi_command_Dot = -attitude_command_bound_derivative;
                    }

                double phi_command_sec_Dot = phi_command_sec_Derivative.compute(phi_command_Dot);
                double theta_command_sec_Dot = theta_command_sec_Derivative.compute(theta_command_Dot);
                double psi_command_sec_Dot = psi_command_sec_Derivative.compute(psi_command_Dot);

                 if (phi_command_sec_Dot> attitude_command_bound_sec_derivative) {
                        phi_command_sec_Dot = attitude_command_bound_sec_derivative;
                } else if ( phi_command_sec_Dot < -attitude_command_bound_sec_derivative) {
                        phi_command_sec_Dot = -attitude_command_bound_sec_derivative;
                    }
                
                if (theta_command_sec_Dot> attitude_command_bound_sec_derivative) {
                        theta_command_sec_Dot = attitude_command_bound_sec_derivative;
                } else if ( theta_command_sec_Dot < -attitude_command_bound_sec_derivative) {
                       theta_command_sec_Dot = -attitude_command_bound_sec_derivative;
                    }
                
                if ( psi_command_sec_Dot> attitude_command_bound_sec_derivative) {
                        psi_command_sec_Dot = attitude_command_bound_sec_derivative;
                } else if ( psi_command_sec_Dot< -attitude_command_bound_sec_derivative) {
                        psi_command_sec_Dot = -attitude_command_bound_sec_derivative;
                    }




                double phi_Dot=imu_data.angular_velocity.x;
                double theta_Dot=-imu_data.angular_velocity.y;
                double psi_Dot=-imu_data.angular_velocity.z;



                double phi_e=phi-phi_command;
                double theta_e=theta-theta_command;
                double psi_e=psi-psi_command;
                


                double phi_e_Dot=phi_Dot-phi_command_Dot;
                double theta_e_Dot=theta_Dot-theta_command_Dot;
                double psi_e_Dot=psi_Dot-psi_command_Dot;

                double sphi=C[3]*phi_e+phi_e_Dot;
                double stheta=C[4]*theta_e+theta_e_Dot;
                double spsi=C[5]*psi_e+psi_e_Dot;
                
                double sphi_sat;
                if (std::abs(sphi) > bound) {
                    sphi_sat = (sphi > 0) ? 1 : -1;
                } else {
                    sphi_sat = sphi / bound;
                }
                 
                double stheta_sat;
                if (std::abs(stheta) > bound) {
                    stheta_sat = (stheta > 0) ? 1 : -1;
                } else {
                    stheta_sat = stheta / bound;
                }
                
                double spsi_sat;
                if (std::abs(spsi) > bound) {
                    spsi_sat = (spsi > 0) ? 1 : -1;
                } else {
                    spsi_sat = spsi / bound;
                }

                //d_phi=0;d_theta=0;d_psi=0;

                double uphi=((-(Iyy-Izz)/Ixx)*theta_Dot*psi_Dot-C[3]*phi_e_Dot-K[3]*sphi-Eta[3]*sphi_sat+((Jtp/Ixx)*theta_Dot*omega)+phi_command_sec_Dot)-d_phi;
                double utheta=((-(Izz-Ixx)/Iyy)*phi_Dot*psi_Dot-C[4]*theta_e_Dot-K[4]*stheta-Eta[4]*stheta_sat-((Jtp/Iyy)*phi_Dot*omega)+theta_command_sec_Dot)-d_theta;
                double upsi=((-(Ixx-Iyy)/Izz)*phi_Dot*theta_Dot-C[5]*psi_e_Dot-K[5]*spsi-Eta[5]*spsi_sat+psi_command_sec_Dot)-d_psi;
                double U2=uphi*Ixx;
                double U3=utheta*Iyy;
                double U4=upsi*Izz;
                
                // double uphi=((-(Iyy-Izz)/Ixx)*theta_Dot*psi_Dot-C[3]*phi_e_Dot-K[3]*sphi-Eta[3]*sphi_sat+((Jtp/Ixx)*theta_Dot*omega)+phi_command_sec_Dot);
                // double utheta=((-(Izz-Ixx)/Iyy)*phi_Dot*psi_Dot-C[4]*theta_e_Dot-K[4]*stheta-Eta[4]*stheta_sat-((Jtp/Iyy)*phi_Dot*omega)+theta_command_sec_Dot);
                // double upsi=((-(Ixx-Iyy)/Izz)*phi_Dot*theta_Dot-C[5]*psi_e_Dot-K[5]*spsi-Eta[5]*spsi_sat+psi_command_sec_Dot);
                // double U2=uphi*Ixx;
                // double U3=utheta*Iyy;
                // double U4=upsi*Izz;
                
                
    
               

                // std::cout << "phi is " <<phi<< std::endl;
                // std::cout << "theta is " <<theta<< std::endl;
                // std::cout << "psi is " <<psi<< std::endl;
                // std::cout << "sphi is " <<sphi<< std::endl;
                // std::cout << "stheta is " <<stheta<< std::endl;
                // std::cout << "spsi is " <<spsi<< std::endl;
                // std::cout << "phi_Dot is " <<phi_Dot<< std::endl;
                // std::cout << "theta_Dot is " <<theta_Dot<< std::endl;
                // std::cout << "psi_Dot is " <<psi_Dot<< std::endl;
                // std::cout << "phi_e_Dot is " <<phi_e_Dot<< std::endl;
                // std::cout << "theta_e_Dot is " <<theta_e_Dot<< std::endl;
                // std::cout << "psi_e_Dot is " <<psi_e_Dot<< std::endl;
                // std::cout << "phi_command_sec_Dot is " <<phi_command_sec_Dot<< std::endl;
                // std::cout << "theta_command_sec_Dot is " <<theta_command_sec_Dot<< std::endl;
                // std::cout << "psi_command_sec_Dot is " <<psi_command_sec_Dot<< std::endl;
                // std::cout << "U2 is" <<U2<< std::endl;
                // std::cout << "U3 is" <<U3<< std::endl;
                // std::cout << "U4 is" <<U4<< std::endl;


                




                double vector[4] = {U1, U2, U3, U4};
                double result[4] = {0.0, 0.0, 0.0, 0.0};

                double matrix[4][4] = {
                {0.25, 1.866, -1.866, -14.286},
                {0.25, 1.866, 1.866, 14.286},
                {0.25, -1.866, 1.866, -14.286},
                {0.25, -1.866, -1.866, 14.286}
                };

                for(int i = 0; i < 4; i++) {
                    for(int j = 0; j < 4; j++) {
                        result[i] += matrix[i][j] * vector[j];
                            }
                        }
                double force_1=result[0];
                double force_2=result[1];
                double force_3=result[2];
                double force_4=result[3];
                
                if ( force_1> max_force) {
                   force_1 =  max_force;
                } else if (force_1<min_force) {
                    force_1=min_force;
                    }
                 if ( force_2> max_force) {
                   force_2 = max_force;
                } else if (force_2< min_force) {
                    force_2= min_force;
                    }
                 if ( force_3>  max_force) {
                   force_3 =  max_force;
                } else if (force_3<min_force) {
                    force_3= min_force;
                    }
                  if ( force_4>  max_force) {
                   force_4 =  max_force;
                } else if (force_4< min_force) {
                    force_4= min_force;
                    }
                









                double rotor_sign_1=interpolate(force_1, rotorforce_change);
                double rotor_sign_2=interpolate(force_2, rotorforce_change);
                double rotor_sign_3=interpolate(force_3, rotorforce_change);
                double rotor_sign_4=interpolate(force_4, rotorforce_change);


                

                if ( rotor_sign_1> max_sign) {
                   rotor_sign_1 = max_sign;
                } else if ( rotor_sign_1< min_sign) {
                    rotor_sign_1= min_sign;
                    }
                
                if (rotor_sign_2> max_sign) {
                    rotor_sign_2= max_sign;
                } else if ( rotor_sign_2< min_sign) {
                    rotor_sign_2 = min_sign;
                    }

                 if ( rotor_sign_3 >max_sign) {
                    rotor_sign_3= max_sign;
                } else if ( rotor_sign_3< min_sign) {
                   rotor_sign_3= min_sign;
                    }

                if (rotor_sign_4> max_sign) {
                    rotor_sign_4 = max_sign;
                } else if (rotor_sign_4< min_sign) {
                   rotor_sign_4 = min_sign;
                    }





                double rotor_speed_1=interpolate(rotor_sign_1,rotorspeed_change);
                double rotor_speed_2=interpolate(rotor_sign_2,rotorspeed_change);
                double rotor_speed_3=interpolate(rotor_sign_3,rotorspeed_change);
                double rotor_speed_4=interpolate(rotor_sign_4,rotorspeed_change);




                omega=-rotor_speed_1+rotor_speed_2-rotor_speed_3+rotor_speed_4;




                int pwm_sign_motor1=std::round(rotor_sign_1);
                int pwm_sign_motor2=std::round(rotor_sign_2);
                int pwm_sign_motor3=std::round(rotor_sign_3);
                int pwm_sign_motor4=std::round(rotor_sign_4);

                
                ndob_phi(phi_Dot,theta_Dot,psi_Dot,attuide_dt,uphi,omega,z_phi,d_phi,z_phi_new);
                ndob_theta(phi_Dot,theta_Dot,psi_Dot,attuide_dt,utheta,omega,z_theta,d_theta,z_theta_new);
                ndob_psi(phi_Dot,theta_Dot,psi_Dot,attuide_dt,upsi,omega,z_psi,d_psi,z_psi_new);
                z_phi=z_phi_new;
                z_theta=z_theta_new;
                z_psi=z_psi_new;

                //Eta = {0.1, 0.1, 0.1, (0.1+std::abs(d_phi)), (0.1+std::abs(d_theta)), (0.1+std::abs(d_psi))};
                // std::cout << "z_phi is " <<z_phi<< std::endl;
                // std::cout << "d_phi" <<d_phi<< std::endl;



                
                setPWM(file, 1, 0, pwm_sign_motor1); 
                setPWM(file, 2, 0, pwm_sign_motor2); 
                setPWM(file, 3, 0, pwm_sign_motor3); 
                setPWM(file, 4, 0, pwm_sign_motor4);
            

                auto end_time = std::chrono::steady_clock::now(); // 結束時間

                double response_time = std::chrono::duration_cast<std::chrono::microseconds>(
                                    end_time - start_time).count() / 1000.0; // 單位：毫秒
                std::cout << "Response time: " << response_time << " ms" << std::endl;


                VariablesGroup group1 = {{sphi,stheta,sphi}};
                VariablesGroup group2 = {{d_phi,d_theta,d_psi}};
                VariablesGroup group3 = {{sphi,index1}};
                VariablesGroup group4 = {{phi_Dot,theta_Dot,psi_Dot}};
                VariablesGroup group12 = {{phi_e*180/M_PI}};
                VariablesGroup group13 = {{theta_e*180/M_PI}};
                VariablesGroup group14 = {{psi_e*180/M_PI}};
                VariablesGroup group15 = {{pwm_sign_motor1,pwm_sign_motor2,pwm_sign_motor3,pwm_sign_motor4}};
                VariablesGroup group16 = {{phi*180/M_PI,theta*180/M_PI,psi*180/M_PI}};
                VariablesGroup group17 = {{U1,U2,U3,U4}};
                


                for (const auto& var : group1.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;

                for (const auto& var : group2.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;
                
  
                for (const auto& var : group3.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;
                

                for (const auto& var : group4.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;

                for (const auto& var : group12.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;

                for (const auto& var : group13.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;

      
                for (const auto& var : group14.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;


                for (const auto& var : group15.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;



                for (const auto& var : group16.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;

                for (const auto& var : group17.group) {
                    outFile2 << var << " ";
                }
                outFile2 << std::endl;


                index1++;		
		    }
            }
        

            else if (currentCommand == "v6") {
                std::cout << "v6go  "  << std::endl;
                std::lock_guard<std::mutex> lock(commandMutex);
                sleep(1);

                setPWM(file, 0, 0, 600); 
                setPWM(file, 1, 0, 600); 
                setPWM(file, 2, 0, 600); 
                setPWM(file, 3, 0, 600);
                setPWM(file, 4, 0, 600); 
                
 

               
            }

                else if (currentCommand == "v7") {
    
                std::lock_guard<std::mutex> lock(commandMutex);
                sleep(1);
                setPWM(file, 0, 0, 400); 
                setPWM(file, 1, 0, 400); 
                setPWM(file, 2, 0, 400); 
                setPWM(file, 3, 0, 400);
                setPWM(file, 4, 0, 400); 
                std::cout << " " <<i<< std::endl; 
               
            }


            else {
                std::cout << "not define command  " <<currentCommand << std::endl;
                sleep(1); 
               
            }
            if(!running)
            {
                break;
            }
        





        }
}

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signal_handler);
    const char *filename = "/dev/i2c-7";
    int file = open(filename, O_RDWR);
    if (file < 0) {
        std::cerr << "Failed to open the i2c bus\n";
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, PCA9685_ADDRESS) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave.\n";
        return 1;
    }

    // Initialize the PCA9685
    writeRegister(file, MODE1, 0x00); // Normal mode
    setPWMFreq(file, 100);             // Set frequency to 50 Hz
    setPWM(file, 0, 0, 400); 
    setPWM(file, 1, 0, 400); 
    setPWM(file, 2, 0, 400); 
    setPWM(file, 3, 0, 400);
    setPWM(file, 4, 0, 400);  
    std::cout << "gogogogogo "  << std::endl;
    sleep(3);
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = ss.str();

    // Construct file paths
    std::string filePath1 = "/home/dodo/position_log_" + timestamp + ".txt";
    std::string filePath2 = "/home/dodo/attitude_log_" + timestamp + ".txt";

    // std::string filePath1= "/home/dodo/position_log.txt";
    // std::string filePath2= "/home/dodo/attitude_log.txt";

    std::ofstream outFile1(filePath1);
    std::ofstream outFile2(filePath2);
        
    if (!outFile1) {
    std::cerr << "Unable to open file at " << filePath1 << std::endl;
    return 1;
    }
    if (!outFile2) {
    std::cerr << "Unable to open file at " << filePath2 << std::endl;
    return 1;
    }





    rclcpp::init(argc, argv);
    auto node=std::make_shared<MultiTopicSubscriber>();
    std::thread inputThread(commandInputThread);
    std::thread controlThread(droneControlThread,file,node,std::ref(outFile1),std::ref(outFile2));
    while (rclcpp::ok()) {
    rclcpp::spin_some(node);
}
    inputThread.join();
    controlThread.join();
    setPWM(file, 0, 0, 0); 
    setPWM(file, 1, 0, 0); 
    setPWM(file, 2, 0, 0); 
    setPWM(file, 3, 0, 0);
    setPWM(file, 4, 0, 0); 
    usleep(10000);
    std::cout << "stop "  << std::endl;
    close(file);
    outFile1.close();
    outFile2.close();

    return 0;
}