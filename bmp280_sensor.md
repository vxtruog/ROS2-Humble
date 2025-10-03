# Cấu hình I2C trên Rapsberry Pi 4
# Viết node C++ trên ROS2
- Tạo package
```
ros2 pkg create --build-type ament_cmake bmp280_reader --dependencies rclcpp std_msgs sensor_msgs
```
- Tạo node C++ đọc BMP280
```
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#define BMP280_I2C_ADDR 0x76   // Địa chỉ mặc định, có thể là 0x77
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_TEMP_MSB  0xFA
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CALIB     0x88

class BMP280Node : public rclcpp::Node {
public:
    BMP280Node() : Node("bmp280_node") {
        // Mở I2C device
        fd_ = open("/dev/i2c-1", O_RDWR);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không mở được /dev/i2c-1");
            return;
        }

        if (ioctl(fd_, I2C_SLAVE, BMP280_I2C_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Không kết nối được BMP280");
            return;
        }

        read_calibration_data();

        // Cấu hình chế độ đo: temp và press oversampling x1, mode normal
        write_register(BMP280_REG_CTRL_MEAS, 0x27);
        write_register(BMP280_REG_CONFIG, 0xA0);

        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("bmp280/temperature", 10);
        press_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("bmp280/pressure", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&BMP280Node::read_data, this));
    }

    ~BMP280Node() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr press_pub_;

    // Calibration data
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t t_fine;

    void write_register(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        write(fd_, buf, 2);
    }

    uint8_t read8(uint8_t reg) {
        write(fd_, &reg, 1);
        uint8_t value;
        read(fd_, &value, 1);
        return value;
    }

    uint16_t read16(uint8_t reg) {
        uint8_t buf[2];
        write(fd_, &reg, 1);
        read(fd_, buf, 2);
        return (buf[1] << 8) | buf[0];
    }

    int16_t readS16(uint8_t reg) {
        return (int16_t)read16(reg);
    }

    uint32_t read24(uint8_t reg) {
        uint8_t buf[3];
        write(fd_, &reg, 1);
        read(fd_, buf, 3);
        return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    }

    void read_calibration_data() {
        dig_T1 = read16(0x88);
        dig_T2 = readS16(0x8A);
        dig_T3 = readS16(0x8C);
        dig_P1 = read16(0x8E);
        dig_P2 = readS16(0x90);
        dig_P3 = readS16(0x92);
        dig_P4 = readS16(0x94);
        dig_P5 = readS16(0x96);
        dig_P6 = readS16(0x98);
        dig_P7 = readS16(0x9A);
        dig_P8 = readS16(0x9C);
        dig_P9 = readS16(0x9E);
    }

    float compensate_T(int32_t adc_T) {
        float var1 = (((float)adc_T) / 16384.0 - ((float)dig_T1) / 1024.0) * ((float)dig_T2);
        float var2 = ((((float)adc_T) / 131072.0 - ((float)dig_T1) / 8192.0) *
                      (((float)adc_T) / 131072.0 - ((float)dig_T1) / 8192.0)) * ((float)dig_T3);
        t_fine = (int32_t)(var1 + var2);
        return (var1 + var2) / 5120.0;
    }

    float compensate_P(int32_t adc_P) {
        float var1 = ((float)t_fine / 2.0) - 64000.0;
        float var2 = var1 * var1 * ((float)dig_P6) / 32768.0;
        var2 = var2 + var1 * ((float)dig_P5) * 2.0;
        var2 = (var2 / 4.0) + (((float)dig_P4) * 65536.0);
        var1 = (((float)dig_P3) * var1 * var1 / 524288.0 +
               ((float)dig_P2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * ((float)dig_P1);
        if (var1 == 0.0) return 0; // tránh chia 0
        float p = 1048576.0 - (float)adc_P;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((float)dig_P9) * p * p / 2147483648.0;
        var2 = p * ((float)dig_P8) / 32768.0;
        p = p + (var1 + var2 + ((float)dig_P7)) / 16.0;
        return p;
    }

    void read_data() {
        int32_t adc_T = read24(BMP280_REG_TEMP_MSB) >> 4;
        int32_t adc_P = read24(BMP280_REG_PRESS_MSB) >> 4;

        float temp = compensate_T(adc_T);
        float press = compensate_P(adc_P);

        auto t_msg = sensor_msgs::msg::Temperature();
        t_msg.temperature = temp;
        t_msg.header.stamp = this->now();
        t_msg.header.frame_id = "bmp280";

        auto p_msg = sensor_msgs::msg::FluidPressure();
        p_msg.fluid_pressure = press; // Pa
        p_msg.header = t_msg.header;

        temp_pub_->publish(t_msg);
        press_pub_->publish(p_msg);

        RCLCPP_INFO(this->get_logger(), "T=%.2f °C, P=%.2f Pa", temp, press);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMP280Node>());
    rclcpp::shutdown();
    return 0;
}
```
