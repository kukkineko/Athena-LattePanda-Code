#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>


class CPPServer : public rclcpp::Node {
public:
    CPPServer() : Node("cpp_server"), io_service_(), socket_(io_service_) {
        // Initialize publishers
        laser_scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_frame", 10);
        imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu_frame", 10);

        // Setup communication with Python client
        setup_communication();
    }

    void setup_communication() {
        boost::asio::ip::tcp::acceptor acceptor(io_service_,
            boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 12345));
        acceptor.accept(socket_);

        // Start async reading or synchronous reading based on your design
        // For example, using synchronous reading:
        for (;;) {
            boost::system::error_code error;
            boost::asio::streambuf buffer;
            boost::asio::read_until(socket_, buffer, "\n", error);

            if (error) {
                RCLCPP_ERROR(this->get_logger(), "Read error: %s", error.message().c_str());
                break;
            }
            else {
                // Process incoming data
                std::istream is(&buffer);
                std::string line;
                std::getline(is, line);
                process_incoming_data(line);
            }
        }
    }

    void process_incoming_data(const std::string& data) {
        auto json_msg = nlohmann::json::parse(data);

        if (json_msg["type"] == "lidar") {
            publish_lidar_data(json_msg["data"]);
        }
        else if (json_msg["type"] == "imu") {
            publish_imu_data(json_msg["data"]);
        }
    }


    void publish_lidar_data(const nlohmann::json& lidar_data) {
        auto laser_scan_msg = sensor_msgs::msg::LaserScan();

        // Assuming the time fields are already filled correctly in Python
        laser_scan_msg.header.stamp.sec = lidar_data["header"]["stamp"]["sec"];
        laser_scan_msg.header.stamp.nanosec = lidar_data["header"]["stamp"]["nanosec"];
        laser_scan_msg.header.frame_id = lidar_data["header"]["frame_id"];

        laser_scan_msg.angle_min = lidar_data["angle_min"];
        laser_scan_msg.angle_max = lidar_data["angle_max"];
        laser_scan_msg.angle_increment = lidar_data["angle_increment"];
        laser_scan_msg.time_increment = lidar_data["time_increment"];
        laser_scan_msg.scan_time = lidar_data["scan_time"];
        laser_scan_msg.range_min = lidar_data["range_min"];
        laser_scan_msg.range_max = lidar_data["range_max"];

        // Assigning the ranges and intensities
        laser_scan_msg.ranges = lidar_data["ranges"].get<std::vector<float>>();
        laser_scan_msg.intensities = lidar_data["intensities"].get<std::vector<float>>();

        laser_scan_publisher->publish(laser_scan_msg);
    }


    void publish_imu_data(const nlohmann::json& imu_data) {
        auto imu_msg = sensor_msgs::msg::Imu();

        // Assuming the time fields are already filled correctly in Python
        imu_msg.header.stamp.sec = imu_data["header"]["stamp"]["sec"];
        imu_msg.header.stamp.nanosec = imu_data["header"]["stamp"]["nanosec"];
        imu_msg.header.frame_id = imu_data["header"]["frame_id"];

        imu_msg.orientation.x = imu_data["orientation"]["x"];
        imu_msg.orientation.y = imu_data["orientation"]["y"];
        imu_msg.orientation.z = imu_data["orientation"]["z"];
        imu_msg.orientation.w = imu_data["orientation"]["w"];

        imu_msg.angular_velocity.x = imu_data["angular_velocity"]["x"];
        imu_msg.angular_velocity.y = imu_data["angular_velocity"]["y"];
        imu_msg.angular_velocity.z = imu_data["angular_velocity"]["z"];

        imu_msg.linear_acceleration.x = imu_data["linear_acceleration"]["x"];
        imu_msg.linear_acceleration.y = imu_data["linear_acceleration"]["y"];
        imu_msg.linear_acceleration.z = imu_data["linear_acceleration"]["z"];

        imu_publisher->publish(imu_msg);
    }


private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;

    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cpp_server = std::make_shared<CPPServer>();
    rclcpp::spin(cpp_server);
    rclcpp::shutdown();
    return 0;
}
