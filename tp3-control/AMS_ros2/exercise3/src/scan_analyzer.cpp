#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric> 
#include <iostream>
#include <fstream>

class ScanPlotter : public rclcpp::Node {
public:
    ScanPlotter() : Node("scan_plotter") {
        auto qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&ScanPlotter::scan_callback, this, std::placeholders::_1));
        
        cv::namedWindow("Laser Scan Live", cv::WINDOW_AUTOSIZE);
    }

    void display_final_results() {
        if (history_sums.empty()) {
            std::cout << "no data acquired -_-" << std::endl;
            return;
        }

        size_t n = history_sums.size();
        
        float sum = std::accumulate(history_sums.begin(), history_sums.end(), 0.0f);
        float mean = sum / n;

        std::vector<float> sorted_sums = history_sums;
        std::sort(sorted_sums.begin(), sorted_sums.end());
        float median = (n % 2 == 0) ? (sorted_sums[n/2 - 1] + sorted_sums[n/2]) / 2 : sorted_sums[n/2];

        float variance = 0;
        for (float v : history_sums) {
            variance += std::pow(v - mean, 2);
        }
        variance /= n;
        float std_dev = std::sqrt(variance);

        std::cout << "\n========== Stats ==========" << std::endl;
        std::cout << "nbr : " << n << std::endl;
        std::cout << "Average              : " << mean << std::endl;
        std::cout << "Median              : " << median << std::endl;
        std::cout << "Standard Deviation           : " << std_dev << std::endl;
        std::cout << "==========================================" << std::endl;

    }

public:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float current_sum = 0;
        int img_size = 500;
        cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC3);
        cv::Point2i center(img_size / 2, img_size / 2);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
                current_sum += range;
                
                float angle = msg->angle_min + i * msg->angle_increment;
                int x = center.x + static_cast<int>(range * std::cos(angle) * 50.0);
                int y = center.y + static_cast<int>(range * std::sin(angle) * 50.0);
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
        
        history_sums.push_back(current_sum);
        cv::imshow("Laser Scan Live", img);
        cv::waitKey(1);
    }
    void save_data_to_file() {
    	std::ofstream file("sums.txt");
    	for (const auto& v : history_sums) {
        	file << v << "\n";
    	}
    	file.close();
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::vector<float> history_sums;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanPlotter>();
    rclcpp::spin(node);
    node->save_data_to_file();
    node->display_final_results();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
