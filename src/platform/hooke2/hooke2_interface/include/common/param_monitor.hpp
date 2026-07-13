#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <iostream>
#include <fmt/format.h>
#include <fmt/color.h>

class ParamMonitor {
public:
    ParamMonitor() {
        std::cout << "ParamMonitor constructor called" << std::endl;
        fmt::print("\033[?25l");  // Hide cursor
        printHeader();
    }

    void printHeader() {
        std::cout << "printHeader called" << std::endl;
        constexpr int param_width = 18;
        constexpr int current_width = 18;
        constexpr int command_width = 18;

        const std::string separator = 
            fmt::format("\033[37m+{:-<{}}+{:-<{}}+{:-<{}}+\033[0m\n",
                        "", param_width+2, 
                        "", current_width+2,
                        "", command_width+2);

        const std::string header = 
            fmt::format("| {:<{}} | {:<{}} | {:<{}} |\n",
                        "Param", param_width,
                        "Current", current_width,
                        "Command", command_width);

        fmt::print("{}{}{}", separator, header, separator);
        fmt::print("\033[s");  // Save cursor position
    }
    struct ParamData {
        std::string current_value;
        fmt::color current_color;
        std::string command_value;
        fmt::color command_color;
    };

    static ParamMonitor& getInstance() {
        static ParamMonitor instance;
        return instance;
    }

    void updateCurrent(const std::string& name, 
                      const std::string& value,
                      fmt::color color = fmt::color::white) {
        std::lock_guard<std::mutex> lock(mutex_);
        // std::cout << "Updating current value for " << name << ": " << value << std::endl; // 调试信息
        auto& param = params_[name];
        param.current_value = value;
        param.current_color = color;
        need_refresh_ = true;
    }

    void updateCommand(const std::string& name,
                      const std::string& value,
                      fmt::color color = fmt::color::white) {
        std::lock_guard<std::mutex> lock(mutex_);
        // std::cout << "Updating command value for " << name << ": " << value << std::endl; // 调试信息
        auto& param = params_[name];
        param.command_value = value;
        param.command_color = color;
        need_refresh_ = true;
    }

    // 保持原有的start/stop方法
    void startAutoRefresh(int interval_ms = 100) {
        if (running_) return;
        running_ = true;
        refresh_thread_ = std::thread([this, interval_ms]() {
            try {
                while (running_) {
                    if (need_refresh_) {
                        // std::cout << "Refreshing display..." << std::endl; // Debug information
                        refreshDisplay();
                        need_refresh_ = false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
                }
            } catch (const std::exception& e) {
                std::cerr << "Exception in refresh thread: " << e.what() << std::endl;
            }
        });
    }

    void refreshDisplay() {
        try {
            std::lock_guard<std::mutex> lock(mutex_);
            static int last_line_count = 0;
    
            // std::cout << "In refreshDisplay, params size: " << params_.size() << std::endl; // Debug information
    
            // Move to the start of the data area
            fmt::print("\033[u");
            
            // // Clear the original data
            // if (last_line_count > 0) {
            //     fmt::print("\033[{}M", last_line_count); // Delete lines
            // }
    
            // Build data rows
            constexpr int param_width = 18;
            constexpr int current_width = 18;
            constexpr int command_width = 18;
            
            std::string output;
            for (const auto& [name, data] : params_) {
                output += fmt::format(
                    "| {:<{}} | {}{} | {}{} |\n",
                    name, param_width,
                    fmt::format(fg(data.current_color), "{:<{}}", 
                              data.current_value, current_width),
                    "\033[0m", // Reset color
                    fmt::format(fg(data.command_color), "{:<{}}", 
                              data.command_value, command_width),
                    "\033[0m" // Reset color
                );
            }
            
            // Add bottom separator
            output += fmt::format("\033[37m+{:-<{}}+{:-<{}}+{:-<{}}+\033[0m\n",
                                "", param_width+2,
                                "", current_width+2,
                                "", command_width+2);
    
            // Record the current number of lines
            last_line_count = params_.empty() ? 1 : (params_.size() + 1);
            fmt::print("{}", output);
        } catch (const std::exception& e) {
            std::cerr << "Exception in refreshDisplay: " << e.what() << std::endl;
        }
    }
    void stopAutoRefresh() {
        running_ = false;
        if (refresh_thread_.joinable()) {
            refresh_thread_.join();
        }
    }
    // 保持原有的线程控制成员变量
    std::map<std::string, ParamData> params_;
    std::mutex mutex_;
    std::atomic<bool> need_refresh_{false};
    std::atomic<bool> running_{false};
    std::thread refresh_thread_;
};

// // 测试用例
// int main() {
//     ParamMonitor::getInstance().startAutoRefresh();

//     // 初始参数设置
//     ParamMonitor::getInstance().updateCurrent("Temperature", "36.5C", 
//                                              fmt::color::red);
//     ParamMonitor::getInstance().updateCommand("Temperature", "35.0C", 
//                                             fmt::color::green);
    
//     ParamMonitor::getInstance().updateCurrent("Battery", "85%", 
//                                              fmt::color::green);
//     ParamMonitor::getInstance().updateCommand("Battery", "90%", 
//                                             fmt::color::yellow);

//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     // 更新参数
//     ParamMonitor::getInstance().updateCurrent("Temperature", "38.1C", 
//                                              fmt::color::yellow);
//     ParamMonitor::getInstance().updateCommand("Temperature", "37.5C", 
//                                             fmt::color::cyan);

//     ParamMonitor::getInstance().updateCurrent("MotorSpeed", "2500RPM", 
//                                              fmt::color::blue);
//     ParamMonitor::getInstance().updateCommand("MotorSpeed", "3000RPM", 
//                                             fmt::color::magenta);

//     std::this_thread::sleep_for(std::chrono::seconds(2));

//     // 移除参数测试
//     ParamMonitor::getInstance().updateCurrent("Battery", "", 
//                                              fmt::color::white);
//     ParamMonitor::getInstance().updateCommand("Battery", "", 
//                                             fmt::color::white);

//     std::this_thread::sleep_for(std::chrono::seconds(1));
//     return 0;
// }