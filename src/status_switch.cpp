#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>

#include <memory>
#include <string>

// 获取键盘按键的非阻塞函数
char getKey()
{
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("keyboard_node");

    auto key_pub = node->create_publisher<std_msgs::msg::String>("/key_input", 10);

    RCLCPP_INFO(node->get_logger(),
                "Keyboard Control Started. Press 'm' to switch mode, 's' to stop, 'q' to exit.");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok())
    {
        char c = getKey();
        if (c == 'q')
        {
            break;
        }

        std_msgs::msg::String msg;
        msg.data = std::string(1, c);
        key_pub->publish(msg);

        executor.spin_some(); // 相当于 ros::spinOnce()
    }

    rclcpp::shutdown();
    return 0;
}
