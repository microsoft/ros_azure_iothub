#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("dynamic_tutorials_node") {
        this->declare_parameter("str_param");
        this->declare_parameter("int_param");
        this->declare_parameter("double_param");
        this->declare_parameter("bool_param");
    }
private:
};
// Code below is just to start the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

