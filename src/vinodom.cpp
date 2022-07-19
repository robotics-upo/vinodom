#include "rclcpp/rclcpp.hpp"
#include "vinodom.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VinOdom>());
  rclcpp::shutdown();
  return 0;
}
