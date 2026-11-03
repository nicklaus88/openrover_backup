#include "BaseDriver.h"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaseDriver>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok()) {
    // Wait up to 2 ms for a timer or message instead of returning at once:
    // the old spin_some() + communicate() loop never slept and kept a core
    // busy. Then handle every MSP reply the receiver thread has queued
    // (packets carry their receive timestamp, so the wait skews nothing).
    executor.spin_once(2ms);
    while (node->communicate()) {
    }
  }
  rclcpp::shutdown();
  return 0;
}
