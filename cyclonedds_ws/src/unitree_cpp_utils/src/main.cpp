/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of
 *unitree go2 robot
 **/
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "motor_crc.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

namespace py = pybind11;

class LowControl {
 public:
  std::shared_ptr<std::thread> t;
  LowControl() {
    char *argv[] = {strdup("python")};
    rclcpp::init(1, argv);
    node = std::make_shared<rclcpp::Node>("low_level_cmd_sender");

    /* create publisher */
    cmd_pub = node->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    initCmd();

    t = std::make_shared<std::thread>([this]() { rclcpp::spin(node); });
  }

  ~LowControl() { rclcpp::shutdown(); t->join(); }

  void publish(py::array_t<float> q, py::array_t<float> kp, float kd) {
    auto q_data = q.unchecked<1>();
    auto kp_data = kp.unchecked<1>();
    for (int i = 0; i < 12; i++) {
      cmd_msg.motor_cmd[i].q = q_data[i];
      cmd_msg.motor_cmd[i].kp = kp_data[i];
      cmd_msg.motor_cmd[i].kd = kd;
      raw.motorCmd[i].q = q_data[i];
      raw.motorCmd[i].Kp = kp_data[i];
      raw.motorCmd[i].Kd = kd;
    }
    cmd_msg.crc = get_crc(raw);
    cmd_pub->publish(cmd_msg);
  }

 private:
  void initCmd() {
    for (int i = 0; i < 20; i++) {
      cmd_msg.motor_cmd[i].mode = 0x01;  // Set toque mode, 0x00 is passive mode
      cmd_msg.motor_cmd[i].q = 0;
      cmd_msg.motor_cmd[i].kp = 0;
      cmd_msg.motor_cmd[i].dq = 0;
      cmd_msg.motor_cmd[i].kd = 0;
      cmd_msg.motor_cmd[i].tau = 0;
    }
    cpy_msg(raw, cmd_msg);
  }

  rclcpp::Node::SharedPtr node;

  /* publishers */
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_pub;

  unitree_go::msg::LowCmd cmd_msg;
  LowCmd raw{};
};

PYBIND11_MODULE(go2_cpp_utils, m) {
  py::class_<LowControl>(m, "LowControl")
      .def(py::init<>())
      .def("publish", &LowControl::publish);
}
