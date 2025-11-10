#include "tester.hpp"

// You can remove or add any includes you need
#include <chrono>
#include <iostream>
#include <thread>
#include "Controller.h"

int solver(std::shared_ptr<backend_interface::Tester> tester, bool preempt) {

  auto controller = std::make_shared<Controller>();
  controller->setQueueAvailability(preempt);
  // Short example you can remove it
  std::cout << (preempt ? "Preempt" : "Queue") << '\n';
  auto motor1 = tester->get_motor_1();
  auto motor2 = tester->get_motor_2();
  auto commands = tester->get_commands();
  motor1->add_data_callback([controller, motor1](const uint16_t& data) {
    std::cout<< "Motor1 new: " << data << std::endl;
    std::int8_t signal = controller->handleMotorHorizontal(data);
    motor1->send_data(signal);
  });
  motor2->add_data_callback([controller, motor2](const uint16_t& data) {
    std::cout<< "Motor2 new : " << data << std::endl;
    std::int8_t signal = controller->handleMotorVertical(data);
    motor2->send_data(signal);
  });
  commands->add_data_callback([controller](const Point& point) {
    std::cout << "New target: " << point.x << ", " << point.y << ", " << point.z << std::endl;
    controller->handleNewTarget(point);
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(25000));
  //
  return 0;
}
