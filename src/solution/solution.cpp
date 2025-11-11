#include "tester.hpp"

// You can remove or add any includes you need
#include <chrono>
#include <iostream>
#include <thread>
#include "Controller.h"

int solver(std::shared_ptr<backend_interface::Tester> tester, bool preempt) {
  auto controller = std::make_shared<Controller>();
  controller->setQueueAvailability(preempt);
  auto commands = tester->get_commands();
  commands->add_data_callback([controller](const Point& point) {
    std::cout << "New target: " << point.x << ", " << point.y << ", " << point.z << std::endl;
    controller->handleNewTarget(point);
  });
  auto motor1 = tester->get_motor_1();
  auto motor2 = tester->get_motor_2();
  
  motor1->add_data_callback([controller, motor1](const uint16_t& data) {
    std::cout<< "Motor1: " << data << std::endl;
    std::int8_t signal = controller->handleMotorHorizontal(data);
    std::cout<<"+ HORIZONTAL MOVE: " << static_cast<int>(signal) <<std::endl;
    motor1->send_data(signal);
  });
  motor2->add_data_callback([controller, motor2](const uint16_t& data) {
    std::cout<< "Motor2: " << data << std::endl;
    std::int8_t signal = controller->handleMotorVertical(data);
    std::cout<<"+ VERTICAL MOVE: " << static_cast<int>(signal) <<std::endl;
    motor2->send_data(signal);
  });
  

  std::cout << "Running -- press Ctrl+C to stop" << std::endl;
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
