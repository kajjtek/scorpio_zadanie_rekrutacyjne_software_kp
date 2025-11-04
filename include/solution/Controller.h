#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "tester.hpp"
#include <queue>

class Controller {
    public:
    std::queue<Point> queue;
    std::uint16_t aim_position_1;
    std::uint16_t previousMistake_1;
    std::uint16_t sumOfMistakes_1;
    std::uint16_t aim_position_2;
    std::uint16_t previousMistake_2;
    std::uint16_t sumOfMistakes_2;
    void handleMotor1(std::uint16_t current_position);
    void handleMotor2(std::uint16_t current_position);
    void handleNewTarget(Point target);
    Controller() : previousMistake_1(0), sumOfMistakes_1(0) {}
    private:
    std::uint8_t calculateMove(std::uint16_t current_position, std::uint16_t aim_position);
    std::uint8_t componentP(std::uint16_t current_position, std::uint16_t aim_position);
    std::uint8_t componentI(std::uint16_t current_position, std::uint16_t aim_position);
    std::uint8_t componentD(std::uint16_t current_position, std::uint16_t aim_position);
    std::uint16_t translatePointToAngle(Point point);
};

#endif