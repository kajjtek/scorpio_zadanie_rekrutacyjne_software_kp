#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "tester.hpp"
#include <queue>
#include <utility>
#include <optional>

class Controller {
    public:
    /* Variables */
    std::queue<std::pair<std::uint16_t, std::uint16_t>> queue;
    std::optional<std::uint16_t> target_position_horizontal = std::nullopt;
    std::optional<std::uint16_t> target_position_vertical = std::nullopt;
    /* Functions */
    void setQueueAvailability(bool flag){this->queueable=flag;}
    std::int8_t handleMotorHorizontal(std::uint16_t current_position);
    std::int8_t handleMotorVertical(std::uint16_t current_position);
    void handleNewTarget(Point target);
    private:
    /* Variables */
    bool queueable = false;
    std::int32_t previousError_horizontal;
    std::int32_t sumOfErrors_horizontal;
    std::int32_t previousError_vertical;
    std::int32_t sumOfErrors_vertical;
    /* Functions */
    std::int8_t calculateMove(std::uint16_t error, std::uint16_t sumerror, std::uint16_t preverror);
    std::int8_t componentP(std::uint16_t error);
    std::int8_t componentI(std::uint16_t error);
    std::int8_t componentD(std::uint16_t error);
    std::pair<std::uint16_t, std::uint16_t> translatePointToAngle(Point point);
    std::uint16_t translatePointToVerticalAngle(Point point);
    std::uint16_t translatePointToHorizontalAngle(Point point);
    void nextTarget();
    void setTarget(std::uint16_t target_horizontal, std::uint16_t target_vertical){
        this->target_position_horizontal=target_horizontal;
        this->target_position_vertical=target_vertical;
        
        this->previousError_horizontal = 0;
        this->sumOfErrors_horizontal = 0;
        this->previousError_vertical = 0;
        this->sumOfErrors_vertical = 0;
    }
};

#endif