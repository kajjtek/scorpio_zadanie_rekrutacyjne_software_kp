#include "Controller.h"
#include <cmath>
#include <limits>
#include <iostream>
#include <cstdint>

std::pair<std::uint16_t, std::uint16_t> Controller::translatePointToAngle(Point point){
    return {translatePointToHorizontalAngle(point), translatePointToVerticalAngle(point)};
}

std::uint16_t Controller::translatePointToHorizontalAngle(Point point){
    double horizontal_angle_rad = atan2(point.y, point.x);
    int horizontal_angle_enc = std::round((horizontal_angle_rad)/(2*M_PI)*4096);
    int normalised_horizontal_angle_enc = ((horizontal_angle_enc % 4096)+4096)%4096;
    std::uint16_t enc_horizontal = static_cast<std::uint16_t>(normalised_horizontal_angle_enc);
    return enc_horizontal;
}

//nie wiedzialem czy kamera moze sie obracac 180 stopni czy 360 - zakladam 360 ostatecznie
std::uint16_t Controller::translatePointToVerticalAngle(Point point){
        double vector_x_y_length = sqrt(point.x*point.x+point.y*point.y);
        double vertical_angle_rad = atan2(point.z, vector_x_y_length);
        int vertical_angle_enc = std::round((vertical_angle_rad)/2*M_PI*4096);
        int normalised_vertical_angle_enc = ((vertical_angle_enc % 4096)+4096)%4096; 
        std::uint16_t enc_vertical = static_cast<std::uint16_t>(normalised_vertical_angle_enc);
        return enc_vertical;
}

void Controller::handleNewTarget(Point target){
    std::pair<std::uint16_t, std::uint16_t> encs = translatePointToAngle(target);
    if(this->queueable){
        queue.push(encs);
        if(queue.size()==1 && this->horizontal_at_target && this->vertical_at_target){
            nextTarget();
        }
    }else {
        setTarget(encs.first, encs.second);
    }
}

void Controller::nextTarget(){
    if (queue.empty()) {
        this->target_position_horizontal = std::nullopt; 
        this->target_position_vertical = std::nullopt;
        return; 
    }

    std::pair<std::uint16_t, std::uint16_t> newTarget = queue.front();
    queue.pop();
    setTarget(newTarget.first, newTarget.second);
}

std::int8_t Controller::handleMotorHorizontal(std::uint16_t current_position){
    if(!this->target_position_horizontal.has_value()){
      return 0;  
    }

    std::uint16_t target_postion = this->target_position_horizontal.value();
    int raw_error = static_cast<int>(target_postion) - static_cast<int>(current_position);
    int error = raw_error;


    if(raw_error>2048){
        error -= 4096;
    }
    else if (raw_error<-2048) {
        error += 4096;
    }

    if(abs(error)<=1){
        this->horizontal_at_target = true;
        if(this->vertical_at_target){
             std::cout<<"------------------------------------------------------------------------------------------"<<std::endl;
            std::cout<<"----------- TARGET ACHIEVED"<<std::endl;
            nextTarget();
            return 0;
        }
        this->target_position_horizontal = std::nullopt; 
    }

    std::int8_t move = calculateMove(error, this->sumOfErrors_horizontal, this->previousError_horizontal);
    this->previousError_horizontal=error;
    this->sumOfErrors_horizontal+=error;

    return move;
}

std::int8_t Controller::handleMotorVertical(std::uint16_t current_position){
    if(!this->target_position_vertical.has_value()){
      return 0;  
    }

    std::uint16_t target_postion = this->target_position_vertical.value();
    int raw_error = static_cast<int>(target_postion) - static_cast<int>(current_position);
    int error = raw_error;

    if(raw_error>2048){
        error -= 4096;
    }
    else if (raw_error<-2048) {
        error += 4096;
    }

    if(abs(error)<=1){
        this->vertical_at_target = true;
        if(this->horizontal_at_target){
            std::cout<<"----------- TARGET ACHIEVED"<<std::endl;
            nextTarget();
            return 0;
        }
        this->target_position_vertical = std::nullopt; 
    }

    std::int8_t move = calculateMove(error, this->sumOfErrors_vertical, this->previousError_vertical);
    this->previousError_vertical=error;
    this->sumOfErrors_vertical+=error;

    return move;
}

std::int8_t Controller::calculateMove(int error, std::int32_t sumerror, std::int32_t preverror){
    const double p_constant = 10;
    const double i_constant = 0.1;
    const double d_constant = 0.1;
    int result = std::round(p_constant * componentP(error) + i_constant * componentI(sumerror) + d_constant * componentD(preverror));
    if(result>127) return INT8_MAX;
    if(result<-127) return INT8_MIN;
    return static_cast<std::int8_t>(result);
}

int Controller::componentP(int error){
    return error;
}

int Controller::componentI(std::int32_t sum_error){
    return sum_error;
}

int Controller::componentD(std::int32_t prev_error){
    return prev_error;
}