/*
 * Created on Wed Aug 21 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#include "coroco_chassis/robot.hpp"


Robot::Robot(): x(0.0), y(0.0), theta(0.0), speed(0.0), omega(0.0) {}

Robot::Robot (double x, double y,  double theta, double speed, double omega)
        : x(x), y(y), theta(theta), speed(speed), omega(omega) {}

void Robot::step(double speed, double angular, double dt) {
    /* Update speed */
    this->speed = speed;

    /* Calculate the angular velocity (turning rate) */
    this->omega = angular;

    /* Update heading angle (theta) */
    theta += this->omega * dt;

    /* Update position (x, y) */
    x += speed * std::cos(theta) * dt;
    y += speed * std::sin(theta) * dt;
}