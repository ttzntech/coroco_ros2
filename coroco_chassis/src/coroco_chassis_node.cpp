/*
 * Created on Wed Aug 21 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#include <rclcpp/rclcpp.hpp>

#include "coroco_chassis/coroco_chassis.hpp"


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<COROCODriver>());
    rclcpp::shutdown();
    return 0;
}