/*
 * Created on Tue Aug 20 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#ifndef COROCO_CHASSIS_H
#define COROCO_CHASSIS_H

#include <rclcpp/rclcpp.hpp>
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <nav_msgs/msg/odometry.hpp>

#include "ttzn_sdk/coroco/tran.hpp"
#include "coroco_chassis/robot.hpp"
#include "coroco_msgs/msg/sys_status.hpp"
#include "coroco_msgs/msg/move_ctrl.hpp"
#include "coroco_msgs/msg/motor_info_fb.hpp"
#include "coroco_msgs/msg/warn_fb.hpp"
#include "coroco_msgs/msg/mode_ctrl.hpp"
#include "coroco_msgs/msg/light_ctrl.hpp"
#include "coroco_msgs/msg/odom_fb.hpp"
#include "coroco_msgs/msg/bms_fb.hpp"


class COROCODriver : public rclcpp::Node {
    /* Robot Model */
    Robot                           robot;

    /* CAN tran and recv */
    coroco::CANTran                 canTran;

    /* ROS related */
    rclcpp::TimerBase::SharedPtr    timer_;
    tf2_ros::TransformBroadcaster   tfBroadcaster;
    rclcpp::Time                    currentTime;

    /* Subscriber */
    rclcpp::Subscription<coroco_msgs::msg::MoveCtrl>::SharedPtr   moveCtrlSub;
    rclcpp::Subscription<coroco_msgs::msg::ModeCtrl>::SharedPtr   modeCtrlSub;
    rclcpp::Subscription<coroco_msgs::msg::LightCtrl>::SharedPtr  lightCtrlSub;

    /* Publisher for feedback */
    rclcpp::Publisher<coroco_msgs::msg::SysStatus>::SharedPtr     sysStatusPub;
    rclcpp::Publisher<coroco_msgs::msg::MoveCtrl>::SharedPtr      moveCtrlFbPub;
    rclcpp::Publisher<coroco_msgs::msg::MotorInfoFb>::SharedPtr   motorInfoFbPub;
    /* odomFbPub data is from STM32 wheel's odom */
    rclcpp::Publisher<coroco_msgs::msg::OdomFb>::SharedPtr        odomFbPub;

    /* odomPub data is calculated through math kinematic model */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr         odomPub;

    /* Messages for feedback */
    coroco_msgs::msg::SysStatus         sysStatusMsg;
    coroco_msgs::msg::MoveCtrl          moveCtrlFbMsg;
    coroco_msgs::msg::MotorInfoFb       motorInfoFbMsg;
    coroco_msgs::msg::OdomFb            odomFbMsg;

    nav_msgs::msg::Odometry             odomMsg;

    /* Parameters */
    bool                                pub_tf;
    /* frame name */
    std::string                         base_frame;
    std::string                         odom_frame;
    /* CAN device */
    std::string                         dev_path;
    std::string                         dev_type;

public:
    COROCODriver(const std::string& node_name = "cody_driver", int rate = 50);

    ~COROCODriver();

private:
    void run();

    void moveCtrlCallback(const coroco_msgs::msg::MoveCtrl& msg);

    void modeCtrlCallback(const coroco_msgs::msg::ModeCtrl& msg);

    void lightCtrlCallback(const coroco_msgs::msg::LightCtrl& msg);

    void publishOdom(double speed, double corner, double dt);
};



#endif /* end COROCO_CHASSIS_H */