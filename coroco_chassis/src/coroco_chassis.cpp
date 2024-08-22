/*
 * Created on Tue Aug 20 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#include "coroco_chassis/coroco_chassis.hpp"

using namespace coroco;


COROCODriver::COROCODriver(const std::string& node_name, int rate):
Node(node_name),
tfBroadcaster(this),
currentTime(this->now()),
pub_tf(this->declare_parameter("pub_tf", true)),
base_frame(this->declare_parameter("base_frame", "base_link")),
odom_frame(this->declare_parameter("odom_frame", "odom")),
dev_path(this->declare_parameter("dev_path", "/dev/ttyUSB0")),
dev_type(this->declare_parameter("dev_type", 0)),
canTran(dev_path, static_cast<DevType>(dev_type))
{
    RCLCPP_INFO(this->get_logger(), "dev_path: %s  dev_type: %hhu \n", dev_path.c_str(), dev_type);

    /* Subscriber setup */
    moveCtrlSub = this->create_subscription<coroco_msgs::msg::MoveCtrl>(
        "/cmd_vel", 10, std::bind(&COROCODriver::moveCtrlCallback, this, std::placeholders::_1));
    modeCtrlSub = this->create_subscription<coroco_msgs::msg::ModeCtrl>(
        "mode_ctrl", 10, std::bind(&COROCODriver::modeCtrlCallback, this, std::placeholders::_1));
    lightCtrlSub = this->create_subscription<coroco_msgs::msg::LightCtrl>(
        "light_ctrl", 10, std::bind(&COROCODriver::lightCtrlCallback, this, std::placeholders::_1));

    /* Publisher setup */
    sysStatusPub = this->create_publisher<coroco_msgs::msg::SysStatus>("sys_status", 10);
    moveCtrlFbPub = this->create_publisher<coroco_msgs::msg::MoveCtrl>("move_ctrl_fb", 10);
    reMoveCtrlFbPub = this->create_publisher<coroco_msgs::msg::MoveCtrl>("re_move_ctrl_fb", 10);
    motorInfoFbPub = this->create_publisher<coroco_msgs::msg::MotorInfoFb>("motor_info_fb", 10);
    warnFbPub = this->create_publisher<coroco_msgs::msg::WarnFb>("warn_fb", 10);
    odomFbPub = this->create_publisher<coroco_msgs::msg::OdomFb>("odom_fb", 10);
    BMSFbPub = this->create_publisher<coroco_msgs::msg::BMSFb>("BMS_fb", 10);
    odomPub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / rate), std::bind(&COROCODriver::run, this));

    /* odomMsg covariance matrix setup */
    odomMsg.pose.covariance = {
        0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
        0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
        0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
        0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
        0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
        0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
    odomMsg.twist.covariance = {
        0.001,      0.0,        0.0,        0.0,        0.0,        0.0,
        0.0,        0.001,      0.0,        0.0,        0.0,        0.0,
        0.0,        0.0,        1000000.0,  0.0,        0.0,        0.0,
        0.0,        0.0,        0.0,        1000000.0,  0.0,        0.0,
        0.0,        0.0,        0.0,        0.0,        1000000.0,  0.0,
        0.0,        0.0,        0.0,        0.0,        0.0,        1000.0};
}

void COROCODriver::run() {
    rclcpp::Time currentTime_ = this->now();
    double dt = (currentTime_ - currentTime).seconds();

    /* sys status publisher */
    canTran.recv(ID_SysStatus);
    sysStatusMsg.cur_status = canTran.data.i211SysStatus.cur_status;
    sysStatusMsg.ctrl_mode = canTran.data.i211SysStatus.ctrl_mode;
    sysStatusMsg.bat_vol = canTran.data.i211SysStatus.bat_vol;
    sysStatusMsg.bat_cur = canTran.data.i211SysStatus.bat_cur;
    sysStatusMsg.parity = canTran.data.i211SysStatus.parity;
    sysStatusPub->publish(sysStatusMsg);

    /* move ctrl feedback publisher */
    canTran.recv(ID_MoveCtrlFb);
    moveCtrlFbMsg.speed = canTran.data.i221MoveCtrlFb.speed;
    moveCtrlFbMsg.angular = canTran.data.i221MoveCtrlFb.angular;
    moveCtrlFbPub->publish(moveCtrlFbMsg);

    /* remote move ctrl feedback publisher */
    canTran.recv(ID_ReMoveCtrlFb);
    reMoveCtrlFbMsg.speed = canTran.data.i241ReMoveCtrlFb.speed;
    reMoveCtrlFbMsg.angular = canTran.data.i241ReMoveCtrlFb.angular;
    reMoveCtrlFbPub->publish(reMoveCtrlFbMsg);

    /* motor info feedback publisher */
    canTran.recv(ID_Motor1InfoFb);
    motorInfoFbMsg.motor1_rpm = canTran.data.i250Motor1InfoFb.rpm;
    motorInfoFbMsg.motor1_pos = canTran.data.i250Motor1InfoFb.pos;
    canTran.recv(ID_Motor2InfoFb);
    motorInfoFbMsg.motor2_rpm = canTran.data.i251Motor2InfoFb.rpm;
    motorInfoFbMsg.motor2_pos = canTran.data.i251Motor2InfoFb.pos;
    canTran.recv(ID_Motor3InfoFb);
    motorInfoFbMsg.motor3_rpm = canTran.data.i252Motor3InfoFb.rpm;
    motorInfoFbMsg.motor3_pos = canTran.data.i252Motor3InfoFb.pos;
    canTran.recv(ID_Motor4InfoFb);
    motorInfoFbMsg.motor4_rpm = canTran.data.i253Motor4InfoFb.rpm;
    motorInfoFbMsg.motor4_pos = canTran.data.i253Motor4InfoFb.pos;
    motorInfoFbPub->publish(motorInfoFbMsg);

    /* warn feedback publisher */
    canTran.recv(ID_WarnFb);
    warnFbMsg.steer_motor_warn = canTran.data.i261WarnFb.steer_motor_warn;
    warnFbMsg.motor1_warn = canTran.data.i261WarnFb.motor1_warn;
    warnFbMsg.motor2_warn = canTran.data.i261WarnFb.motor2_warn;
    warnFbMsg.bat_warn = canTran.data.i261WarnFb.bat_warn;
    warnFbMsg.temp1 = canTran.data.i261WarnFb.temp1;
    warnFbMsg.temp2 = canTran.data.i261WarnFb.temp2;
    warnFbMsg.warn = canTran.data.i261WarnFb.warn;
    warnFbPub->publish(warnFbMsg);

    /* odom feedback publisher */
    canTran.recv(ID_OdomFb);
    odomFbMsg.odom = canTran.data.i311OdomFb.odom;
    odomFbPub->publish(odomFbMsg);

    /* BMS feedback publisher */
    canTran.recv(ID_BMSFb);
    BMSFbMsg.bat_soc = canTran.data.i361BMSFb.bat_soc;
    BMSFbMsg.vol = canTran.data.i361BMSFb.vol;
    BMSFbMsg.cur = canTran.data.i361BMSFb.cur;
    BMSFbMsg.temp = canTran.data.i361BMSFb.temp;
    BMSFbPub->publish(BMSFbMsg);

    /* calculate odom and publish */
    publishOdom(moveCtrlFbMsg.speed, moveCtrlFbMsg.angular, dt);
}

COROCODriver::~COROCODriver() {

}

void COROCODriver::moveCtrlCallback(const coroco_msgs::msg::MoveCtrl& msg) {
    canTran.data.i111MoveCtrl.speed = msg.speed;
    canTran.data.i111MoveCtrl.angular = msg.angular;
    canTran.send(ID_MoveCtrl);
}

void COROCODriver::modeCtrlCallback(const coroco_msgs::msg::ModeCtrl& msg) {
    canTran.data.i421ModeCtrl.mode = static_cast<E421Mode>(msg.mode);
    canTran.send(ID_ModeCtrl);
}

void COROCODriver::lightCtrlCallback(const coroco_msgs::msg::LightCtrl& msg) {
    canTran.data.i121LightCtrl.front = static_cast<E121Light>(msg.front);
    canTran.data.i121LightCtrl.parity = msg.parity;
    canTran.send(ID_LightCtrl);
}

void COROCODriver::publishOdom(double speed, double angular, double dt) {
    /* step model */
    robot.step(speed, angular, dt);

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, robot.theta);

    /* publish tf transformation */
    if (pub_tf) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = currentTime;
        tf_msg.header.frame_id = base_frame;
        tf_msg.child_frame_id = odom_frame;

        tf_msg.transform.translation.x = robot.x;
        tf_msg.transform.translation.y = robot.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = tf2::toMsg(odom_quat);

        tfBroadcaster.sendTransform(tf_msg);
    }

    /* publish odometry message */
    odomMsg.header.stamp = currentTime;
    odomMsg.header.frame_id = odom_frame;
    odomMsg.child_frame_id = base_frame;

    odomMsg.pose.pose.position.x = robot.x;
    odomMsg.pose.pose.position.y = robot.y;
    odomMsg.pose.pose.position.z = 0.0;
    odomMsg.pose.pose.orientation = tf2::toMsg(odom_quat);

    odomMsg.twist.twist.linear.x = robot.speed;
    odomMsg.twist.twist.linear.y = 0.0;
    odomMsg.twist.twist.angular.z = robot.omega;

    //   std::cerr << "linear: " << linear_speed_ << " , angular: " << steering_angle_
    //             << " , pose: (" << position_x_ << "," << position_y_ << ","
    //             << theta_ << ")" << std::endl;

    odomPub->publish(odomMsg);
}