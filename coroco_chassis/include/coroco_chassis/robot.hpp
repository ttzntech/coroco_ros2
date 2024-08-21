/*
 * Created on Tue Aug 20 2024
 *
 * Author: Joonchen Liau
 * Email: liaojuncheng123@foxmail.com
 *
 * Copyright (c) 2024 Tiante Intelligent Technology
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>


class Robot {
public:
    Robot ();

    Robot (double x, double y, double theta, double speed, double omega);

    void step(double speed, double angular, double dt);

    double x;           /* Current x position */
    double y;           /* Current y position */
    double theta;       /* Current heading angle (radians) */
    double speed;       /* Current speed (m/s) */
    double omega;       /* Current omega (rad/s) */
};


#endif /* end ROBOT_H */