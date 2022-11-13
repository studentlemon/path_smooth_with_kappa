//
// Created by ljn on 20-1-26.
//

#ifndef PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
#define PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>
#include <ctime>

namespace tk {
class spline;
}

class State;

// Set angle to -pi ~ pi
template<typename T>
T constrainAngle(T angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
        return constrainAngle(angle);
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
        return constrainAngle(angle);
    } else {
        return angle;
    }
}

// Time duration in seconds.
double time_s(const clock_t &begin, const clock_t &end);

// Time duration in ms.
double time_ms(const clock_t &begin, const clock_t &end);

// Output time duration.
void time_s_out(const clock_t &begin, const clock_t &end, const std::string &text);
void time_ms_out(const clock_t &begin, const clock_t &end, const std::string &text);

// Return true if a == b.
bool isEqual(double a, double b);

// Calculate heading for spline.
double getHeading(const tk::spline &xs, const tk::spline &ys, double s);

// Calculate curvature for spline.
double getCurvature(const tk::spline &xs, const tk::spline &ys, double tmp_s);

// Calculate distance between two points.
double caldistance(const State &p1, const State &p2);

// Coordinate transform.
State local2Global(const State &reference, const State &target);
State global2Local(const State &reference, const State &target);

// Find the closest point on spline.
State getProjection(const tk::spline &xs,
                    const tk::spline &ys,
                    double target_x,
                    double target_y,
                    double max_s,
                    double start_s = 0.0);

State getProjectionByNewton(const tk::spline &xs,
                            const tk::spline &ys,
                            double target_x,
                            double target_y,
                            double max_s,
                            double hint_s);

State getDirectionalProjection(const tk::spline &xs,
                               const tk::spline &ys,
                               double target_x,
                               double target_y,
                               double angle,
                               double max_s,
                               double start_s = 0.0);

State getDirectionalProjectionByNewton(const tk::spline &xs,
                                       const tk::spline &ys,
                                       double target_x,
                                       double target_y,
                                       double angle,
                                       double max_s,
                                       double hint_s);

#endif //PATH_OPTIMIZER_INCLUDE_TOOLS_TOOLS_HPP_
