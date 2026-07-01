#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <cmath>

// HeadingController — gyro/IMU heading hold for straight-line driving.
//
// The robot is commanded a body-frame yaw rate `w`. Open-loop, any small wheel
// imbalance makes a "drive straight" (w=0) command slowly curve. This controller
// closes the loop on the pi3hat IMU:
//
//   * When the operator commands a real turn (|w| > w_deadband) it passes the
//     command straight through and keeps re-capturing the current heading as the
//     new target, so releasing the turn holds the new heading.
//   * When |w| is ~0 (driving straight / translating) it actively holds the last
//     captured heading with a PD loop: w_out = kp*heading_error - kd*yaw_rate.
//
// Everything is clamped to max_w. `yaw_sign` flips the feedback polarity: if the
// robot ever *runs away* (spins faster instead of holding) when enabled, flip it.
// Disabled by default — enable + tune from config once verified on the bench.
class HeadingController {
public:
    bool   enabled    = false;   // OFF until bench-verified (config turns it on)
    double kp         = 4.0;     // rad/s per rad of heading error
    double kd         = 0.10;    // rad/s per (rad/s) of measured yaw rate (damping)
    double w_deadband = 0.20;    // rad/s: above this the operator is turning on purpose
    double max_w      = 3.15;    // rad/s output clamp
    double yaw_sign   = 1.0;     // +1 / -1: flip if feedback is the wrong way round

    // w_cmd     : commanded yaw rate (rad/s, + = CCW, robot convention)
    // yaw       : measured heading (rad) from the IMU
    // yaw_rate  : measured yaw rate (rad/s) from the IMU
    // returns the yaw rate to actually drive into the wheel kinematics.
    double update(double w_cmd, double yaw, double yaw_rate) {
        yaw      *= yaw_sign;
        yaw_rate *= yaw_sign;

        if (!enabled) { target_ = yaw; have_ = true; return w_cmd; }
        if (!have_)   { target_ = yaw; have_ = true; }

        if (std::fabs(w_cmd) > w_deadband) {
            target_ = yaw;              // deliberate turn: follow, re-capture heading
            return w_cmd;
        }
        // hold heading: PD on error, damped by measured yaw rate
        double w_out = kp * wrap(target_ - yaw) - kd * yaw_rate;
        if (w_out >  max_w) w_out =  max_w;
        if (w_out < -max_w) w_out = -max_w;
        return w_out;
    }

    void reset() { have_ = false; }

private:
    double target_ = 0.0;
    bool   have_   = false;
    static double wrap(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
};

#endif // HEADING_CONTROL_H
