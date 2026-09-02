// MatchFeedback assembly — builds the 29-byte report from the bridge tick +
// robot health, TIGERs' SystemMatchFeedback semantics adapted to what OUR
// hardware can actually measure:
//
//   * pose/velocity are the present estimator state used by the controller
//     (TIGERs report the delayed slot matched to the vision timepoint) with
//     the present-time output as fallback;
//   * the kicker level is a recharge MODEL (elapsed since the last fire
//     over the capacitor recharge time) — our kicker has no charge ADC.
//     kicker_max_v is a nominal constant for the server's scaling;
//   * barrier = the camera ball-contact signal; dribbler traction rides
//     the OFF/IDLE/STRONG ladder from the actuator policy (the server's
//     contact fallback chain);
//   * battery percent maps the moteus bus voltage over a configurable
//     empty/full window;
//   * feature bits advertise what is healthy: MOVE (motion enabled),
//     DRIBBLER + KICK_STRAIGHT (Arduino link), BARRIER (camera loop).
//     KICK_CHIP is never set — this robot has no chipper;
//   * onboard ball position is not estimated (bearing+size needs a camera
//     calibration we don't have yet): ballPosAge stays 255.
//
// Pure: no sockets, no clocks — the caller feeds everything in.
#pragma once

#include <cstdint>

#include "../Motion/actuators.h"
#include "../Motion/match_bridge.h"
#include "../Networks/matchctrl.h"

namespace rf {

struct FeedbackHealth {
    int robot_id = 0;     // command-channel id (also seq ownership)
    int hardware_id = 0;  // physical asset id reported to the server
    double battery_v = 0.0;
    bool arduino_connected = false;
    bool camera_running = false;
    bool estop = false;
    // Battery percent window (4S LiPo): empty..full volts.
    double battery_empty_v = 10.0;
    double battery_full_v = 16.8;
    // Kicker charge model: nominal max + recharge time (matches the
    // Arduino's own 5 s guard).
    double kicker_max_v = 200.0;
    double kicker_recharge_s = 5.0;
    // Onboard motion profile identity (Motion.yaml profile.id, or the last
    // MotionParams push) and whether the adaptive surface estimator runs.
    // The RL mode is read live from the bridge's augmentor.
    int motion_profile_id = 0;
    bool adaptive_enabled = false;
};

class MatchFeedbackBuilder {
public:
    // One frame per send tick. `bridge`/`bt` are the live bridge + freshest
    // tick (matched-slot pose and kicker charge come from the bridge),
    // `ball` the freshest ball observation.
    MatchFeedback build(const MatchBridge& bridge, const BridgeTick& bt,
                        const FeedbackHealth& health, const BallContactObs& ball,
                        double now_s);

private:
    uint16_t seq_ = 0;
};

}  // namespace rf
