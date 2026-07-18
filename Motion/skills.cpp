#include "skills.h"

#include <algorithm>

#include "phx/angle.h"

namespace rf {

std::array<double, 4> wheel_order_wire_to_can(const std::array<double, 4>& wire) {
    // Wire (TIGERs): [0]=FR [1]=FL [2]=RL [3]=RR.
    // Ours (CAN id 1..4): [0]=FR [1]=RR [2]=RL [3]=FL.
    return {wire[0], wire[3], wire[2], wire[1]};
}

MotionSetpoint decode_skill(const MatchCtrl& mc) {
    MotionSetpoint out;
    out.skill_id = mc.skill_id;

    switch (static_cast<SkillId>(mc.skill_id)) {
    case SkillId::GlobalPos: {
        const GlobalPosSkill s = decode_global_pos(mc.skill_data);
        out.kind = MotionSetpoint::Kind::Pose;
        out.target.pos = {s.tx, s.ty};
        out.target.heading = phx::wrap_angle(s.ttheta);
        out.vel_max_xy = std::max(s.vel_max_xy, kSkillMinVelXY);
        out.vel_max_w = std::max(s.vel_max_w, kSkillMinVelW);
        out.acc_max_xy = std::max(s.acc_max_xy, kSkillMinAccXY);
        out.acc_max_w = s.acc_max_w;
        out.primary_direction = s.primary_direction;
        out.kd = s.kd;
        return out;
    }
    case SkillId::FastPos: {
        const FastPosSkill s = decode_fast_pos(mc.skill_data);
        out.kind = MotionSetpoint::Kind::Pose;
        out.target.pos = {s.tx, s.ty};
        out.target.heading = phx::wrap_angle(s.ttheta);
        out.vel_max_xy = std::max(s.vel_max_xy, kSkillMinVelXY);
        out.vel_max_w = std::max(s.vel_max_w, kSkillMinVelW);
        out.acc_max_xy = std::max(s.acc_max_xy, kSkillMinAccXY);
        out.acc_max_w = s.acc_max_w;
        out.fast_pos = true;
        out.acc_max_xy_fast = std::max(s.acc_max_xy_fast, kSkillMinAccXY);
        out.kd = s.kd;
        return out;
    }
    case SkillId::LocalVel:
    case SkillId::GlobalVel: {
        const VelSkill s = decode_vel(mc.skill_data);
        out.kind = mc.skill_id == static_cast<int>(SkillId::LocalVel)
                       ? MotionSetpoint::Kind::LocalVel
                       : MotionSetpoint::Kind::GlobalVel;
        out.vel = phx::Twist{s.vx, s.vy, s.w};
        out.acc_max_xy = std::max(s.acc_max_xy, kSkillMinAccXY);
        out.acc_max_w_vel = std::max(s.acc_max_w, kSkillMinAccW);
        out.jerk_max_xy = std::max(s.jerk_max_xy, kSkillMinJerkXY);
        out.jerk_max_w = std::max(s.jerk_max_w, kSkillMinJerkW);
        out.kd = s.kd;
        return out;
    }
    case SkillId::WheelVel: {
        const WheelVelSkill s = decode_wheel_vel(mc.skill_data);
        out.kind = MotionSetpoint::Kind::WheelVel;
        out.wheel_rad_s = wheel_order_wire_to_can(s.wheel_rad_s);
        out.kd = s.kd;
        return out;
    }
    case SkillId::Sine: {
        const SineSkill s = decode_sine(mc.skill_data);
        out.kind = MotionSetpoint::Kind::Sine;
        out.sine_vx_amp = s.vx_amp;
        out.sine_vy_amp = s.vy_amp;
        out.sine_w_amp = s.w_amp;
        out.sine_freq_hz = s.freq_hz;
        // Sine carries no KD field — actuators stay disarmed.
        return out;
    }
    case SkillId::Emergency:
    default:
        // EMERGENCY, an unknown id, or one without a wire layout
        // (GlobalVelAndOrient): controlled ramp to zero. An undecodable
        // command must never produce arbitrary motion.
        out.kind = MotionSetpoint::Kind::Emergency;
        return out;
    }
}

}  // namespace rf
