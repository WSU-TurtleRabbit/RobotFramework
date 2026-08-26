# Adaptive motion architecture

This system is software-only. It changes no motor, Moteus firmware or
persistent Moteus configuration, encoder, wheel, gearing, chassis, battery,
sensor, wiring, or weight distribution. The existing deterministic controller
and emergency paths remain the fallback.

## Runtime path and ownership

At 250 Hz, `MatchBridge` validates the newest MatchCtrl packet, inserts delayed
SSL-Vision observations into estimator history, regenerates the existing
trajectory, and calls the stable Panthera-style controller. `MotionAugmentor`
then runs the following pure, allocation-free sequence:

```
trajectory -> stable controller -> robust RLS context
           -> bounded adaptive correction -> residual actor proposal
           -> deterministic constraints -> calibrated wheel kinematics
           -> existing Pi3Hat/Moteus velocity transaction
```

The stable output is computed before augmentation. Emergency and direct wheel
skills bypass adaptation. Missing, incompatible, unhealthy, late, stale, or
non-finite learned state produces the stable output. The learner has no socket,
clock, file watcher, optimizer, or motor interface.

The Pi 4 only runs twelve two-parameter RLS buckets and, when enabled, one
32-input/single-hidden-layer/3-output actor. The asynchronous JSONL writer uses
a fixed 512-sample queue. TD3 updates, replay, reports and policy selection run
offboard.

## Surface and drivetrain identification

The estimator fits, independently for body X, body Y and yaw and for
positive/negative drive/braking buckets,

```
dv/dt = response * (command - velocity) - rolling_drag * velocity
```

RLS uses a configurable forgetting factor, Huber residual weighting, bounded
covariance and physical parameter ranges. It scores delayed command history to
identify 0--60 ms actuation delay. Wheel odometry versus fused velocity gives
signed longitudinal, lateral and yaw slip; bus voltage yields a bounded
battery-response scale.

Samples are rejected for stale/discontinuous vision, excessive acceleration,
Moteus saturation/fault/current/temperature, missed deadlines, kicks,
dribbler activity, detected collision, bad `dt`, or insufficient excitation.
Parameter updates are bounded per sample. Confidence combines excitation,
covariance and age; stale confidence decays and parameters move toward safe
defaults. Corrections require confidence and are limited to 8% by default.

Each persisted profile includes `robot_id` and `surface_id`; identity mismatch
fails closed. Fixed wheel geometry is absent from the profile. Saved
steady-clock timestamps are never trusted after reboot: parameters can seed a
new session, but fresh motion must rebuild confidence. Replacement preserves a
`.previous` profile for recovery.

## Residual modes and constraints

| Mode | Identifier | Actor evaluated | Actor applied |
|---|---|---|---|
| `off` | configurable | no | no |
| `collect` | yes | no | no |
| `shadow` | yes | yes | no |
| `bounded` | yes | yes | only while all gates are healthy |

The actor ABI is `turtlerabbit-motion-v1`: 32 SI observations, one tanh hidden
layer of at most 32 units, and three tanh actions. A strict loader checks schema,
ABI, dimensions, finite tensors, normalization, version and declared residual
limit. The initial limit is 5% and cannot be enlarged by a policy file.

Bounded application additionally requires fresh vision, estimator confidence,
all four Moteus replies, healthy current/voltage/temperature/faults, and an
on-time control loop. Residual slew, combined-correction bound and jerk,
skill/global velocity, angular velocity, calibrated wheel speed and finite
checks run after inference. Repeated RL-related constraint interventions latch
RL off. The deterministic controller remains active and can be restored
immediately by setting `rl.mode: off`.

## Telemetry schema v1

`AsyncMotionLogger` writes one schema header followed by keyed JSONL samples in
SI units using steady monotonic seconds. The header declares global/body frames
and `[FR, RR, RL, FL]` wheel order. Samples include:

- target, reference, fused pose/velocity and command age;
- stable, adaptive, proposed/applied residual and final safety output;
- target/measured wheel velocity and wheel position;
- q-current, bus voltage, temperature, mode, fault and reply availability;
- raw Pi3Hat rate/acceleration plus body acceleration only when its axis mapping
  is explicitly configured;
- vision age/delay/innovation and explicit confidence availability;
- every estimated context value, confidence, coverage and sample counts;
- mode, policy version/health, interventions, saturation, deadline, kick,
  dribbler and collision flags.

Pi3Hat acceleration axes remain disabled in the shipped configuration because
the repository does not establish the chassis mapping. Wheel/vision dynamics
still support identification; an operator may configure axes only after a
stationary gravity and signed-motion validation.

## Offline learning and promotion

`testing/rlearn/rlearn/residual_env.py` randomizes wheel radius and angle,
mass/inertia, anisotropic/directional friction, rolling resistance, carpet
transitions, motor gain, battery, command delay/jitter, vision noise/delay/
dropout, IMU bias/noise, encoder scale, thermal response and external
disturbances. These are conservative priors, not transfer claims.

TD3 is used because the action is small and continuous, the deployed actor
should be deterministic, and twin critics/target smoothing/delayed actor
updates are sample-efficient without an entropy mechanism onboard. Exploration
noise exists only offboard. Reward separately penalizes cross-track and heading
error, velocity/arrival/stop/corner error, slip, action smoothness, jerk,
current/energy proxy, saturation and safety interventions.

The replay buffer stratifies surface/direction and mixes mostly uniform with
capped priority sampling. Disturbed cases are retained but excluded from normal
updates. Versioned real logs can be ingested explicitly.

Training atomically saves an improving simulator challenger but never authorizes
deployment. Lifecycle promotion requires at least three simulation seeds, two
surface conditions, clean log replay and 1,000 real-runtime shadow samples, as
well as cross-track improvement and no safety, stopping, heading or wheel-
tracking regression. Promotion atomically swaps the local champion and keeps
the prior champion for rollback. Export does not edit runtime mode or contact a
robot.

## Known limitations

- MatchCtrl carries vision age but not detector confidence. The schema records
  `confidence_available=false`; age and innovation gates are authoritative.
- MatchFeedback does not carry per-wheel/current telemetry. Physical benchmark
  JSON must be joined with onboard schema-v1 telemetry by run timing for those
  metrics.
- Simulation fidelity is intentionally modest. No RL policy is accepted from
  simulator reward alone, and no current candidate is physically validated.
- The 4 m/s X/Y and 6/5 m/s² reference targets are report defaults only. The
  physical runbook begins at 0.5 m/s and 0.5 m/s².
