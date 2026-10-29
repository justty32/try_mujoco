#pragma once

enum { CTRL_JOINT_COUNT = 6 };
enum CtrlMode { CTRL_MODE_PHASE, CTRL_MODE_POSE };

/* Call after sim_load(); starts in pose mode, holding the current angles. */
void ctrl_init(void);
/* Legacy torque presets; all modes keep the model's motor control limits. */
void ctrl_set_phase(int phase);   /* 1=base/roll  2=shoulder/elbow/wrist pitch  3=reverse */
int  ctrl_get_phase(void);
int  ctrl_get_mode(void);
void ctrl_apply(void);            /* call once before every sim_step() */
/* Direct write; next ctrl_apply() overwrites it, using current mode limits. */
void ctrl_set_motors(const double ctrl[CTRL_JOINT_COUNT]);

/* Joint order: J1 base yaw, J2 shoulder pitch, J3 elbow pitch,
 * J4 wrist roll, J5 wrist pitch, J6 tool roll. Angles are degrees.
 * Setters reject non-finite input, clamp to MJCF joint limits, and enter pose
 * mode. Return 0 on success, -1 for invalid input/uninitialized controller. */
const char *ctrl_joint_name(int joint);
double ctrl_get_target_deg(int joint);
double ctrl_get_min_deg(int joint);
double ctrl_get_max_deg(int joint);
int ctrl_set_target_deg(int joint, double degrees);
int ctrl_set_pose_deg(const double degrees[CTRL_JOINT_COUNT]);
void ctrl_hold_current(void);
/* Slew toward the model home keyframe; does not teleport or reset time. */
void ctrl_home_targets(void);
/* Explicit all-zero pose, distinct from the folded home posture. */
void ctrl_zero_targets(void);
