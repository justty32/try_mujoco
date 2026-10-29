#pragma once
#include <mujoco/mujoco.h>

/* ---- lifecycle ---- */

/** Load XML model, allocate mjData, call sim_reset(). Exits on failure. */
void     sim_load(const char *path);

/** Free mjData and mjModel. Call at program exit. */
void     sim_unload(void);

/** Reset to the model home keyframe when present (otherwise qpos0),
 *  clear time/velocity, and run forward kinematics. */
void     sim_reset(void);

/** Advance physics by one timestep (m->opt.timestep, default 2 ms).
 *  Caller must set d->ctrl[] before calling. */
void     sim_step(void);

/* ---- accessors ---- */

/** Return the global mjModel pointer. Valid after sim_load(). */
mjModel *sim_model(void);

/** Return the global mjData pointer. Valid after sim_load(). */
mjData  *sim_data(void);

/* ---- [STATE MONITOR] joint ---- */

/** Current angle of a hinge joint, in degrees. Returns 0.0 if not found.
 *  Joints: "J1_base_yaw" "J2_shoulder_pitch" "J3_elbow_pitch"
 *  "J4_wrist_roll" "J5_wrist_pitch" "J6_tool_roll" */
double   sim_joint_angle_deg(const char *name);

/** Current angular velocity of a hinge joint, in degrees/s. */
double   sim_joint_vel_deg(const char *name);

/* ---- [STATE MONITOR] body position ---- */

/** Fill pos[3] with the world-frame Cartesian position of a body (x, y, z).
 *  Returns 0 on success, -1 if body name not found.
 *  Bodies: "base" "waist" "shoulder" "elbow" "wrist_roll" "wrist_pitch" "tool_roll" */
int      sim_body_pos(const char *name, double pos[3]);

/* ---- [STATE MONITOR] actuator force ---- */

/** Scalar actuator output, before transmission/gear scaling.
 *  For this model's hinge motors, joint torque = actuator_force * gear.
 *  index: 0=J1 base 1=J2 shoulder 2=J3 elbow 3=J4 roll 4=J5 pitch 5=J6 roll
 *  Returns 0.0 if index is out of range. */
double   sim_actuator_force(int index);
