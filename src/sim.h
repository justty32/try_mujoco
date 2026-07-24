#pragma once
#include <mujoco/mujoco.h>

/* ---- lifecycle ---- */

/** Load XML model, allocate mjData, call sim_reset(). Exits on failure. */
void     sim_load(const char *path);

/** Free mjData and mjModel. Call at program exit. */
void     sim_unload(void);

/** Reset to initial state (qpos/qvel = 0, time = 0) and run forward kinematics. */
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
 *  Available joints: "yaw_joint" (±90°), "pitch_joint" (±60°) */
double   sim_joint_angle_deg(const char *name);

/** Current angular velocity of a hinge joint, in degrees/s. */
double   sim_joint_vel_deg(const char *name);

/* ---- [STATE MONITOR] body position ---- */

/** Fill pos[3] with the world-frame Cartesian position of a body (x, y, z).
 *  Returns 0 on success, -1 if body name not found.
 *  Available bodies: "base_cylinder", "top_cylinder", "pitch_cylinder" */
int      sim_body_pos(const char *name, double pos[3]);

/* ---- [STATE MONITOR] actuator force ---- */

/** Force currently output by an actuator (after gear ratio).
 *  index: 0 = yaw_motor, 1 = pitch_motor.
 *  Returns 0.0 if index is out of range. */
double   sim_actuator_force(int index);
