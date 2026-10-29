#include "controller.h"
#include "sim.h"
#include <math.h>
#include <stdio.h>

#define D2R (3.14159265358979323846 / 180.0)
#define REFERENCE_SPEED (30.0 * D2R)

static const char *JOINTS[CTRL_JOINT_COUNT] = {
    "yaw1", "pitch1", "yaw2", "pitch2", "yaw3", "pitch3"
};
static const char *MOTORS[CTRL_JOINT_COUNT] = {
    "yaw1_motor", "pitch1_motor", "yaw2_motor", "pitch2_motor", "yaw3_motor", "pitch3_motor"
};
static const double PHASES[3][CTRL_JOINT_COUNT] = {
    { 1.0, 0.0, 1.0, 0.0, 1.0, 0.0 },
    { 0.0, 0.5, 0.0, 0.5, 0.0, 0.5 },
    {-0.8,-0.3,-0.8,-0.3,-0.8,-0.3 }
};
/* Acceleration-space PD: common 8 rad/s natural frequency and critical damping. */
#define POSE_KP 64.0
#define POSE_KD 16.0
static int phase = 1, mode = CTRL_MODE_PHASE, ready;
static mjModel *bound_model;
static int joint_id[CTRL_JOINT_COUNT], motor_id[CTRL_JOINT_COUNT];
static double target[CTRL_JOINT_COUNT], reference[CTRL_JOINT_COUNT];
static double saved_range[CTRL_JOINT_COUNT][2];
static int saved_limited[CTRL_JOINT_COUNT];

static double clamp(double value, double low, double high) {
    return fmax(low, fmin(high, value));
}
static int valid_joint(int i) { return ready && i >= 0 && i < CTRL_JOINT_COUNT; }

/* Only pose mode needs a stronger shoulder motor: at zero posture gravity
 * requires ~20 Nm but the original pitch1 limit is 10 Nm (gear=10, ctrl=1).
 * All old presets retain both their original commands and original limits. */
static void set_limits(int pose) {
    mjModel *m = sim_model();
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) {
        int a = motor_id[i];
        m->actuator_ctrllimited[a] = saved_limited[i];
        m->actuator_ctrlrange[2*a] = saved_range[i][0];
        m->actuator_ctrlrange[2*a+1] = saved_range[i][1];
        if (pose && i == 1) {
            m->actuator_ctrllimited[a] = 1;
            m->actuator_ctrlrange[2*a] = -4;
            m->actuator_ctrlrange[2*a+1] = 4;
        }
    }
}

void ctrl_init(void) {
    mjModel *m = sim_model();
    if (ready && bound_model == m) set_limits(0);
    ready = 0;
    bound_model = m;
    phase = 1;
    mode = CTRL_MODE_PHASE;
    if (m->nv != CTRL_JOINT_COUNT) {
        fprintf(stderr, "ctrl_init: expected six hinge degrees of freedom\n");
        return;
    }
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) {
        int j = mj_name2id(m, mjOBJ_JOINT, JOINTS[i]);
        int a = mj_name2id(m, mjOBJ_ACTUATOR, MOTORS[i]);
        if (j < 0 || a < 0 || m->jnt_type[j] != mjJNT_HINGE ||
            m->actuator_trntype[a] != mjTRN_JOINT || m->actuator_trnid[2*a] != j ||
            !isfinite(m->actuator_gear[6*a]) || fabs(m->actuator_gear[6*a]) < 1e-12) {
            fprintf(stderr, "ctrl_init: missing/incompatible hinge motor %s\n", MOTORS[i]);
            return;
        }
        joint_id[i] = j;
        motor_id[i] = a;
        saved_range[i][0] = m->actuator_ctrlrange[2*a];
        saved_range[i][1] = m->actuator_ctrlrange[2*a+1];
        saved_limited[i] = m->actuator_ctrllimited[a];
    }
    ready = 1;
    ctrl_hold_current();
    ctrl_apply();
}

void ctrl_set_phase(int p) {
    if (!ready || p < 1 || p > 3) return;
    phase = p;
    mode = CTRL_MODE_PHASE;
    set_limits(0);
}
int ctrl_get_phase(void) { return phase; }
int ctrl_get_mode(void) { return mode; }
const char *ctrl_joint_name(int i) {
    return i >= 0 && i < CTRL_JOINT_COUNT ? JOINTS[i] : "?";
}
double ctrl_get_min_deg(int i) {
    return valid_joint(i) ? sim_model()->jnt_range[2*joint_id[i]] / D2R : 0;
}
double ctrl_get_max_deg(int i) {
    return valid_joint(i) ? sim_model()->jnt_range[2*joint_id[i]+1] / D2R : 0;
}
double ctrl_get_target_deg(int i) { return valid_joint(i) ? target[i] / D2R : 0; }

void ctrl_hold_current(void) {
    if (!ready) return;
    mjModel *m = sim_model();
    mjData *d = sim_data();
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) {
        int j = joint_id[i];
        reference[i] = d->qpos[m->jnt_qposadr[j]];
        target[i] = clamp(reference[i], m->jnt_range[2*j], m->jnt_range[2*j+1]);
    }
    mode = CTRL_MODE_POSE;
    set_limits(1);
}
int ctrl_set_target_deg(int i, double degrees) {
    if (!valid_joint(i) || !isfinite(degrees)) return -1;
    if (mode != CTRL_MODE_POSE) ctrl_hold_current();
    target[i] = clamp(degrees, ctrl_get_min_deg(i), ctrl_get_max_deg(i)) * D2R;
    return 0;
}
int ctrl_set_pose_deg(const double degrees[CTRL_JOINT_COUNT]) {
    if (!ready || !degrees) return -1;
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i)
        if (!isfinite(degrees[i])) return -1;
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) ctrl_set_target_deg(i, degrees[i]);
    return 0;
}
void ctrl_zero_targets(void) {
    ctrl_hold_current();
    const double zero[CTRL_JOINT_COUNT] = {0};
    ctrl_set_pose_deg(zero);
}

void ctrl_apply(void) {
    if (!ready) return;
    mjModel *m = sim_model();
    mjData *d = sim_data();
    if (mode == CTRL_MODE_PHASE) {
        for (int i = 0; i < CTRL_JOINT_COUNT; ++i)
            d->ctrl[motor_id[i]] = PHASES[phase-1][i];
        return;
    }
    /* Refresh qfrc_bias at the current state; it includes gravity + Coriolis.
     * We command actual motors, never teleport qpos or add external forces. */
    mj_forward(m, d);
    double step = REFERENCE_SPEED * m->opt.timestep;
    mjtNum acceleration[CTRL_JOINT_COUNT] = {0};
    mjtNum torque[CTRL_JOINT_COUNT];
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) {
        int j = joint_id[i], dof = m->jnt_dofadr[j];
        reference[i] += clamp(target[i] - reference[i], -step, step);
        double error = reference[i] - d->qpos[m->jnt_qposadr[j]];
        acceleration[dof] = POSE_KP*error - POSE_KD*d->qvel[dof];
    }
    /* Inertia scaling keeps the light distal joints from receiving excessive
     * explicit damping. Compensate model damping, but never contact forces. */
    mj_mulM(m, d, torque, acceleration);
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) {
        int j = joint_id[i], a = motor_id[i], dof = m->jnt_dofadr[j];
        double u = (torque[dof] + d->qfrc_bias[dof] - d->qfrc_passive[dof]) /
                   m->actuator_gear[6*a];
        d->ctrl[a] = m->actuator_ctrllimited[a]
            ? clamp(u, m->actuator_ctrlrange[2*a], m->actuator_ctrlrange[2*a+1]) : u;
    }
}

void ctrl_set_motors(const double ctrl[CTRL_JOINT_COUNT]) {
    if (!ready || !ctrl) return;
    mjData *d = sim_data();
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) d->ctrl[motor_id[i]] = ctrl[i];
}
