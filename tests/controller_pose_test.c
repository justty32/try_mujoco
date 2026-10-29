#include <math.h>
#include <stdio.h>
#include "controller.h"
#include "sim.h"

#define CHECK(c, label) do { if (!(c)) { fprintf(stderr, "FAIL: %s\n", label); ok = 0; } } while (0)

static int saw_contact;

static int advance(double seconds) {
    mjModel *m = sim_model();
    mjData *d = sim_data();
    int steps = (int)(seconds / m->opt.timestep + 0.5);
    for (int n = 0; n < steps; ++n) {
        ctrl_apply();
        for (int a = 0; a < m->nu; ++a)
            if (!isfinite(d->ctrl[a]) ||
                (m->actuator_ctrllimited[a] &&
                 (d->ctrl[a] < m->actuator_ctrlrange[2*a] - 1e-10 ||
                  d->ctrl[a] > m->actuator_ctrlrange[2*a+1] + 1e-10))) return 0;
        sim_step();
        if (d->ncon > 0) saw_contact = 1;
        for (int q = 0; q < m->nq; ++q) if (!isfinite(d->qpos[q])) return 0;
        for (int v = 0; v < m->nv; ++v) if (!isfinite(d->qvel[v])) return 0;
        for (int w = 0; w < mjNWARNING; ++w) if (d->warning[w].number) return 0;
    }
    return 1;
}
static int settled(const char *label, double tolerance) {
    double error = 0, speed = 0;
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) {
        error = fmax(error, fabs(sim_joint_angle_deg(ctrl_joint_name(i)) - ctrl_get_target_deg(i)));
        speed = fmax(speed, fabs(sim_joint_vel_deg(ctrl_joint_name(i))));
    }
    printf("%s: max error %.6f deg, speed %.6f deg/s\n", label, error, speed);
    if (error >= tolerance || speed >= 1.0) {
        for (int i = 0; i < CTRL_JOINT_COUNT; ++i)
            printf("  %s target=%g actual=%g ctrl=%g\n", ctrl_joint_name(i),
                   ctrl_get_target_deg(i), sim_joint_angle_deg(ctrl_joint_name(i)),
                   sim_data()->ctrl[i]);
        printf("  contacts=%d\n", sim_data()->ncon);
    }
    return error < tolerance && speed < 1.0;
}
int main(int argc, char **argv) {
    if (argc != 2) { fprintf(stderr, "usage: controller_pose_test MODEL.xml\n"); return 2; }
    sim_load(argv[1]);
    ctrl_init();
    int ok = 1;
    mjModel *m = sim_model();
    mjData *d = sim_data();
    const double home[6] = {0,65,-100,0,35,0};
    for (int i=0; i<6; ++i) CHECK(fabs(sim_joint_angle_deg(ctrl_joint_name(i))-home[i])<1e-10, "startup uses radian home key");
    int shoulder = mj_name2id(m, mjOBJ_ACTUATOR, "J2_motor");
    CHECK(ctrl_get_mode() == CTRL_MODE_POSE, "startup pose mode");
    CHECK(m->actuator_ctrlrange[2*shoulder+1] == 1, "pose preserves original shoulder capacity");
    CHECK(advance(3.0), "startup finite/limited");
    CHECK(settled("home hold", 0.25), "home posture resists gravity");

    ctrl_set_target_deg(4,45);
    CHECK(advance(3.0), "wrist upper travel finite/limited");
    CHECK(settled("wrist +45",0.5), "wrist reaches usable travel boundary");
    ctrl_home_targets();
    CHECK(advance(3.0), "home after wrist finite/limited");

    const double pose[6] = {20, 50, -85, 25, 25, -40};
    CHECK(ctrl_set_pose_deg(pose) == 0, "set six-joint pose");
    CHECK(advance(0.1), "first ramp segment finite");
    for (int i = 0; i < 6; ++i)
        CHECK(fabs(sim_joint_angle_deg(ctrl_joint_name(i))-home[i]) < 5.0, "target change does not teleport joint");
    CHECK(advance(6.0), "combined target finite/limited");
    CHECK(settled("combined target", 0.5), "combined target converges");
    CHECK(advance(3.0), "continued hold finite/limited");
    CHECK(settled("continued hold", 0.5), "combined target stays held");
    const double second_pose[6] = {-25, 80, -110, -30, 20, 60};
    ctrl_set_pose_deg(second_pose);
    CHECK(advance(6.0), "second pose finite/limited");
    CHECK(settled("second target", 0.5), "different shoulder/elbow/wrist target converges");

    double before[6];
    for (int i = 0; i < 6; ++i) before[i] = ctrl_get_target_deg(i);
    double invalid[6] = {0, 0, NAN, 0, 0, 0};
    CHECK(ctrl_set_pose_deg(invalid) == -1, "nonfinite pose rejected");
    CHECK(ctrl_set_target_deg(0, INFINITY) == -1, "infinite target rejected");
    CHECK(ctrl_set_target_deg(-1, 0) == -1 && ctrl_set_target_deg(6, 0) == -1, "invalid joint rejected");
    for (int i = 0; i < 6; ++i)
        CHECK(ctrl_get_target_deg(i) == before[i], "invalid pose is atomic");
    for (int i = 0; i < 6; ++i) {
        CHECK(ctrl_set_target_deg(i, 10000) == 0, "upper target accepted with clamp");
        CHECK(fabs(ctrl_get_target_deg(i) - ctrl_get_max_deg(i)) < 1e-10, "upper joint limit");
        ctrl_set_target_deg(i, -10000);
        CHECK(fabs(ctrl_get_target_deg(i) - ctrl_get_min_deg(i)) < 1e-10, "lower joint limit");
    }
    ctrl_set_pose_deg(before);

    const double phases[3][6] = {
        {.25,0,0,.10,0,.10}, {0,.35,.25,0,.10,0}, {-.20,-.25,-.20,-.10,-.10,-.10}
    };
    for (int p = 1; p <= 3; ++p) {
        ctrl_set_phase(p);
        ctrl_apply();
        CHECK(ctrl_get_mode() == CTRL_MODE_PHASE, "legacy mode selected");
        for (int i = 0; i < 6; ++i) {
            CHECK(fabs(d->ctrl[i] - phases[p-1][i]) < 1e-12, "new mechanism torque preset mapping");
            CHECK(m->actuator_ctrlrange[2*i] == -1 && m->actuator_ctrlrange[2*i+1] == 1,
                  "motor limits consistent across modes");
        }
    }
    ctrl_set_phase(1);
    CHECK(advance(0.05), "legacy torque short motion");
    double captured[6];
    for (int i = 0; i < 6; ++i) captured[i] = sim_joint_angle_deg(ctrl_joint_name(i));
    ctrl_hold_current();
    for (int i = 0; i < 6; ++i)
        CHECK(fabs(ctrl_get_target_deg(i) - captured[i]) < 1e-10, "capture actual posture");
    CHECK(advance(5.0), "capture/hold finite/limited");
    CHECK(settled("capture current", 0.5), "captured posture converges");

    ctrl_zero_targets();
    CHECK(advance(10.0), "zero target finite/limited");
    CHECK(settled("return to zero", 0.5), "explicit zero API converges");
    double home_time=d->time, home_q=d->qpos[1];
    ctrl_home_targets();
    CHECK(d->time==home_time && d->qpos[1]==home_q, "Home does not reset time or teleport");
    CHECK(advance(7.0), "Home finite/limited");
    CHECK(settled("return home",0.5), "Home converges");
    for (int i=0;i<6;++i) CHECK(fabs(ctrl_get_target_deg(i)-home[i])<1e-10,"Home key targets");
    ctrl_set_pose_deg(pose);
    CHECK(advance(1.0), "move before reset");
    sim_reset();
    ctrl_home_targets();
    CHECK(d->time==0, "reset clears simulation time");
    for (int i=0;i<6;++i) CHECK(fabs(sim_joint_angle_deg(ctrl_joint_name(i))-home[i])<1e-10,"reset returns home");
    CHECK(advance(2.0), "reset finite/limited");
    CHECK(settled("reset hold", 0.25), "reset clears ramp and keeps home");
    /* This target can collide with another arm segment. It is not promised
     * to converge, but must not disable contacts, exceed limits, or go NaN. */
    const double blocked_pose[6] = {0, -60, 0, 0, 0, 0};
    saw_contact = 0;
    ctrl_set_pose_deg(blocked_pose);
    CHECK(advance(8.0), "contact-blocked target remains finite and torque limited");
    CHECK(!(m->opt.disableflags & mjDSBL_CONTACT), "pose mode preserves contacts");
    CHECK(saw_contact, "unreachable downward posture encounters the base/floor");
    const int flags[] = {mjDSBL_DAMPER, mjDSBL_EULERDAMP};
    for (int i = 0; i < 2; ++i) {
        int original = m->opt.disableflags;
        m->opt.disableflags |= flags[i];
        sim_reset();
        ctrl_home_targets();
        ctrl_set_pose_deg(pose);
        CHECK(advance(4.0), "damping option finite/limited");
        CHECK(settled("damping option", 0.5), "Euler compensation follows damping option");
        m->opt.disableflags = original;
    }
    for (int i = 0; i < mjNWARNING; ++i) CHECK(d->warning[i].number == 0, "no MuJoCo warning");
    printf("controller_pose: %s\n", ok ? "ok" : "FAILED");
    sim_unload();
    return ok ? 0 : 1;
}
