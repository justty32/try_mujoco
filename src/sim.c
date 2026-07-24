#include "sim.h"
#include <stdio.h>
#include <stdlib.h>

#define R2D (180.0 / 3.14159265358979323846)

static mjModel *m = NULL;
static mjData  *d = NULL;

void sim_load(const char *path) {
    char error[1000] = "";
    m = mj_loadXML(path, NULL, error, sizeof(error));
    if (!m) { fprintf(stderr, "sim_load: %s\n", error); exit(1); }
    d = mj_makeData(m);
    sim_reset();
    printf("Loaded: %d bodies, %d joints, %d actuators\n",
           m->nbody, m->njnt, m->nu);
}

void sim_unload(void) {
    if (d) { mj_deleteData(d);  d = NULL; }
    if (m) { mj_deleteModel(m); m = NULL; }
}

void sim_reset(void) {
    mj_resetData(m, d);
    mj_forward(m, d);
}

void sim_step(void) {
    mj_step(m, d);
}

mjModel *sim_model(void) { return m; }
mjData  *sim_data(void)  { return d; }

double sim_joint_angle_deg(const char *name) {
    int j = mj_name2id(m, mjOBJ_JOINT, name);
    return (j < 0) ? 0.0 : d->qpos[m->jnt_qposadr[j]] * R2D;
}

double sim_joint_vel_deg(const char *name) {
    int j = mj_name2id(m, mjOBJ_JOINT, name);
    return (j < 0) ? 0.0 : d->qvel[m->jnt_dofadr[j]] * R2D;
}

int sim_body_pos(const char *name, double pos[3]) {
    int b = mj_name2id(m, mjOBJ_BODY, name);
    if (b < 0) return -1;
    pos[0] = d->xpos[b * 3 + 0];
    pos[1] = d->xpos[b * 3 + 1];
    pos[2] = d->xpos[b * 3 + 2];
    return 0;
}

double sim_actuator_force(int index) {
    if (index < 0 || index >= m->nu) return 0.0;
    return d->actuator_force[index];
}
