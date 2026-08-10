#include "controller.h"
#include "sim.h"

static int phase = 1;

/* ctrl[0..5] = yaw1, pitch1, yaw2, pitch2, yaw3, pitch3 */
static const double PHASES[3][6] = {
    {  1.0,  0.0,  1.0,  0.0,  1.0,  0.0 },  /* 1: all yaw          */
    {  0.0,  0.5,  0.0,  0.5,  0.0,  0.5 },  /* 2: all pitch        */
    { -0.8, -0.3, -0.8, -0.3, -0.8, -0.3 },  /* 3: all reverse      */
};

void ctrl_init(void) {
    phase = 1;
    ctrl_apply();
}

void ctrl_set_phase(int p) {
    if (p >= 1 && p <= 3) phase = p;
}

int ctrl_get_phase(void) {
    return phase;
}

void ctrl_apply(void) {
    mjData *d = sim_data();
    const double *p = PHASES[phase - 1];
    for (int i = 0; i < 6; i++) d->ctrl[i] = p[i];
}

void ctrl_set_motors(const double ctrl[6]) {
    mjData *d = sim_data();
    for (int i = 0; i < 6; i++) d->ctrl[i] = ctrl[i];
}
