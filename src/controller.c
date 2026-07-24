#include "controller.h"
#include "sim.h"

static int phase = 1;

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
    switch (phase) {
    case 1: d->ctrl[0] =  1.0;  d->ctrl[1] =  0.0;  break;
    case 2: d->ctrl[0] =  0.0;  d->ctrl[1] =  0.5;  break;
    case 3: d->ctrl[0] = -0.8;  d->ctrl[1] = -0.3;  break;
    }
}

void ctrl_set_motors(double yaw, double pitch) {
    mjData *d = sim_data();
    d->ctrl[0] = yaw;
    d->ctrl[1] = pitch;
}
