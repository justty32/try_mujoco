#pragma once

void ctrl_init(void);
void ctrl_set_phase(int phase);   /* 1=all yaw  2=all pitch  3=all reverse */
int  ctrl_get_phase(void);
void ctrl_apply(void);            /* write current phase to d->ctrl[0..5] */
/* Direct write for all 6 motors; the next ctrl_apply() overwrites these values. */
void ctrl_set_motors(const double ctrl[6]);
