#pragma once

void ctrl_init(void);
void ctrl_set_phase(int phase);   /* 1=yaw  2=pitch  3=both */
int  ctrl_get_phase(void);
void ctrl_apply(void);            /* write current phase to d->ctrl */
void ctrl_set_motors(double yaw, double pitch);  /* manual override */
