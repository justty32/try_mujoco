#include "sim.h"
#include "controller.h"
#include "viewer.h"

#define MODEL_PATH    "assets/three_cylinders.xml"
#define SIM_PER_FRAME (1.0 / 60.0)

int main(void) {
    sim_load(MODEL_PATH);
    ctrl_init();
    if (!viewer_init(1200, 900, "MuJoCo - Three Cylinders"))
        return 1;

    while (!viewer_should_close()) {
        if (!viewer_is_paused()) {
            mjtNum start = sim_data()->time;
            while (sim_data()->time - start < SIM_PER_FRAME) {
                ctrl_apply();
                sim_step();
            }
        }
        viewer_render();
    }

    viewer_free();
    sim_unload();
    return 0;
}
