#include "viewer.h"
#include "sim.h"
#include "controller.h"
#include <stdio.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

static GLFWwindow *win = NULL;
static mjvCamera   cam;
static mjvOption   opt;
static mjvScene    scn;
static mjrContext  con;

static int    paused    = 0;
static int    btn_left  = 0;
static int    btn_mid   = 0;
static int    btn_right = 0;
static double lastx     = 0;
static double lasty     = 0;
static int    selected_joint = 0;

/* ------------------------------------------------------------------ */
/* Input callbacks                                                      */
/* ------------------------------------------------------------------ */

static void cb_key(GLFWwindow *w, int key, int scan, int act, int mods) {
    (void)scan;
    if (act != GLFW_PRESS && act != GLFW_REPEAT) return;

    /* Repeating arrows make small angle adjustments practical. */
    if (key == GLFW_KEY_UP || key == GLFW_KEY_DOWN) {
        int direction = key == GLFW_KEY_UP ? -1 : 1;
        selected_joint = (selected_joint + direction + CTRL_JOINT_COUNT)
                         % CTRL_JOINT_COUNT;
        return;
    }
    if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT) {
        if (ctrl_get_mode() != CTRL_MODE_POSE) ctrl_hold_current();
        double step = (mods & GLFW_MOD_SHIFT) ? 1.0 : 5.0;
        if (key == GLFW_KEY_LEFT) step = -step;
        ctrl_set_target_deg(selected_joint,
                            ctrl_get_target_deg(selected_joint) + step);
        return;
    }
    if (act != GLFW_PRESS) return;
    if (key >= GLFW_KEY_F1 && key <= GLFW_KEY_F6) {
        selected_joint = key - GLFW_KEY_F1;
        return;
    }
    switch (key) {
    case GLFW_KEY_ESCAPE:    glfwSetWindowShouldClose(w, GLFW_TRUE); break;
    case GLFW_KEY_SPACE:     paused ^= 1;                            break;
    case GLFW_KEY_BACKSPACE:
        sim_reset(); ctrl_zero_targets(); ctrl_apply();             break;
    case GLFW_KEY_P:         ctrl_hold_current();                   break;
    case GLFW_KEY_HOME:      ctrl_zero_targets();                   break;
    case GLFW_KEY_1:         ctrl_set_phase(1);                      break;
    case GLFW_KEY_2:         ctrl_set_phase(2);                      break;
    case GLFW_KEY_3:         ctrl_set_phase(3);                      break;
    }
}

static void cb_mouse_button(GLFWwindow *w, int btn, int act, int mods) {
    btn_left  = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    btn_mid   = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    btn_right = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    glfwGetCursorPos(w, &lastx, &lasty);
}

static void cb_mouse_move(GLFWwindow *w, double xpos, double ypos) {
    if (!btn_left && !btn_mid && !btn_right) return;
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(w, &width, &height);

    int shift = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                 glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    mjtMouse action;
    if (btn_right)     action = shift ? mjMOUSE_MOVE_H   : mjMOUSE_MOVE_V;
    else if (btn_left) action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else               action = mjMOUSE_ZOOM;

    mjv_moveCamera(sim_model(), action, dx / height, dy / height, &scn, &cam);
}

static void cb_scroll(GLFWwindow *w, double xoff, double yoff) {
    mjv_moveCamera(sim_model(), mjMOUSE_ZOOM, 0, -0.05 * yoff, &scn, &cam);
}

/* ------------------------------------------------------------------ */
/* HUD                                                                  */
/* ------------------------------------------------------------------ */

static void build_overlay(char *buf, int bufsz) {
    mjData *d = sim_data();
    int pose = ctrl_get_mode() == CTRL_MODE_POSE;
    char rows[1024] = "";
    int used = 0;
    for (int i = 0; i < CTRL_JOINT_COUNT; ++i) {
        const char *name = ctrl_joint_name(i);
        int count = snprintf(rows + used, sizeof(rows) - (size_t)used,
            "%c F%d %-6s target %+6.1f  actual %+6.1f deg"
            "  vel %+6.1f  ctrl %+.2f\n",
            i == selected_joint ? '>' : ' ', i + 1, name,
            ctrl_get_target_deg(i), sim_joint_angle_deg(name),
            sim_joint_vel_deg(name), d->ctrl[i]);
        if (count < 0 || (size_t)count >= sizeof(rows) - (size_t)used) break;
        used += count;
    }
    snprintf(buf, bufsz,
        "Time: %.2f s%s | %s\n"
        "%s\n"
        "%s"
        "Selected: %s  target range [%+.0f, %+.0f] deg\n"
        "\n"
        "[F1-F6 / Up Down] select joint\n"
        "[Left Right] target -/+5 deg  [Shift] -/+1 deg\n"
        "[P] hold current pose  [Home] zero targets\n"
        "[1] force yaw  [2] force pitch  [3] force reverse\n"
        "[Space] %-6s  [Bksp] reset  [Esc] quit\n"
        "Drag: left=rotate  right=pan  scroll=zoom",
        d->time, paused ? "  PAUSED" : "",
        pose ? "POSE HOLD" : "FORCE PRESET (targets inactive)",
        pose ? "Target ramp: 30 deg/s | pitch1 torque limit: +/-40 Nm"
             : "Original motor limits | press P to hold current pose",
        rows, ctrl_joint_name(selected_joint),
        ctrl_get_min_deg(selected_joint), ctrl_get_max_deg(selected_joint),
        paused ? "resume" : "pause"
    );
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

int viewer_init(int width, int height, const char *title) {
    if (!glfwInit()) { fprintf(stderr, "glfwInit failed\n"); return 0; }

    win = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!win) {
        fprintf(stderr, "glfwCreateWindow failed\n");
        glfwTerminate();
        return 0;
    }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    glfwSetKeyCallback(win,         cb_key);
    glfwSetMouseButtonCallback(win, cb_mouse_button);
    glfwSetCursorPosCallback(win,   cb_mouse_move);
    glfwSetScrollCallback(win,      cb_scroll);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(sim_model(), &scn, 2000);
    mjr_makeContext(sim_model(), &con, mjFONTSCALE_150);

    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.4;
    cam.distance  = 2.0;
    cam.azimuth   = 135.0;
    cam.elevation = -20.0;
    return 1;
}

void viewer_free(void) {
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    if (win) { glfwDestroyWindow(win); win = NULL; }
    glfwTerminate();
}

int viewer_should_close(void) { return glfwWindowShouldClose(win); }
int viewer_is_paused(void)    { return paused; }

void viewer_render(void) {
    mjrRect vp = {0, 0, 0, 0};
    glfwGetFramebufferSize(win, &vp.width, &vp.height);

    mjv_updateScene(sim_model(), sim_data(), &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(vp, &scn, &con);

    char overlay[2048];
    build_overlay(overlay, sizeof(overlay));
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, overlay, NULL, &con);

    glfwSwapBuffers(win);
    glfwPollEvents();
}
