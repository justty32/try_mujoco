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

/* ------------------------------------------------------------------ */
/* Input callbacks                                                      */
/* ------------------------------------------------------------------ */

static void cb_key(GLFWwindow *w, int key, int scan, int act, int mods) {
    if (act != GLFW_PRESS) return;
    switch (key) {
    case GLFW_KEY_ESCAPE:    glfwSetWindowShouldClose(w, GLFW_TRUE); break;
    case GLFW_KEY_SPACE:     paused ^= 1;                            break;
    case GLFW_KEY_BACKSPACE: sim_reset(); ctrl_apply();              break;
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
    snprintf(buf, bufsz,
        "Time: %.2f s%s\n"
        "Yaw joint:   %+7.2f deg  vel %+.2f deg/s\n"
        "Pitch joint: %+7.2f deg  vel %+.2f deg/s\n"
        "yaw_motor:   ctrl = %+.2f\n"
        "pitch_motor: ctrl = %+.2f\n"
        "\n"
        "[1] yaw  [2] pitch  [3] both\n"
        "[Space] %-6s  [Bksp] reset  [Esc] quit\n"
        "Drag: left=rotate  right=pan  scroll=zoom",
        d->time, paused ? "  PAUSED" : "",
        sim_joint_angle_deg("yaw_joint"),   sim_joint_vel_deg("yaw_joint"),
        sim_joint_angle_deg("pitch_joint"), sim_joint_vel_deg("pitch_joint"),
        d->ctrl[0], d->ctrl[1],
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

    char overlay[512];
    build_overlay(overlay, sizeof(overlay));
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, overlay, NULL, &con);

    glfwSwapBuffers(win);
    glfwPollEvents();
}
