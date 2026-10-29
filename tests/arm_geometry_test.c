#include <math.h>
#include <stdio.h>
#include <mujoco/mujoco.h>

static const char *JOINTS[6] = {"yaw1", "pitch1", "yaw2", "pitch2", "yaw3", "pitch3"};

static int connected(mjModel *m, mjData *d, const char *first, const char *second) {
    int a = mj_name2id(m, mjOBJ_GEOM, first), b = mj_name2id(m, mjOBJ_GEOM, second);
    if (a < 0 || b < 0) {
        fprintf(stderr, "missing assembly part: %s / %s\n", first, second);
        return 0;
    }
    double gap = mj_geomDistance(m, d, a, b, 0.05, NULL);
    if (gap > 1e-6) {
        fprintf(stderr, "floating assembly: %s / %s gap %.6g m\n", first, second, gap);
        return 0;
    }
    return 1;
}

static int place(mjModel *m, mjData *d, const double degrees[6]) {
    mj_resetData(m, d);
    for (int i = 0; i < 6; ++i) {
        int j = mj_name2id(m, mjOBJ_JOINT, JOINTS[i]);
        if (j < 0) return 0;
        d->qpos[m->jnt_qposadr[j]] = degrees[i] * 3.14159265358979323846 / 180;
    }
    mj_forward(m, d);
    return 1;
}

/* Unlike d->ncon, explicit pair distances also inspect parent-child bodies.
 * Same-body solids may overlap to join primitives into a rigid assembly. */
static int clear_pose(mjModel *m, mjData *d, const double degrees[6], const char *label) {
    if (!place(m, d, degrees)) return 0;
    for (int a = 0; a < m->ngeom; ++a) {
        for (int b = a+1; b < m->ngeom; ++b) {
            if (m->geom_bodyid[a] == m->geom_bodyid[b]) continue;
            double distance = mj_geomDistance(m, d, a, b, 0.05, NULL);
            if (!isfinite(distance) || distance < -1e-6) {
                fprintf(stderr, "%s intersects: %s / %s (%.6g m)\n", label,
                        mj_id2name(m, mjOBJ_GEOM, a), mj_id2name(m, mjOBJ_GEOM, b), distance);
                return 0;
            }
        }
    }
    return 1;
}

int main(int argc, char **argv) {
    if (argc != 2) { fprintf(stderr, "usage: arm_geometry_test MODEL.xml\n"); return 2; }
    char error[1024] = "";
    mjModel *m = mj_loadXML(argv[1], NULL, error, sizeof(error));
    if (!m) { fprintf(stderr, "load: %s\n", error); return 1; }
    mjData *d = mj_makeData(m);
    if (!d) { mj_deleteModel(m); return 1; }
    int ok = m->nexclude == 0 && !(m->opt.disableflags & mjDSBL_CONTACT);
    if (!ok) fprintf(stderr, "model disables/excludes collision checks\n");
    for (int a = 0; a < m->ngeom; ++a) {
        if (!m->geom_contype[a] || !m->geom_conaffinity[a]) {
            fprintf(stderr, "noncollidable solid geom %s\n", mj_id2name(m, mjOBJ_GEOM, a));
            ok = 0;
        }
    }
    for (int b = 1; b < m->nbody; ++b) {
        if (!isfinite(m->body_mass[b]) || m->body_mass[b] <= 0) ok = 0;
        for (int k = 0; k < 3; ++k)
            if (!isfinite(m->body_inertia[3*b+k]) || m->body_inertia[3*b+k] <= 0) ok = 0;
    }
    const double zero[6] = {0};
    ok &= clear_pose(m, d, zero, "zero");
    for (int stage = 1; stage <= 3; ++stage) {
        char seat[32], rotor[32], bridge[32], shaft[32], hub[32], beam[32];
        snprintf(seat, sizeof(seat), "yaw%d_seat", stage);
        snprintf(rotor, sizeof(rotor), "disc%d_geom", stage);
        snprintf(bridge, sizeof(bridge), "fork%d_bridge", stage);
        snprintf(shaft, sizeof(shaft), "pitch%d_shaft", stage);
        snprintf(hub, sizeof(hub), "pitch%d_hub", stage);
        snprintf(beam, sizeof(beam), "arm%d_geom", stage);
        ok &= connected(m, d, seat, rotor);
        ok &= connected(m, d, rotor, bridge);
        ok &= connected(m, d, shaft, hub);
        ok &= connected(m, d, hub, beam);
        for (int side = 0; side < 2; ++side) {
            char post[32], bearing[32];
            const char *suffix = side ? "right" : "left";
            snprintf(post, sizeof(post), "fork%d_%s", stage, suffix);
            snprintf(bearing, sizeof(bearing), "bearing%d_%s", stage, suffix);
            ok &= connected(m, d, bridge, post);
            ok &= connected(m, d, post, bearing);
            ok &= connected(m, d, shaft, bearing);
        }
        if (stage < 3) {
            char next_seat[32];
            snprintf(next_seat, sizeof(next_seat), "yaw%d_seat", stage+1);
            ok &= connected(m, d, beam, next_seat);
        } else {
            ok &= connected(m, d, beam, "tip_geom");
        }
    }
    int poses = 1;
    const double angles[] = {-30, -10, 10, 30};
    for (int j = 0; j < 6; ++j) {
        for (int a = 0; a < 4; ++a) {
            double q[6] = {0};
            q[j] = angles[a];
            char label[40];
            snprintf(label, sizeof(label), "%s %+.0f deg", JOINTS[j], angles[a]);
            ok &= clear_pose(m, d, q, label);
            ++poses;
        }
    }
    const double mixed[2][6] = {{20,25,-15,15,10,20}, {-25,40,20,10,-15,15}};
    for (int i = 0; i < 2; ++i) { ok &= clear_pose(m, d, mixed[i], "mixed"); ++poses; }

    /* A real large-angle base/floor collision must remain detectable. This
     * is not a demand that the Cartesian product of joint limits is usable. */
    const double blocked[6] = {0,-60,0,0,0,0};
    if (!place(m, d, blocked) || d->ncon == 0) {
        fprintf(stderr, "expected base/floor obstruction at pitch1=-60 is missing\n");
        ok = 0;
    }
    printf("arm_geometry: %s (%d clear poses, connected supports, collision preserved)\n",
           ok ? "ok" : "FAILED", poses);
    mj_deleteData(d);
    mj_deleteModel(m);
    return ok ? 0 : 1;
}
