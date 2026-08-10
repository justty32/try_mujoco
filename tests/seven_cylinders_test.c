#include <math.h>
#include <stdio.h>
#include <string.h>

#include <mujoco/mujoco.h>

#define ARRAY_LEN(a) ((int)(sizeof(a) / sizeof((a)[0])))

typedef struct {
    const char *name;
    double expected[3];
} ExpectedPosition;

typedef struct {
    const char *name;
    double center[3];
    double end[3];
    double half_length;
} ExpectedArmGeom;

static int check_count(const char *name, int actual, int expected) {
    if (actual == expected) return 1;
    fprintf(stderr, "%s: expected %d, got %d\n", name, expected, actual);
    return 0;
}

static int check_vec3(const char *kind, const char *name,
                      const mjtNum *actual, const double expected[3], double tolerance) {
    for (int i = 0; i < 3; ++i) {
        if (fabs((double)actual[i] - expected[i]) > tolerance) {
            fprintf(stderr,
                    "%s %s: expected [%.6f %.6f %.6f], got [%.6f %.6f %.6f]\n",
                    kind, name, expected[0], expected[1], expected[2],
                    (double)actual[0], (double)actual[1], (double)actual[2]);
            return 0;
        }
    }
    return 1;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "usage: %s MODEL.xml\n", argv[0]);
        return 2;
    }

    char error[1024] = "";
    mjModel *model = mj_loadXML(argv[1], NULL, error, sizeof(error));
    if (!model) {
        fprintf(stderr, "mj_loadXML failed: %s\n", error);
        return 1;
    }

    mjData *data = mj_makeData(model);
    if (!data) {
        fprintf(stderr, "mj_makeData failed\n");
        mj_deleteModel(model);
        return 1;
    }
    mj_forward(model, data);

    int ok = 1;
    ok &= check_count("bodies", model->nbody, 8);  /* world + seven cylinders */
    ok &= check_count("joints", model->njnt, 6);
    ok &= check_count("actuators", model->nu, 6);

    static const char *JOINTS[] = {
        "yaw1", "pitch1", "yaw2", "pitch2", "yaw3", "pitch3"
    };
    static const double AXES[][3] = {
        {0, 0, 1}, {0, 1, 0}, {0, 0, 1},
        {0, 1, 0}, {0, 0, 1}, {0, 1, 0}
    };
    for (int i = 0; i < ARRAY_LEN(JOINTS); ++i) {
        int id = mj_name2id(model, mjOBJ_JOINT, JOINTS[i]);
        if (id < 0) {
            fprintf(stderr, "missing joint: %s\n", JOINTS[i]);
            ok = 0;
        } else {
            ok &= check_vec3("joint axis", JOINTS[i], model->jnt_axis + 3 * id,
                             AXES[i], 1e-12);
        }
    }

    static const ExpectedPosition BODIES[] = {
        {"base_cylinder", {0.00, 0.0, 0.10}},
        {"disc1",         {0.00, 0.0, 0.20}},
        {"arm1",          {0.00, 0.0, 0.30}},
        {"disc2",         {0.20, 0.0, 0.30}},
        {"arm2",          {0.20, 0.0, 0.39}},
        {"disc3",         {0.38, 0.0, 0.39}},
        {"arm3",          {0.38, 0.0, 0.47}},
    };
    for (int i = 0; i < ARRAY_LEN(BODIES); ++i) {
        int id = mj_name2id(model, mjOBJ_BODY, BODIES[i].name);
        if (id < 0) {
            fprintf(stderr, "missing body: %s\n", BODIES[i].name);
            ok = 0;
        } else {
            ok &= check_vec3("body position", BODIES[i].name, data->xpos + 3 * id,
                             BODIES[i].expected, 1e-9);
        }
    }

    static const ExpectedArmGeom ARM_GEOMS[] = {
        {"arm1_geom", {0.10, 0.0, 0.30}, {0.20, 0.0, 0.30}, 0.10},
        {"arm2_geom", {0.29, 0.0, 0.39}, {0.38, 0.0, 0.39}, 0.09},
        {"arm3_geom", {0.46, 0.0, 0.47}, {0.54, 0.0, 0.47}, 0.08},
    };
    const double expected_arm_axis[3] = {1.0, 0.0, 0.0};
    for (int i = 0; i < ARRAY_LEN(ARM_GEOMS); ++i) {
        int id = mj_name2id(model, mjOBJ_GEOM, ARM_GEOMS[i].name);
        if (id < 0) {
            fprintf(stderr, "missing geom: %s\n", ARM_GEOMS[i].name);
            ok = 0;
            continue;
        }

        ok &= check_vec3("geom center", ARM_GEOMS[i].name, data->geom_xpos + 3 * id,
                         ARM_GEOMS[i].center, 1e-9);

        const mjtNum *rotation = data->geom_xmat + 9 * id;
        mjtNum axis[3] = {rotation[2], rotation[5], rotation[8]};
        ok &= check_vec3("geom long axis", ARM_GEOMS[i].name, axis,
                         expected_arm_axis, 1e-9);

        double half_length = (double)model->geom_size[3 * id + 1];
        if (fabs(half_length - ARM_GEOMS[i].half_length) > 1e-12) {
            fprintf(stderr, "%s half-length: expected %.6f, got %.6f\n",
                    ARM_GEOMS[i].name, ARM_GEOMS[i].half_length, half_length);
            ok = 0;
        }

        mjtNum end[3];
        for (int j = 0; j < 3; ++j) {
            end[j] = data->geom_xpos[3 * id + j] + axis[j] * half_length;
        }
        ok &= check_vec3("geom end", ARM_GEOMS[i].name, end,
                         ARM_GEOMS[i].end, 1e-9);
    }

    int site_id = mj_name2id(model, mjOBJ_SITE, "end_effector");
    const double expected_site[3] = {0.54, 0.0, 0.47};
    if (site_id < 0) {
        fprintf(stderr, "missing site: end_effector\n");
        ok = 0;
    } else {
        ok &= check_vec3("site position", "end_effector", data->site_xpos + 3 * site_id,
                         expected_site, 1e-9);
    }

    for (int i = 0; i < model->nu; ++i) {
        if (!model->actuator_ctrllimited[i] ||
            fabs((double)model->actuator_ctrlrange[2 * i] + 1.0) > 1e-12 ||
            fabs((double)model->actuator_ctrlrange[2 * i + 1] - 1.0) > 1e-12) {
            fprintf(stderr, "actuator %d does not have ctrlrange [-1, 1]\n", i);
            ok = 0;
        }
    }

    for (int i = 0; i < 10; ++i) mj_step(model, data);
    for (int i = 0; i < model->nq; ++i) {
        if (!isfinite((double)data->qpos[i])) {
            fprintf(stderr, "non-finite qpos[%d] after stepping\n", i);
            ok = 0;
        }
    }

    printf("seven_cylinders: %s (%d bodies, %d joints, %d actuators, MuJoCo %s)\n",
           ok ? "ok" : "FAILED", model->nbody - 1, model->njnt, model->nu,
           mj_versionString());

    mj_deleteData(data);
    mj_deleteModel(model);
    return ok ? 0 : 1;
}
