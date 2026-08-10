# robot_core 公開 C ABI 草案

← [計畫入口](README.md)

本 API 面向自訂 C 演算法與其他語言 binding。v1 不公開 `mjModel`、`mjData`、GLFW type、內部 task vtable 或 worker handle。

## ABI 規則

- `ROBOT_ABI_VERSION` 從 1 開始；每個公開 option/result struct 第一欄是 `struct_size`。
- Status 用 `int32_t`，flag 用 `uint32_t/uint64_t`；不把 compiler-dependent enum、C `bool` 或 bitfield 放進 ABI。
- 數值邊界固定 `double`；若未來 MuJoCo 改成 single precision，在 core 內轉換。
- Caller 配置 action/observation/result buffer；hot path 不配置記憶體、不回傳內部可變 pointer。
- 所有 create/load 函式接受 caller-owned `robot_error*`；library 不 `exit()`、不寫任意 stdout/stderr。
- v1 同步 API 不可 re-enter；同一 handle 同時只能由一個 caller 操作。
- Caller 把公開 input/output struct 清零並設定 `struct_size=sizeof(struct)`；library 只讀／寫雙方共同認得的欄位，reserved 必須保持 0。

## 共用型別

```c
#define ROBOT_ABI_VERSION 1u
#define ROBOT_ERROR_MESSAGE_SIZE 512u

typedef int32_t robot_status;
enum {
    ROBOT_OK = 0,
    ROBOT_ERROR_INVALID_ARGUMENT = 1,
    ROBOT_ERROR_OUT_OF_MEMORY = 2,
    ROBOT_ERROR_IO = 3,
    ROBOT_ERROR_MODEL = 4,
    ROBOT_ERROR_SCHEMA = 5,
    ROBOT_ERROR_STATE = 6,
    ROBOT_ERROR_VERSION = 7,
    ROBOT_ERROR_NONFINITE_ACTION = 8,
    ROBOT_ERROR_PARTIAL_FAILURE = 9,
    ROBOT_ERROR_BUSY = 10
};

typedef struct robot_model robot_model;
typedef struct robot_env robot_env;
typedef struct robot_batch robot_batch;

typedef struct {
    uint32_t struct_size;
    int32_t code;
    char message[ROBOT_ERROR_MESSAGE_SIZE];
} robot_error;

typedef struct {
    uint32_t struct_size;
    char name[32];
    uint32_t offset;
    uint32_t count;
} robot_field_spec;

uint32_t robot_abi_version(void);
const char *robot_version_string(void);
const char *robot_status_string(robot_status status);
```

`robot_error` 僅在錯誤時有意義。每次 public call 都回 status；caller 可用 `robot_status_string(status)` 取得靜態短描述。

## Model API 與 ownership

```c
typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t nq;
    uint32_t nv;
    uint32_t nu;
    uint32_t action_dim;
    uint32_t mujoco_version_major;
    uint32_t mujoco_version_minor;
    uint32_t mujoco_version_patch;
    uint8_t model_sha256[32];
    uint64_t reserved[8];
} robot_model_spec;

robot_status robot_model_load(const char *path,
                              robot_model **out_model,
                              robot_error *error);
void robot_model_retain(robot_model *model);
void robot_model_release(robot_model *model);

robot_status robot_model_get_spec(const robot_model *model,
                                  robot_model_spec *out_spec,
                                  robot_error *error);
robot_status robot_model_get_action_bounds(const robot_model *model,
                                           double *low, double *high,
                                           size_t count,
                                           robot_error *error);
```

`robot_model` 內含：compiled `mjModel`、模型 SHA-256、MuJoCo version、joint/actuator cached IDs、action bounds 與 atomic reference count。Task-specific site IDs 與 observation schema 在 env create 時解析並存進 env/task state。load 完成後 model 視為唯讀。

`robot_env_create` 與 `robot_batch_create` 會 retain model；caller 可在 create 成功後立即 release 自己的 reference。最後一個 owner release 才 `mj_deleteModel`。

## Single environment API

```c
enum {
    ROBOT_TASK_RAW_V1 = 1,
    ROBOT_TASK_SEVEN_REACH_V1 = 2
};

enum {
    ROBOT_ACTION_STRICT = 1,
    ROBOT_ACTION_CLIP = 2
};

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t task_kind;
    uint32_t action_policy;
    uint32_t frame_skip;
    uint32_t max_episode_steps;
    uint64_t reserved[8];
} robot_env_config;

typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t nq;
    uint32_t nv;
    uint32_t nu;
    uint32_t observation_dim;
    uint32_t action_dim;
    uint32_t observation_field_count;
    uint32_t frame_skip;
    uint32_t task_kind;
    uint32_t action_policy;
    uint32_t max_episode_steps;
    uint32_t reserved0;
    double physics_dt;
    double policy_dt;
    uint64_t reserved[8];
} robot_env_spec;

enum {
    ROBOT_STEP_TERMINATED = 1u << 0,
    ROBOT_STEP_TRUNCATED = 1u << 1,
    ROBOT_STEP_SUCCESS = 1u << 2,
    ROBOT_STEP_ACTION_CLIPPED = 1u << 3,
    ROBOT_STEP_MUJOCO_WARNING = 1u << 4,
    ROBOT_STEP_NUMERIC_FAILURE = 1u << 5
};

typedef struct {
    uint32_t struct_size;
    uint32_t flags;
    int32_t env_status;
    uint32_t reserved0;
    uint64_t episode_id;
    uint64_t episode_step;
    double reward;
    double episode_return;
    double distance_to_goal;
    uint64_t warning_mask;
    uint64_t reserved[4];
} robot_step_result;

void robot_env_default_config(robot_env_config *config);
robot_status robot_env_create(robot_model *model,
                              const robot_env_config *config,
                              robot_env **out_env,
                              robot_error *error);
void robot_env_destroy(robot_env *env);

robot_status robot_env_get_spec(const robot_env *env,
                                robot_env_spec *out_spec,
                                robot_error *error);
robot_status robot_env_get_observation_field(const robot_env *env,
                                             uint32_t index,
                                             robot_field_spec *out_field,
                                             robot_error *error);
robot_status robot_env_reset(robot_env *env,
                             uint64_t seed,
                             double *observation,
                             size_t observation_count,
                             robot_error *error);
robot_status robot_env_step(robot_env *env,
                            const double *action,
                            size_t action_count,
                            double *observation,
                            size_t observation_count,
                            robot_step_result *result,
                            robot_error *error);
```

Task schema：

| Task | Observation | Reward/episode |
|------|-------------|----------------|
| `RAW_V1` | `qpos[nq] + qvel[nv]` | reward=0；只依 max steps truncated，供核心 smoke/debug |
| `SEVEN_REACH_V1` | `qpos[6] + qvel[6] + achieved[3] + desired[3] + delta[3]` | dense distance reward、success terminated、time-limit truncated |

Task create 時驗證模型具備所需 joint/actuator/site；不相容模型回 `ROBOT_ERROR_SCHEMA`，不以錯誤 index 繼續。

Step 行為固定為：驗證 action → 寫入 control → `frame_skip` 次 `mj_step` → task observation/reward/done → 更新 result。Action 在所有 substeps 保持不變。`terminated` 或 `truncated` 後再次 step 回 `ROBOT_ERROR_STATE`，直到 caller reset。

Strict mode 遇到 out-of-range 或非有限 action 時不推進該 env；clip mode 只 clip 有限的超界值並設 `ACTION_CLIPPED`。NaN/Inf 永遠不 clip，視為錯誤。

## Snapshot API

```c
robot_status robot_env_state_size(const robot_env *env,
                                  size_t *out_size,
                                  robot_error *error);
robot_status robot_env_save_state(const robot_env *env,
                                  void *buffer, size_t buffer_size,
                                  robot_error *error);
robot_status robot_env_load_state(robot_env *env,
                                  const void *buffer, size_t buffer_size,
                                  robot_error *error);
```

Snapshot envelope 包含 ABI/version、model hash、MuJoCo `mjSTATE_INTEGRATION`、RNG、target、episode id/step/return、success streak。load 驗證後呼叫 `mj_setState` 與必要的 forward 計算；任何不相符都在修改 env 前拒絕。

## Batch API

```c
typedef struct {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t num_envs;
    uint32_t worker_count;       /* 0 = auto；create 後固定 */
    robot_env_config env;
    uint64_t reserved[8];
} robot_batch_config;

void robot_batch_default_config(robot_batch_config *config);
robot_status robot_batch_create(robot_model *model,
                                const robot_batch_config *config,
                                robot_batch **out_batch,
                                robot_error *error);
void robot_batch_destroy(robot_batch *batch);

robot_status robot_batch_reset(robot_batch *batch,
                               const uint8_t *reset_mask,
                               const uint64_t *seeds,
                               double *observations,
                               size_t observation_stride,
                               robot_step_result *results,
                               robot_error *error);

robot_status robot_batch_step(robot_batch *batch,
                              const double *actions,
                              size_t action_stride,
                              double *observations,
                              size_t observation_stride,
                              robot_step_result *results,
                              robot_error *error);
```

- `reset_mask == NULL` 代表全部 reset；否則只有非零項被重設，其他 observation/result 不改。
- 被 reset 的 `results[i]` 會清除 terminal/error flags，並填入新的 episode id、step=0、reward/return=0。
- `seeds[i]` 是 env i 的 episode seed；core 不依賴 worker id 或執行順序導出 RNG。
- `actions`/`observations` 是 row-major；stride 以 `double` 元素計，必須至少為 dim。
- Batch 不 auto-reset。runner 先保存 terminal result/observation，再組下一個 reset mask。
- 單一 env 失敗時 batch 回 `ROBOT_ERROR_PARTIAL_FAILURE`，對應 `results[i].env_status` 記原因；其他 env 可以完成本次 step。
- v1 是 synchronous barrier。未經 profile 證明需要，不增加 `step_async/step_wait`。

## Thread-safety

- `robot_model` load 完成後唯讀，可被多個 env/thread 共用。
- 一個 `robot_env` 同時只由一個 worker 操作；不同 env 可平行。
- `robot_batch` 的 create/reset/step/destroy 不可互相重入。
- Core 不安裝 per-env MuJoCo global callback；control 在 `mj_step` 前明確寫入。若日後需要 callback，先設計 thread-local dispatch，不直接使用全域 mutable context。
- Worker 採固定 range，例如 worker 0 處理 `[0,k)`；每個 env 的 RNG、result、episode buffer 都不共享 cache line 熱點。

## 自訂演算法最小迴圈

```c
robot_model *model = NULL;
robot_batch *batch = NULL;
robot_error error = {.struct_size = sizeof(error)};

/* load model, query spec, create batch, allocate buffers */
robot_batch_reset(batch, NULL, seeds, obs, obs_stride, results, &error);

for (;;) {
    my_algorithm(actions, obs, results, num_envs);
    robot_batch_step(batch, actions, action_stride,
                     obs, obs_stride, results, &error);

    build_reset_mask(results, reset_mask, seeds);
    robot_batch_reset(batch, reset_mask, seeds,
                      obs, obs_stride, results, &error);
}
```

演算法控制 episode loop；`robot_headless` 只提供 zero/random/demo policy 與完整使用範例，不把 policy callback 固化進 core ABI。
