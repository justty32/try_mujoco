# robot_core、headless、batch、record/replay 實作計畫

狀態：**Proposed，尚未開始實作**

基準：目前 `main.c + sim.c + controller.c + viewer.c`、MuJoCo 3.10.0、七圓柱 6-DoF 模型

目標：把現有單一互動程式演進成可由自訂 C 演算法呼叫、可大量 headless rollout、可選擇性紀錄與重播的模擬環境。

本目錄是計畫入口：

- [api.md](api.md)：公開 C ABI、ownership、buffer layout、thread-safety 與 task 契約。
- [formats.md](formats.md)：run/episode/trajectory 格式、紀錄選擇、重播與影片流程。
- [milestones.md](milestones.md)：依賴順序、逐階段檔案變更、測試與 `Done when`。
- 上游評估與框架比較見 [模擬環境評估報告](../../simulation_environment_report.md)。

## 完成後應得到什麼

```text
自訂 C 演算法 / robot_headless
            │ public C ABI + contiguous buffers
            ▼
       robot_core library
       ├─ robot_model：唯讀 mjModel、名稱解析、action schema、model hash
       ├─ robot_env：一份 mjData、RNG、task/episode state
       ├─ robot_batch：N 個 env、固定 worker pool、masked reset
       └─ robot_record：run summary、selected trajectory、checkpoint
            │                         │
            │                         └── .rtraj ──► robot_replay ──► video
            ▼
    physics only；不依賴 GLFW/OpenGL

robot_viewer：同樣使用 robot_core，但 renderer 為可選 target
```

最重要的邊界是：演算法只看 `robot/*` 公開 header；MuJoCo、GLFW、task vtable、worker pool 與檔案 writer 都是 private implementation。viewer 可以經 repo-private accessor 取得 `mjModel/mjData`，但這不是穩定 ABI。

## 固定決策

| 主題 | v1 決策 | 原因 |
|------|---------|------|
| ABI | Opaque handles、fixed-width integer、公開 struct 帶 `struct_size`/version | 允許日後加欄位，不暴露 MuJoCo layout |
| 核心 ownership | `robot_model` 擁有唯讀 `mjModel`；每個 `robot_env` 擁有一份 `mjData` | 符合 MuJoCo 平行 sampling 模型 |
| Step 語意 | 同步、blocking、固定 `frame_skip`；無自動 reset | 結果簡單、terminal observation 不會被 reset observation 覆蓋 |
| Action v1 | 先支援 actuator torque/control，維度 `nu`；strict/clip 可配置 | 最接近目前控制方式，也不預綁自己的演算法 |
| Observation | 連續 row-major `double` buffer；schema 可查詢 | 演算法沒有 magic index，batch 可直接使用 |
| 時脈 | physics dt 由 MJCF；v1 預設 `frame_skip=10`，policy dt=20 ms/50 Hz | timestep 2 ms 的整數倍，避免目前 1/60 秒造成 8/9 step 抖動 |
| Reset | 每個 env 自有 64-bit RNG；seed 不使用 `rand()` | worker 數或排程改變時結果仍可重現 |
| Batch | 一個 shared model、N 個 env；固定 C11 worker pool、static range | 避免每 step 建 thread，也避免排程影響 env RNG |
| Model randomization | v1 只做 `mjData`/task state randomization；改 `mjModel` 延後 | shared model 不可被 worker 同時修改 |
| Snapshot | MuJoCo `mjSTATE_INTEGRATION` + robot task state | 包含 control、warmstart、mocap/user/plugin 等重播所需輸入 |
| Recorder | episode summary 全留；trajectory 先在 RAM 累積，episode 結束後才依規則落盤 | 不讓每個 training step 都做檔案 I/O |
| Replay | 預設要求 model hash、MuJoCo major/minor、ABI/schema 相符 | 不把版本不符造成的漂移誤判為演算法問題 |
| Video | train 不建立 GL context；`robot_replay` 才 offscreen render | 物理 throughput 與影片完全解耦 |
| First task | `seven_reach_v1` 作為環境契約驗收 task | 能測 observation、reward、success、termination、seed 與 batch |

## `seven_reach_v1` 驗收 task

這不是最終研究題目，只是證明環境 API 完整的第一個 task。

- Action：六個 actuator control，預設範圍 `[-1, 1]`。
- Observation，21 維：`qpos[6] + qvel[6] + achieved_pos[3] + desired_pos[3] + delta[3]`。
- Target：reset 時抽樣安全 joint configuration，做一次 FK 取得可達的 end-effector 世界座標；target 留在 task state，不修改 shared model。
- 初始狀態：從另一組安全 joint configuration 抽樣，速度為 0。
- Reward：dense `-||achieved - desired||`；同時在 info 提供 distance，之後可加 sparse mode。
- Success：距離小於 0.03 m，連續 5 個 policy steps。
- `terminated`：成功或數值失敗；`truncated`：250 policy steps，約 5 秒模擬時間。
- Renderer：viewer/replay 用 decorative geom 畫 target；headless core 不需要把 target 放入 MJCF。

## 目標 build targets

| CMake target | 產物 | 依賴 |
|--------------|------|------|
| `robot_core` | static library，之後可選 shared | MuJoCo、Threads；**無 GLFW/OpenGL** |
| `robot_headless` | 單／多環境 CLI smoke runner | `robot_core` |
| `robot_render` | 可選 renderer library | `robot_core`、GLFW、OpenGL |
| `robot_viewer` | 互動 viewer；輸出檔暫保留 `robot.exe` | `robot_core`、GLFW、OpenGL |
| `robot_replay` | trajectory 驗證；啟用 rendering 時再加 interactive/offscreen | `robot_core`，可選 `robot_render` |
| `robot_tests` | CTest targets | 各自最小依賴 |

CMake 選項：

```cmake
ROBOT_BUILD_RENDERING=ON
ROBOT_BUILD_VIEWER=ON
ROBOT_BUILD_REPLAY=ON
ROBOT_BUILD_TESTS=ON
ROBOT_ENABLE_ASAN=OFF
BUILD_SHARED_LIBS=OFF
```

`ROBOT_BUILD_RENDERING=OFF` 時不得宣告或下載 GLFW，且 replay 只保留 verify/seek；這是 headless server/CI 能否乾淨建置的驗收條件。`ROBOT_BUILD_VIEWER=ON` 必須要求 rendering 同時開啟。

## 目標 repo 佈局

```text
include/robot/
  robot.h               # status、version、共用型別
  model.h               # model load/spec/schema/hash
  env.h                 # single env reset/step/state
  batch.h               # batch/masked reset
  record.h              # recorder policy/config

src/core/
  model.c  env.c  task_reach.c  state.c  error.c  internal.h
src/batch/
  batch.c  worker_pool.c  worker_pool.h
src/io/
  record.c  trajectory.c  json_writer.c  sha256.c
src/render/
  viewer.c  offscreen.c  render_internal.h
apps/
  viewer_main.c  headless_main.c  replay_main.c
  policies/demo_policy.c
tests/
  model_test.c  env_test.c  determinism_test.c  batch_test.c
  record_replay_test.c  public_abi_test.c
```

依 repo 規則，單檔接近 300 行即檢視是否按職責拆分；不要把所有 API 與 worker/record 實作重新堆進一個大型 `sim.c`。

## 里程碑順序

| 順序 | 里程碑 | 可交付結果 |
|------|--------|------------|
| M0 | 行為基線與契約凍結 | 現行模型／步進基線與 ABI 決策可測 |
| M1 | CMake 分層與 `robot_core` 骨架 | core 不依賴 renderer，外部程式能 link |
| M2 | Instance-based single env + viewer migration | 無 global model/data，兩個 env 互不干擾 |
| M3 | `seven_reach_v1` + headless runner | 無視窗完成 episode、輸出 reward/success |
| M4 | CPU batch | masked reset、N env 同步 step、固定 worker pool |
| M5 | Run/episode/trajectory recorder | summary 全留、selected trajectory 可恢復 |
| M6 | Replay + offscreen video | state/action round-trip、選定 episode 產片 |
| M7 | Hardening、ABI/install/docs | ASan、corruption tests、安裝與外部 sample |

完整工作包與驗收見 [milestones.md](milestones.md)。每個里程碑都必須保持 Release build 與全部既有 CTest 綠燈；不採用一次性大改。

## 全案 Definition of Done

- `sim.c` 的 global `mjModel*`/`mjData*` 與核心對 `exit()` 的使用已消失。
- 一個外部 C sample 只 include `include/robot/*.h` 即可 load、reset、step、destroy。
- `robot_headless --num-envs 1` 可在沒有 GLFW/OpenGL context 下跑 10,000 episodes。
- batch=1 和 batch=N 在相同 per-env seeds/actions 下產生相同結果；worker count 改變不改變 rollout。
- terminal observation 在 reset 前可取得；partial reset 只改指定 env。
- 每個 run 有 resolved metadata、model MJB/hash、episode JSONL；selected trajectory 可驗證並 seek/replay。
- corrupt/truncated trajectory、model hash/version mismatch 都會明確拒絕，不 crash。
- train path 不 render；replay 可輸出固定 FPS 的影片。
- summary-only logging 的端到端 overhead 經 benchmark 證明小於 5%；performance 數字記錄但不作跨硬體 CI gate。
- README、HTML API、code map 與本計畫的「已完成／未完成」狀態同步。
