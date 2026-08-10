# Run、trajectory、replay 與影片格式

← [計畫入口](README.md)

## Run 目錄

```text
runs/20260810T142530Z-seven_reach-seed42/
  run.json
  model.mjb
  model.sha256
  episodes.jsonl
  trajectories/
    env0000-episode00000123.rtraj
  videos/
  tmp/
```

- 目錄用 exclusive create；已存在即失敗，不默默覆寫。
- `model.mjb` 由當次 compiled `mjModel` 輸出，確保原始 MJCF/mesh 後來變動仍能重播。
- 寫 trajectory 時先放 `tmp/*.partial`；完整寫入、flush、close 後才 rename 到 `trajectories/`。
- Crash 後 `episodes.jsonl` 只接受完整 newline；`.partial` 可由檢查工具回報並忽略。

## `run.json`

最低欄位：

```json
{
  "format_version": 1,
  "created_utc": "2026-08-10T14:25:30Z",
  "robot_abi_version": 1,
  "mujoco_version": "3.10.0",
  "model_sha256": "...",
  "model_source": "assets/seven_cylinders.xml",
  "task": "seven_reach_v1",
  "num_envs": 64,
  "worker_count": 4,
  "run_seed": 42,
  "physics_dt": 0.002,
  "frame_skip": 10,
  "policy_dt": 0.02,
  "max_episode_steps": 250,
  "action_policy": "strict",
  "algorithm": {"name": "random", "config": {}},
  "build": {"git_commit": "...", "dirty": true, "cmake_build_type": "Release"},
  "platform": {"os": "Windows", "cpu": "...", "logical_processors": 8}
}
```

Runner 把「解析後的有效設定」寫入，而不是只記 command line。若 repo dirty，至少記 dirty=true；可再用 `--capture-diff` 額外保存 patch，但預設不把可能含敏感資料的任意工作樹內容複製進 run。

JSON v1 只需要 writer，不引入大型 JSON dependency；所有字串必須正確 escape。Replay 的物理來源是 binary header/model.mjb，不靠解析任意 JSON 才能工作。

## `episodes.jsonl`

每個 episode 一行：

```json
{"episode_id":123,"env_id":0,"seed":9182,"steps":211,
 "return":-8.42,"success":true,"terminated":true,"truncated":false,
 "reason":"success","final_distance":0.027,"warning_mask":0,
 "trajectory":"trajectories/env0000-episode00000123.rtraj"}
```

- 所有 episode 都記 summary；沒有 trajectory 時 `trajectory` 為 null。
- `reason` 使用固定集合：`success`、`time_limit`、`numeric_failure`、`invalid_action`、`user_stop`。
- Summary 在 episode boundary 寫入，量低；可以 buffered synchronous append，避免為簡單資料先做複雜 queue。
- 每 N 行 flush 可配置；正常 shutdown 必須 flush。效能 benchmark 包含 flush policy。

## `.rtraj` binary v1

固定 little-endian。Reader 先看 magic/endian/version/header size，再配置任何大型 buffer。

### Header

```text
magic[8]          = {'R','B','T','R','J',0,1,0}
header_size       u32
format_version    u32 = 1
endian_marker     u32 = 0x01020304
scalar_type       u32 = 2          # f64
robot_abi_version u32
mujoco_version    {major,minor,patch} u32 x3
model_sha256      u8[32]
task_kind         u32
nq,nv,nu          u32 x3
observation_dim   u32
action_dim        u32
state_signature   u32 = mjSTATE_INTEGRATION
physics_dt        f64
frame_skip        u32
env_id            u32
episode_id        u64
seed              u64
reserved          u8[128]
header_crc32      u32
```

所有 binary struct 以逐欄 encode/decode 寫入，不直接 `fwrite(struct)`；避免 padding 與 compiler ABI 差異。

### Chunk

```text
type              u32
flags             u32
policy_step       u64
payload_size      u64
payload_crc32     u32
reserved          u32
payload           byte[payload_size]
```

Chunk type：

1. `INITIAL_STATE`：robot snapshot，含 `mjSTATE_INTEGRATION` 與 task/RNG/episode state。
2. `ACTION_BLOCK`：連續 policy actions；預設每 64 steps 一塊。
3. `OBSERVATION_BLOCK`：可選 debug；預設不存，避免重複資料。
4. `STATE_CHECKPOINT`：預設每 100 policy steps，支援 seek 與 drift 校正。
5. `EVENT`：success、warning、numeric failure 等少量事件。
6. `END`：final result、chunk count、整檔 rolling checksum。

Reader 限制 header/chunk size 上限，所有乘法先檢查 overflow；CRC 不符、chunk 截斷或未知 required flag 時明確失敗。

## 哪些 episode 留 trajectory

每個 env 預先配置當前 episode buffer，只保存 initial snapshot、actions、events 與稀疏 checkpoints；episode 結束才決定是否落盤。

Selector 可組合：

- `periodic:N`：每 N 個 episode 保存一個，預設只選少量 env。
- `success` / `failure` / `numeric_failure`。
- `return_below:X` / `return_above:X`。
- `best:K` / `worst:K`：runner 保留 K 個候選；新候選替換舊候選時安全刪除或移到 run tmp/trash policy。
- `env_ids:0,1`：只允許指定 env 產生 trajectory。

大 batch 時不是每個 env 都必須配置 observation history。以 6 維 action、250 steps、f64 計算，每 episode action 僅約 12 KB；64 env 約 0.75 MB。若 max steps/obs 很大，create 時依設定估算並回報 RAM budget。

Trajectory writer 使用單一背景 thread 與 bounded SPSC queue：main/batch caller 在 episode boundary enqueue 完整 immutable buffer，writer 只負責檔案。Summary 不丟；可選 trajectory queue 滿時依設定 `drop_with_flag` 或在 episode boundary backpressure，永遠不在 physics worker 中 I/O。

## Replay 流程

1. 讀 header，驗 magic/CRC/size/version。
2. 載入 run 內 `model.mjb`，計算 hash；預設 hash 不符即拒絕。
3. 比對 MuJoCo major/minor、robot ABI、task/schema/dim。`--allow-mismatch` 僅供人工診斷，輸出必須帶醒目 warning。
4. 建立單一 `robot_env`，載入 `INITIAL_STATE`。
5. 逐 action 做相同 `frame_skip`；遇 checkpoint 比較 state。
6. `--verify` 模式計算 qpos/qvel/task state 最大誤差；超過 tolerance 退出非零。
7. `--seek STEP` 從最近 checkpoint 開始，不必從 episode 開頭重播。

同版本、同 model、同平台的測試先要求 bitwise 或非常小 tolerance；跨硬體只保證在記錄 tolerance 內，不宣稱永遠 bitwise deterministic。

## 影片

- `robot_replay --render human`：互動視窗。
- `robot_replay --render offscreen --fps 30 --raw-rgb out.rgb`：固定 FPS raw RGB。
- `robot_replay --video out.mp4 --ffmpeg PATH`：完成 offscreen frame 後交給 ffmpeg。

Frame 時間依 simulation time 判斷；不是每個 physics step 畫一張。Camera preset、resolution、FPS、是否顯示 target/contact overlay 寫入 video sidecar JSON。

Windows v1 使用 hidden GLFW context；Linux backend 之後加 EGL/OSMesa。Core/record/replay verify 不依賴 GL。ffmpeg 整合不使用未 escape 的 `system()` 字串：先輸出 raw RGB + 明確 argv，或由小型 platform process wrapper 安全啟動。
