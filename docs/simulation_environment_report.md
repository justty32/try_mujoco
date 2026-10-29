# 機器手臂物理模擬環境評估與改進建議

評估日期：2026-08-10

> 正式模型更新（2026-10-29）：使用者已核准典型 9 圓柱 6R 外形與 home 姿態，現行入口為 [`assets/six_axis_arm.xml`](../assets/six_axis_arm.xml)；操作見 [README](../README.md)。下方七圓柱／叉耳模型的補註與評估均保留為歷史，已被本次 6R 模型取代，不能當成現行座標、質量、馬達額定或驗收結果。robot_core 大型計畫仍未開始。

> 現況補註（2026-10-29）：互動 viewer 已加入具名關節目標、姿態保持與 headless 控制測試，操作見 [README](../README.md#操作說明)。下文是原評估基線；其中「沒有 position mode」已由本次互動關節控制補上，但尚無通用 action schema、instance-based core 或批次訓練 API，robot_core 計畫仍未開始。

> 幾何補註（2026-10-29）：同一歷史檔名 `seven_cylinders.xml` 已換成六關節叉耳式模型，保留關節中心／軸／範圍與末端位置，補出轉盤、叉耳與轉軸的裝配間隙。原粗圓柱模型與本輪取樣／真實渲染均保存在 [幾何驗證紀錄](reports/geometry-2026-10-29/README.md)，可直接開 [比較頁](reports/geometry-2026-10-29/preview.html)。新模型以 1000 kg/m³ 的教學密度計算質量與慣量，仍非硬體校準；`pitch1 = -60°` 仍有真實基座／地面碰撞。下方「seven-cylinder 修正」保留為當時的歷史紀錄，不能視為現行外形或全姿態無碰撞證明。

## 結論

目前環境適合「學 MuJoCo、驗證 MJCF、互動看關節運動」，但還不能稱為可大量訓練的機器人學習平台。選 MuJoCo 與 C 是合理的；缺的不是另一套物理引擎，而是物理核心外面的環境層：可重入 instance、reset/step 契約、任務與 reward、批次執行、紀錄、重播、測試和實驗管理。

你預想的方向是對的：訓練時只跑物理與數值紀錄，不開視窗；只有定期評估、最佳／最差或異常 episode 才重播並輸出影片。MuJoCo 官方本身就把互動 viewer 與 offscreen `record` 範例分開，後者可用 `mjr_readPixels` 取像素再交給 ffmpeg。[MuJoCo code samples](https://mujoco.readthedocs.io/en/latest/programming/samples.html#record)

我的建議是先留在原生 MuJoCo CPU/C 路線，不要急著搬到 Isaac Lab、ManiSkill 或 MJX。先把乾淨的 C 環境 API、CPU batch runner、紀錄與 replay 做好；實測 CPU 吞吐成為瓶頸後，再評估 MJX-Warp/JAX 或其他 GPU backend。這樣最符合「之後接自己的 C 演算法」，也保留換物理 backend 的空間。

## 現況判讀

### 已經做對的部分

- `sim`、`controller`、`viewer` 已拆成不同原始碼檔，方向正確。[src/](../src/)
- 使用 MuJoCo 原生 C API，沒有多繞一層 Python；對自訂 C 演算法是優勢。
- 物理 timestep 是 2 ms，viewer 以約 60 Hz 顯示；物理頻率與顯示頻率已有概念上的分離。[main.c](../src/main.c)
- 模型與控制器都有具名 joint/actuator，七段模型現在有可驗證的 end-effector site。[seven_cylinders.xml](reports/geometry-2026-10-29/after.xml)
- 本次已加入第一個 headless 模型測試；它會用真正的 MuJoCo loader 檢查拓撲與零姿態座標。`seven_cylinders_test.c`（歷史檔名；現行測試入口為 [six_axis_arm_test.c](../tests/six_axis_arm_test.c)）

### 阻止大量訓練的主要缺口

| 面向 | 目前狀態 | 造成的限制 |
|------|----------|------------|
| 模擬 instance | `sim.c` 只有一組全域 `mjModel*` / `mjData*` | 同一行程無法安全建立 N 個環境 |
| 執行模式 | `main` 固定初始化 GLFW viewer | 沒有純 headless runner，訓練一定綁視窗 |
| 環境契約 | 只有 load/reset/step 與零散 getter | 沒有 observation/action 規格、reward、episode、成功條件 |
| 控制 | 6 維陣列與 actuator index 寫死 | 模型一改就容易錯；也沒有 torque/position/velocity/Cartesian action mode |
| 錯誤處理 | load 失敗直接 `exit(1)`；查不到 joint 回 0 | library 使用者無法恢復；「真的為 0」和「名稱錯」無法區分 |
| 紀錄與重播 | 只有 stdout 與畫面 HUD | 無法比較 run、找壞 episode、重播或產片 |
| 可重現性 | 沒有 seed、run metadata、model hash | 訓練結果難以追查 |
| 物理模型 | 基本 geom、joint damping、motor gear | 缺實際 mass/inertia、摩擦、actuator 動態、sensor、工作物與任務 |
| 建置 | MuJoCo 預設路徑是 `C:/dev/mujoco`，本次已改成可覆寫 CMake cache path | 換機可用 `-DMUJOCO_DIR=...`，但 MinGW 路徑仍在 batch 檔 |
| 文件 | README/API 頁已同步 seven-cylinder；後續架構變更仍需同 commit 更新 | 避免公開介面、模型與文件再次分叉 |

另有一個要先修的控制語意：`ctrl_set_motors()` 寫入的值會在下一個 physics step 前被 `ctrl_apply()` 覆蓋，因此目前的「manual override」不是持久模式。正式 API 應讓 action 成為 environment step 的輸入，不要讓示範 phase 控制器在背後覆蓋。

## seven-cylinder 修正

原模型有兩個獨立問題：

1. MuJoCo cylinder 的 `size="radius half-length"`，原檔把第二個值當完整長度，卻又把下一個 body 放在該數值的位置。結果後一個關節落在前一根圓柱的中心。
2. 水平臂沿 X 軸，但 pitch hinge 也繞 X 軸；這只會讓圓柱繞自己的長軸轉，不會把後續鏈節抬起。pitch 軸應與臂長軸垂直，本模型改成 Y 軸。

本次修改後的拓撲是：

```text
world
└─ base_cylinder (fixed)
   └─ disc1 -- yaw1 (Z)
      └─ arm1 -- pitch1 (Y)
         └─ disc2 -- yaw2 (Z)
            └─ arm2 -- pitch2 (Y)
               └─ disc3 -- yaw3 (Z)
                  └─ arm3 -- pitch3 (Y)
                     └─ end_effector site
```

每支 arm 的 body 原點在關節，geom 中心位於臂長一半，下一個 body 位於完整臂長。底座與三個轉盤的高度也改成與註解一致。motor 增加 `ctrlrange="-1 1"`，避免演算法意外輸入無界控制。

這是「7 個圓柱、6 DoF」：固定底座不算自由度。如果你原意是 7-DoF 手臂，就還要增加第 7 個 joint/actuator；目前程式與控制陣列都明確是 6 DoF。

驗證：

```powershell
cmd /c build.bat
ctest --test-dir build --output-on-failure
```

`seven_cylinders_model` 已通過，證明 MuJoCo 3.10.0 能載入模型，且 7 body、6 joint、6 actuator、交替 Z/Y 軸、各連接點與 end-effector 零姿態座標符合預期。這是數值與結構驗證；外觀仍可用 viewer 做一次人工 sanity check。

## 和常見方案差在哪裡

| 系統 | 它在 MuJoCo／物理引擎上再提供什麼 | 對本專案的啟示 |
|------|------------------------------------|----------------|
| [Gymnasium](https://gymnasium.farama.org/api/env/) / [Gymnasium Robotics](https://robotics.farama.org/envs/fetch/reach/) | `reset(seed)`、`step(action)`、observation、reward、`terminated`/`truncated`；goal task 另分 achieved/desired goal | 借它的契約，不必採用 Python |
| [robosuite](https://robosuite.ai/docs/modules/environments.html) | robot、arena、object、controller、sensor、task/reward 的模組化；renderer 可關閉或 offscreen | 你的 `sim` 還只是 engine wrapper，尚缺 environment/task/controller 層 |
| [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/setup/quickstart.html) | vectorized env、`--num_envs`、`--headless`、模組化 action/observation/randomization | 批次、headless、訓練／play 設定分離是成熟平台的基本配備 |
| [ManiSkill](https://maniskill.readthedocs.io/en/latest/user_guide/getting_started/quickstart.html) | GPU 批次 physics/render、Gym API、task 初始化/成功/reward/觀測慣例 | 大規模視覺訓練很強，但 Python/Torch/PhysX 生態與你的 C 優先路線不同 |
| [MuJoCo MJX](https://mujoco.readthedocs.io/en/latest/mjx.html) | JAX/Warp 裝置端模型、batch 維度、GPU/TPU 大量相同場景 | 未來需要數千環境才考慮；它不是原生 C ABI 的直接延伸 |
| [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) | 經整理的高品質 robot MJCF、mesh、scene 與模型說明 | 要做真實手臂時，優先借可信模型與參數，不要從圓柱猜 inertia |

差距最大的不是畫質，而是「environment contract 與 experiment pipeline」。市面框架會明確區分物理狀態、agent observation、action mapping、任務 reward/termination、訓練 randomization、評估設定與 render。你現在把 MuJoCo 跑起來了，但這幾層尚未建立。

## 別人通常怎麼訓練機器手臂

一個常見迴圈如下：

1. `reset(seed)`：重設 robot/object，抽樣目標與物理參數，回傳 observation。
2. policy/自訂演算法根據 observation 產生 action。
3. environment 把 action 轉成 torque、joint target 或 end-effector target，保持數個 physics substeps。
4. 回傳 next observation、reward、`terminated`、`truncated`、success 與診斷資訊。
5. 多個環境同時收集 rollout；演算法更新參數。
6. 每隔固定步數，用固定 seeds、關閉訓練擾動做 evaluation；只錄少量影片與 checkpoint。

Gymnasium 特別把「任務真的結束」與「時間上限截斷」拆成 `terminated` 和 `truncated`，因為兩者對演算法 bootstrap 的含義不同。[Gymnasium Env API](https://gymnasium.farama.org/api/env/)

機器手臂常見 action 層不是只有 motor torque：

- torque：最接近物理、自由度最高，也最難學與轉實機。
- joint position/velocity target：由底層 PD/servo 轉成 torque，容易訓練。
- Cartesian delta pose：由 IK 或 operational-space controller 轉成 joint command；robosuite 讓 controller 決定 action space 與 torque 映射。[robosuite controllers](https://robosuite.ai/docs/simulation/controller.html)

常見 observation 先用低維 proprioception（qpos、qvel、end-effector pose、object/goal pose），確定任務與演算法工作，再加 RGB/depth。FetchReach 類 goal task會明確提供 achieved goal 與 desired goal，reward 可選稀疏或距離型。[FetchReach](https://robotics.farama.org/envs/fetch/reach/)

演算法方面，PPO 常搭配很多平行環境；SAC/TD3 類 off-policy 方法重用 replay buffer，環境數可以較少；稀疏 goal task 常加 HER；有示範資料時可先 behavior cloning／imitation，再做 RL。你的演算法可以完全不同，但環境介面仍應提供相同的 reset/step/metrics 基本資料，這會讓比較基準與除錯容易很多。

## 建議的目標架構

```text
自訂 C 演算法 / baseline trainer
              │ contiguous observations/actions
              ▼
       robot_env 公開 C ABI
       ├─ task：obs / reward / done / success
       ├─ controller：torque / joint target / Cartesian target
       ├─ recorder：metadata / episode / selected trajectory
       └─ batch runner：N env + worker pool
                    │
                    ▼
      shared read-only mjModel + one mjData per env/worker
                    │
          ┌─────────┴─────────┐
          ▼                   ▼
   headless train        replay / viewer / offscreen video
```

MuJoCo 官方建議的 CPU sampling 模式正是共用唯讀 `mjModel`，每個 thread 使用自己的可寫 `mjData`；這不需要在每個 environment 建一份模型。[MuJoCo multithreading](https://mujoco.readthedocs.io/en/latest/programming/simulation.html#multi-threading) 若每個環境要 randomize mass、friction 等 `mjModel` 欄位，則要把相同 model variant 分組共用，或每個 variant 複製一份 model；不要讓多個 thread 同時改同一份 `mjModel`。

建置目標建議拆成：

- `robot_core`：static/shared library；只依賴 MuJoCo，不依賴 GLFW。
- `robot_train`：headless CLI；建立 batch、呼叫演算法、寫 log/checkpoint。
- `robot_replay`：讀 trajectory，選擇 interactive 或 offscreen 影片。
- `robot_viewer`：人工模型與 controller 除錯。
- `robot_tests`：模型、reset/step、seed、reward/termination 與 replay 測試。

MuJoCo 請維持外部 dependency，連結它的 library/DLL；除非真的要改 engine，不必把 MuJoCo 原始碼 vendor 進 repo。你的穩定邊界應是 `robot_env`，不是直接把 raw `mjData*` 當演算法 API。

## C 介面草案

```c
typedef struct robot_model robot_model;
typedef struct robot_batch robot_batch;

typedef struct {
    uint32_t abi_version;
    uint32_t num_envs;
    uint32_t observation_dim;
    uint32_t action_dim;
    uint32_t physics_substeps;
    double physics_dt;
    double policy_dt;
} robot_spec;

typedef struct {
    double reward;
    uint32_t episode_steps;
    uint8_t terminated;
    uint8_t truncated;
    uint8_t success;
    uint8_t reserved;
} robot_step_info;

int robot_model_load(const char *mjcf_path, robot_model **out,
                     char *error, size_t error_size);
int robot_batch_create(robot_model *model, uint32_t num_envs,
                       robot_batch **out);
int robot_batch_get_spec(const robot_batch *batch, robot_spec *out);
int robot_batch_reset(robot_batch *batch, const uint64_t *seeds,
                      double *observations);
int robot_batch_step(robot_batch *batch, const double *actions,
                     double *observations, robot_step_info *info);
int robot_batch_save_state(const robot_batch *batch, uint32_t env_id,
                           void *buffer, size_t *buffer_size);
int robot_batch_load_state(robot_batch *batch, uint32_t env_id,
                           const void *buffer, size_t buffer_size);
void robot_batch_destroy(robot_batch *batch);
void robot_model_destroy(robot_model *model);
```

陣列先固定 row-major `[num_envs][dim]`，ABI 加版本，handle 保持 opaque。錯誤用回傳碼與 caller-owned error buffer，不要在 library 內 `exit()`。若研究階段真的需要 MuJoCo escape hatch，可另提供名稱帶 `_unstable` 的 native handle getter，避免演算法不知不覺綁死 `mjData` layout。

狀態保存可直接利用 `mj_stateSize`、`mj_getState`、`mj_setState`，不要自行猜哪些 `qpos/qvel/act` 足夠。[MuJoCo state API](https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html#mj-getstate)

## 批次、紀錄與影片

### CPU batch

在這台 Intel Core Ultra 7 258V（8 cores/8 logical processors）上，以 MuJoCo 3.10.0 官方 `testspeed`、每 thread 200,000 steps、dt=0.002 測 corrected seven-cylinder：

| Threads | Aggregate steps/s | Aggregate realtime factor |
|---------|------------------:|--------------------------:|
| 1 | 30,707 | 61.41× |
| 2 | 63,308 | 126.62× |
| 4 | 72,612 | 145.22× |
| 8 | 61,649 | 123.30× |

本機單次測量的甜蜜點是 4 threads；不要預設 thread 數等於 logical core 就最快。profile 中 collision 約佔單步 74%，而這個長 rollout 會讓手臂接觸地面，故正式優化前要用實際 task、episode 長度與 controller 再量一次。MuJoCo 的 `testspeed` 本身也是用 shared model + per-thread data 模擬 RL sampling。[MuJoCo testspeed](https://mujoco.readthedocs.io/en/latest/programming/samples.html#testspeed)

### 紀錄分三層

1. Run metadata：git commit/dirty 狀態、model SHA-256、MuJoCo 版本、seed、timestep、solver、controller/action mode、演算法與超參數。
2. Episode summary：env id、seed、return、length、success、termination reason、最大 constraint/接觸指標、wall time。
3. Selected trajectory：初始 integration state、action 序列、必要的 observation/reward/event，以及每隔 K 步的 state checkpoint。

每一步全部寫 JSON/CSV 會拖慢 hot loop。建議 summary 用 JSONL/CSV，trajectory 用有 header/version 的 binary chunks；由獨立 writer thread 收 bounded queue，queue 滿時丟 debug sample，不可阻塞 physics。

### 影片策略

- train：完全不建立 OpenGL context。
- eval：固定 seeds，每 N 次更新只挑 1–4 個 episode。
- failure capture：NaN、solver warning、極端 reward 或成功率突降時保存 trajectory。
- replay：另開行程讀 trajectory，必要時從 checkpoint 校正，再用 offscreen renderer 輸出 RGB，最後 ffmpeg 壓成 MP4。

MuJoCo 官方 `record` 在 Windows 可用 invisible GLFW window；Linux server 可用 EGL 或 OSMesa。[MuJoCo offscreen rendering](https://mujoco.readthedocs.io/en/latest/programming/samples.html#record) 這代表 recorder 不該塞進每個 training env，只要 replay renderer 有一份即可。

## 物理模型要補什麼

近期先補「會影響演算法結論」的項目，不追求畫面：

1. 明確 mass、center of mass、inertia；最好來自 CAD/實測，不只靠預設 density。
2. joint damping、friction loss、armature、limit softness，並記錄來源。
3. actuator 的 torque/velocity limit、gear、servo gain、控制頻率、延遲與飽和。
4. 獨立 visual geom 與簡化 collision geom；建立允許接觸表，減少無意義 collision pair。
5. end-effector、target、contact/force、joint position/velocity 等 site/sensor。
6. task 物件、成功條件、失敗條件與 time limit。
7. 用實機資料做 system identification，再對 mass、friction、gain、delay 做有界 domain randomization。Isaac Lab 的 randomization 元件也集中處理 material、mass、gravity、actuator gain 等項目。[Isaac Lab events](https://isaac-sim.github.io/IsaacLab/v2.0.1/source/api/lab/isaaclab.envs.mdp.html)

不要一開始把 randomization 範圍開很大。先校準 nominal model，再讓範圍覆蓋量測不確定性；否則演算法可能只是在學一個過度含糊的世界。

## 建議實作順序

### P0：已完成

- 非侵入式 workflow 導入：根目錄只增加 `AGENTS.md`、`CLAUDE.md`，其餘在 `wf/`。
- seven-cylinder 幾何端點、pitch 軸、尺寸、control range 與 end-effector site 修正。
- CTest headless 模型測試，建置與測試通過。

### P1：先把單環境變成可訓練環境

可直接執行的 API、資料格式與 M0–M7 工作包見 [robot_core 詳細實作計畫](plans/robot_core/README.md)。

- `sim` 由 global singleton 改成 opaque instance/context。
- 建立 `robot_core` library，不連 GLFW。
- 定義 observation/action、policy dt/substeps、reset(seed)、reward、terminated/truncated/success。
- 加 `robot_train --headless --episodes ...`，先單環境跑 random policy。
- 修正 manual action 被 phase controller 覆蓋，並把示範 phase 移到 viewer/debug 層。

驗收：無視窗跑 10,000 episodes；相同 seed 的短 rollout 一致；無 NaN；錯誤可回傳、不會 `exit()`。

### P2：CPU batch 與實驗資料

- shared `mjModel` + N 個 `mjData`，固定 worker pool。
- contiguous batch action/observation buffer，partial reset。
- run metadata、episode summary、selected trajectory、checkpoint。
- 針對 1/2/4/8 env 做端到端 benchmark，包含演算法與紀錄成本。

驗收：batch=1 與 batch=N 的固定 action rollout 結果一致；writer 不阻塞 physics；可由紀錄重播任一 selected episode。

### P3：任務與物理可信度

- 先做 Reach：end-effector 到隨機 target；同時提供 dense 與 sparse reward。
- 再做含接觸的 Push/Pick；引入 gripper/object/controller layer。
- 補質量、慣量、摩擦、actuator/sensor 與 domain randomization。
- 用 Gymnasium/robosuite 或已知 baseline 做 sanity benchmark；你的演算法再與同一環境比較。

### P4：選擇性影片與 GPU 決策

- `robot_replay` + offscreen renderer + ffmpeg。
- 固定 evaluation seeds、自動保存 best/median/worst/failure。
- 只有 CPU throughput 或 vision rendering 經 profile 證明是瓶頸，才做 MJX/ManiSkill/Isaac backend spike。MJX 的 batch 維度很適合大量相同場景和 domain randomization，但 feature parity 與 C/JAX 邊界要另行驗證。[MJX feature parity](https://mujoco.readthedocs.io/en/latest/mjx.html#feature-parity)

## 最後判斷

物理引擎選擇沒有走錯；現在的專案是好用的原型 viewer，加上本次修正後可信度更高的七段測試模型。和成熟方案的距離主要是軟體架構與實驗工程，而不是 MuJoCo 本身。

最值得先做的一步不是 GPU、漂亮影片或更多 robot asset，而是把 `sim` 改成「無全域狀態、無 viewer 依賴、可 reset/step、可回錯誤」的 C library。這一步做好，你的自訂演算法、CPU batch、replay、Python binding，甚至未來換 GPU backend 都會變成外掛，而不必重寫核心。
