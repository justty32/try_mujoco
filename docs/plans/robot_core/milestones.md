# 實作里程碑與驗收

← [計畫入口](README.md)

每個里程碑是一個可獨立 review/merge 的 change set；前一階段測試保持綠燈才進下一階段。檔案名稱是目標名稱，可在實作時依 300 行規則微調，但模組邊界不得倒退。

## M0 — 行為基線與契約凍結

目的：在重構前先把現在「應保持」的行為變成測試，避免 instance migration 只靠肉眼。

工作：

- 擴充現有 model test：記錄 model dimensions、joint/actuator name/order、timestep、control range、初始 site pose。
- 加固定 100-step action sequence golden test；保存預期 qpos/qvel tolerance，不保存平台敏感的完整 `mjData`。
- 加 error-path cases：不存在 XML、錯誤 joint/site、錯誤 actuator count。
- 在 public plan 中凍結 ABI v1、observation order、frame_skip、reset/terminal semantics。
- CMake 加 Debug/Release test presets；不改 runtime 行為。

主要檔案：`tests/model_test.c`、`tests/baseline_rollout_test.c`、本計畫。

Done when：現行程式與模型通過新增基線；測試真的能在故意改 joint order、axis 或 timestep 時失敗。

## M1 — CMake 分層與 `robot_core` 骨架

目的：先建立 renderer-free build boundary，再搬狀態。

工作：

- 建 `include/robot/` 公開 headers、export macro、version/status/error 型別。
- 新增 `robot_core` library target；MuJoCo include/link 設為 `PUBLIC/PRIVATE` 正確範圍。
- `find_package(Threads REQUIRED)`；GLFW `FetchContent` 移入 `if(ROBOT_BUILD_RENDERING)`。
- 建 helper function 複製 `mujoco.dll` 到各 executable；避免每 target 重複 custom command。
- 新增 `public_abi_test.c`，只 include public headers 並 link library。
- 保留現有 `robot.exe` 行為；此階段 core 可以只有 version/error/model load skeleton。

Done when：`ROBOT_BUILD_RENDERING=OFF` 能從 clean build 完成 core/tests/replay-verify，而且 configure 不下載 GLFW；現有 viewer build/CTest 仍通過。

## M2 — Instance-based model/env 與 viewer migration

目的：移除 global singleton，讓兩個環境可同時存在。

工作包 A，model：

- `robot_model_load` 用 `mj_loadXML`，失敗填 `robot_error`，不得 `exit()`。
- 驗證模型 shape/name/type；cache joint/actuator/end-effector IDs 與 qpos/dof address。
- 產生 action/observation schema、compiled MJB SHA-256、MuJoCo version。
- 實作 atomic retain/release；load 後不再修改 `mjModel`。

工作包 B，env：

- `robot_env_create` 配置一份 `mjData`、RNG、episode/task state。
- reset/step/state/getter 全接受 env handle；所有 buffer 做 count/NULL/finite 檢查。
- 實作 `mjSTATE_INTEGRATION` snapshot envelope；task/RNG state 同步保存。
- 把目前 getter 改成 cached ID，不在每 observation 呼叫 `mj_name2id`。

工作包 C，遷移：

- `viewer` init 接受 env/private render view，不再呼叫 global `sim_model/sim_data`。
- phase controller 移到 `apps/policies/demo_policy.c`；core step 的 action 不會被背景 phase 覆蓋。
- `robot_viewer` CMake target 暫以 `OUTPUT_NAME robot` 保留 `build/robot.exe`。
- 所有 caller port 完後刪除 `src/sim.c/.h` 與舊 controller global state；不長期留雙 API。

測試：

- 建兩個 env，step A 不改 B；reset B 不改 A。
- 反覆 create/destroy 1,000 次；Debug ASan 無 leak/use-after-free。
- invalid path/shape/action/state 都回 status，測試行程不中止。
- snapshot → step → restore → 同 action，結果一致。

Done when：source 中沒有 static global `mjModel*`/`mjData*`；viewer 行為保持；外部 ABI test 能完成 single-env rollout。

## M3 — `seven_reach_v1` 與 headless runner

目的：讓單環境在沒有 viewer 的情況下完成真正 episode contract。

工作：

- 實作 per-env SplitMix64/PCG 類固定 RNG；不得使用 libc `rand()`。
- 實作安全 joint sampling → FK target、初始 state、21 維 observation、dense reward、success streak、terminated/truncated。
- MuJoCo warnings 與 qpos/qvel/ctrl finite check 映射到 `robot_step_result`。
- `robot_headless` CLI：`--model --task --episodes --seed --max-steps --frame-skip --policy zero|random|phase --output`。
- CLI 印 resolved config 與週期性 summary；尚未啟用 recorder 時可只 stdout。
- signal/使用者停止在 policy step boundary 結束；完成已開始 episode 的 summary reason=`user_stop`。

測試：

- 同 seed reset observation 完全相同；不同 seed 目標不同且在定義 workspace 內。
- reward/distance、success streak、terminated/truncated 邊界測試。
- policy dt 必須等於 physics dt × integer frame_skip。
- `robot_headless --episodes 10000 --policy random` 不開視窗、不出 NaN、不 leak。
- CMake/link audit 證明 headless binary 不連 GLFW/OpenGL。

Done when：single env 可完整跑 10,000 episodes；終止後 step 會拒絕，reset 後正常；相同 seed/action 重跑一致。

## M4 — CPU batch 與 fixed worker pool

目的：在同一行程中同步推進 N 個互不干擾的環境。

工作：

- `robot_batch_create` 建 N env、contiguous metadata/result 與固定 worker pool。
- 優先 C11 `thrd_t + mtx_t + cnd_t`；CMake 先 compile-test MinGW 支援。若 toolchain 缺 C11 threads，使用相同 private worker-pool API 的 Win32/pthread backend，不把 OpenMP runtime 暴露給 public ABI。
- 每一 generation 由 static ranges 分配 env；main caller 等 completion barrier。
- 實作 row-major action/observation stride、masked reset、per-env error status。
- 不在 worker 做 I/O、malloc、render 或使用全域 RNG/callback state。
- `worker_count=0` auto 先選 `min(num_envs, logical_processors)`；CLI 可覆寫，benchmark 決定建議值。

測試：

- batch=1 與 single-env API，同 seed/action 每一步相同。
- worker=1/2/4 與不同 static partition，同一 env 的結果相同。
- masked reset 只改 mask env；terminal observation 在 reset 前保留。
- 任一 env 注入 invalid action 時其他 env 不被破壞，batch 回 partial failure。
- create/destroy、zero env、stride 太小、worker startup failure 路徑。

Benchmark：

- `num_envs=1,2,4,8,64`、workers=1/2/4/8，分別測 physics-only、含 task、含 summary aggregation。
- 記 steps/s、episodes/s、p50/p95 policy-step latency、RAM/env。
- 效能不作跨硬體 CI gate；本機 4-worker aggregate 至少不應比單 worker 差，若低於 1.5×先 profile。

Done when：worker 數改變不改變數值結果；thread pool 不每 step 重建；本機 batch 有實際 throughput 增益。

## M5 — Recorder

目的：保留每個 episode 的可比較資料，只對選定 episode 寫 trajectory。

工作：

- 安全建立 run directory，寫 resolved `run.json`、compiled `model.mjb` 與 SHA-256。
- episode boundary append `episodes.jsonl`，固定 reason/schema。
- 每 env 預配置 episode action/event/checkpoint buffer；限制最大 RAM。
- 實作 selector：periodic、env_ids、success/failure/numeric、return threshold、best/worst K。
- `.rtraj` 逐欄 little-endian encoder、chunk CRC32、size/overflow limits。
- selected buffer 交給單 writer thread；`.partial` 完成後 atomic rename。
- Queue policy 與 dropped flag 明確；physics worker 永不碰 filesystem。

測試：

- JSON escape、完整 newline、run directory collision。
- binary golden header、round-trip、CRC、truncated/oversized chunk、unknown required chunk。
- crash 模擬後 `.partial` 不被當完成 trajectory。
- summary count 等於 completed episodes；selector 數量與條件正確。
- summary-only logging 相對 no logging median overhead <5%。

Done when：10,000-episode batch run 可查每個 summary，selected trajectory 完整，未選 episode 不產生 step-level 檔案。

## M6 — Replay、seek 與 offscreen video

目的：用紀錄重現指定 episode，必要時才渲染影片。

工作包 A，verify replay：

- 載入 header/model.mjb，驗 hash/version/ABI/schema。
- restore initial `mjSTATE_INTEGRATION` + task state，逐 action step。
- checkpoint 比對、`--seek`、max error 報告與非零 exit code。
- `--allow-mismatch` 只允許人工診斷並標註 non-authoritative。

工作包 B，render：

- 抽出 renderer 接口供 interactive viewer 與 offscreen 共用。
- Windows hidden GLFW context；`mjr_setBuffer(mjFB_OFFSCREEN)`、固定 simulation-time FPS、`mjr_readPixels`。
- target/episode metrics 由 decorative overlay 畫出，不修改 physics model。
- 先支援 raw RGB；再以安全 process argv 呼叫 ffmpeg 產 MP4。
- camera/resolution/FPS 寫 sidecar JSON；video 不影響 verify result。

測試：

- record → replay final snapshot round-trip；checkpoint drift 超標必定失敗。
- seek 0/middle/end 與無 checkpoint fallback。
- model hash/version mismatch、missing MJB、corrupt action block。
- 1 秒 30 FPS 輸出恰為 30 frames，解析度/檔案大小正確。
- offscreen mode 不顯示視窗；沒有 render dependency 時 verify mode 仍可建置執行。

Done when：任一 selected episode 可驗證、seek、interactive 看或產 MP4；train/headless target 完全不建立 GL context。

## M7 — Hardening、安裝與文件

工作：

- Debug `-Wall -Wextra -Wpedantic`、ASan；Linux CI 後續加 UBSan/Valgrind。
- Public headers 的 C/C++ include test、symbol export test、ABI struct size test。
- CMake install/export：headers、library、package config；外部 `examples/custom_algorithm.c` 用 installed package build。
- Stop/shutdown、writer error、disk full、thread creation failure、OOM 等 fault injection。
- 更新 README、HTML API、code map、測試工作流；將本計畫逐項標 completed，全部完成後移入 archive。

Done when：fresh clone 可選 core-only 或 viewer build；外部 sample 可 link/run；完整 Definition of Done 全部有自動或明確人工證據。

## 驗證命令矩陣

實作過程至少維持這些入口：

```powershell
# 現有完整 Windows build；遷移期間保持相容
cmd /c build.bat
ctest --test-dir build --output-on-failure

# 真正 core-only；不得 configure GLFW
cmake -S . -B build-core `
  -G "MinGW Makefiles" `
  -DCMAKE_C_COMPILER=C:\dev\mingw64\bin\gcc.exe `
  -DCMAKE_BUILD_TYPE=Release `
  -DROBOT_BUILD_RENDERING=OFF `
  -DROBOT_BUILD_VIEWER=OFF `
  -DROBOT_BUILD_REPLAY=ON `
  -DROBOT_BUILD_TESTS=ON
cmake --build build-core
ctest --test-dir build-core --output-on-failure

# 單環境與 batch smoke
build\robot_headless.exe --task seven_reach_v1 --episodes 10000 `
  --num-envs 1 --workers 1 --seed 42 --policy random
build\robot_headless.exe --task seven_reach_v1 --episodes 10000 `
  --num-envs 64 --workers 4 --seed 42 --policy random --output runs\smoke

# 重播驗證與影片
build\robot_replay.exe --verify runs\smoke\trajectories\sample.rtraj
build\robot_replay.exe --video sample.mp4 --fps 30 `
  runs\smoke\trajectories\sample.rtraj
```

Core-only build 使用獨立 build directory，不覆蓋開發者現有 `build/`。CI 的 correctness tests 不以固定 steps/s 判定；performance command 產出 machine-readable benchmark JSON 供趨勢比較。

## 風險與對策

| 風險 | 對策 |
|------|------|
| MuJoCo global callback 與多 env 衝突 | v1 不依賴 callback；控制明確寫入每個 `mjData` |
| Domain randomization 想改 shared model | v1 禁止 worker 改 model；後續以 model variants 分組或 copy model |
| Worker scheduling 造成 RNG 漂移 | RNG 屬於 env，seed 由 run/env/episode 明確導出，不使用 worker id |
| Terminal observation 被 reset 覆蓋 | v1 不 auto-reset；caller 先消費 result/obs，再 masked reset |
| I/O 拖慢 hot loop | worker 不 I/O；trajectory episode boundary enqueue；summary 低頻 buffered |
| Replay 因版本/模型變動漂移 | 保存 MJB/hash/version/integration checkpoints；預設 mismatch fail closed |
| Shared library ABI 被 struct padding 破壞 | fixed-width、`struct_size`、binary 不直接 fwrite struct、ABI tests |
| Offscreen context 有平台差異 | renderer optional；Windows hidden GLFW 先落地，Linux backend 分開 |
| 一次重構過大難 review | 嚴格按 M0–M7，小步保持 viewer/CTest 綠燈 |
