# robot

用 C 語言控制 MuJoCo 的機器手臂物理模擬專案。正式模型為 `assets/six_axis_arm.xml`：9 個圓柱組成典型 6R 手臂，依序是底座 yaw、肩／肘 pitch，以及緊湊的腕 roll／pitch／roll。

外形與展示姿態已獲使用者核准，見 [原型預覽](docs/designs/six_axis_arm/prototype-v1/preview.png) 與 [原型保存說明](docs/designs/six_axis_arm/prototype-v1/README.md)。正式模型沿用核准的比例、外形與旋轉軸，補上馬達與 `home` keyframe。

正式接入後的 [模型預覽](docs/reports/six-axis-integration-2026-10-29/preview.png) 與 [驗證紀錄](docs/reports/six-axis-integration-2026-10-29/README.md) 另存，與未推進模擬的原型圖分開。

開啟程式後預設以關節姿態模式保持 home：`[0, 65, -100, 0, 35, 0]°`。可逐一調整六個關節的目標角，機械臂會移向目標並保持；`1`／`2`／`3` 可切換成定力矩示範。

目前程式適合模型與控制器的互動除錯；批次 headless 訓練、穩定 C ABI、紀錄與重播的現況評估見 [`docs/simulation_environment_report.md`](docs/simulation_environment_report.md)，可直接動工的詳細設計與里程碑見 [`docs/plans/robot_core/`](docs/plans/robot_core/README.md)。

## 環境需求

| 工具 | 版本 | 備註 |
|------|------|------|
| [MuJoCo](https://github.com/google-deepmind/mujoco/releases) | 3.x | 安裝至 `C:/dev/mujoco` |
| [MinGW-w64](https://www.mingw-w64.org/) | GCC 16+ | 安裝至 `C:/dev/mingw64` |
| [CMake](https://cmake.org/) | 3.20+ | 需加入 PATH |
| Git | — | FetchContent 下載 GLFW 需要網路 |

> GLFW 會在第一次 cmake configure 時自動從 GitHub 下載並編譯，無需手動安裝。

## 建置

```bat
build.bat
```

或手動執行：

```bat
set PATH=C:\dev\mingw64\bin;%PATH%
cmake -G "MinGW Makefiles" -DCMAKE_C_COMPILER=C:\dev\mingw64\bin\gcc.exe -B build -S .
cmake --build build
```

## 執行

建置後可直接雙擊 `C:\code\robot\run.bat`；它會先切換到專案根目錄，讓相對路徑 `assets/` 可以解析。也可在命令提示字元執行：

```bat
cd /d C:\code\robot
build\robot.exe
```

## 測試

測試不建立視窗，會由 MuJoCo 載入正式 `six_axis_arm.xml`。`six_axis_arm_model` 檢查模型拓撲、關節軸、home 與馬達；`controller_pose` 檢查追蹤／保持、回 home、重置、限位／限幅；`arm_geometry` 檢查有限取樣的幾何與接觸邊界。

```bat
ctest --test-dir build --output-on-failure
```

正式接入的建置與三項 CTest 全通過（0.48 秒），涵蓋 home／五個無引擎接觸的靜態樣本，以及控制追蹤、保持、捕捉姿態、全零目標、回 home、重置、限位／限幅與真實碰撞阻擋時的有限數值。這些是有限範圍的驗證；原型圖本身沒有推進模擬，互動 GUI 的鍵盤操作也尚未實測。詳見 [正式驗證紀錄](docs/reports/six-axis-integration-2026-10-29/README.md)。

## 專案結構

```
robot/
├── assets/
│   └── six_axis_arm.xml      # 核准的 9 圓柱 6R 正式模型
├── src/
│   ├── main.c                # 主迴圈（< 50 行）
│   ├── sim.h / sim.c         # MuJoCo 模型生命週期、狀態查詢
│   ├── controller.h / .c     # 關節目標／姿態保持、定力矩 phase
│   └── viewer.h / viewer.c   # GLFW 視窗、渲染、HUD、input callback
├── tests/
│   ├── six_axis_arm_test.c   # 正式模型／home／軸與馬達測試
│   ├── controller_pose_test.c # 關節姿態追蹤／保持與控制邊界測試
│   └── arm_geometry_test.c   # 有限姿態取樣與真實碰撞邊界測試
├── docs/
│   ├── index.html             # API 文件（瀏覽器直接開啟）
│   └── simulation_environment_report.md
├── wf/                       # 非侵入式 agent 工作流與專案導航
├── CMakeLists.txt
├── build.bat
└── run.bat                   # 從任意工作目錄啟動 viewer
```

## 操作說明

### 先試一個姿勢

1. 雙擊 `run.bat`，確認 HUD 顯示 `POSE HOLD`，六個目標為 home：`[0, 65, -100, 0, 35, 0]°`。
2. 按 `F1` 選 `J1 Base`，按兩次 `→`，把底座目標由 0° 調到 +10°。
3. 按 `F2` 選 `J2 Shoulder`，按一次 `→`，把肩部目標由 +65° 調到 +70°。HUD 的 `target` 是目標、`actual` 是目前角度，`>` 表示選中關節。
4. 用 `Shift`＋方向鍵做 1° 細調；`Home` 平順回到 home，`P` 捕捉目前角度並保持。

這是**關節角控制**，不是末端反向運動學。關節角度限制不等於碰撞安全範圍；任意組合或移動路徑可能碰到自身／地面，控制器不保證穿越障礙到達目標。

| 關節與 HUD | 功能 | 角度範圍 | home |
|---|---|---:|---:|
| `J1_base_yaw`／J1 Base | 底座旋轉 | −180°～180° | 0° |
| `J2_shoulder_pitch`／J2 Shoulder | 肩部俯仰 | −100°～130° | 65° |
| `J3_elbow_pitch`／J3 Elbow | 肘部俯仰 | −145°～145° | −100° |
| `J4_wrist_roll`／J4 Wrist roll | 前臂方向旋轉 | −180°～180° | 0° |
| `J5_wrist_pitch`／J5 Wrist pitch | 手腕俯仰 | −45°～45° | 35° |
| `J6_tool_roll`／J6 Tool roll | 工具法蘭旋轉 | −180°～180° | 0° |

J6 的工具法蘭是旋轉對稱圓柱，roll 時外形與末端位置可能看起來沒變；請看 HUD 的 `actual` 角度。這不代表關節沒轉，也沒有為了顯示旋轉額外改動核准的九圓柱外形。

J5 的正式限位依腕部幾何收為 ±45°：邊界間隙約 2.373 mm，±50° 僅約 0.487 mm，±52° 已開始相交。原型的 ±120° 未作此驗證；home 的 35° 與核准外形保持不變。這項局部間隙限制仍不能保證全手臂任意組合都無碰撞。

### 鍵盤

| 按鍵 | 動作 |
|------|------|
| `F1`–`F6` 或 `↑`／`↓` | 依序選擇 J1–J6 |
| `←`／`→` | 目標角 −5°／+5°；可按住重複調整，並切回姿態模式 |
| `Shift`＋`←`／`→` | 目標角 −1°／+1° |
| `P` | 捕捉目前六個角度，切到姿態保持 |
| `Home` | 六個目標平順回到 home；不是全零角度 |
| `1`／`2`／`3` | 定力矩示範：J1/J4/J6 底座／roll、J2/J3/J5 pitch、全部反向 |
| `Space` | 暫停 / 繼續 |
| `Backspace` | 重置模擬到 home 並進入姿態模式；保留暫停狀態 |
| `Esc` | 離開 |

### 滑鼠

| 操作 | 動作 |
|------|------|
| 左鍵拖曳 | 旋轉視角 |
| 右鍵拖曳 | 平移視角 |
| 滾輪 | 縮放 |

### 姿態保持的範圍

姿態模式使用質量矩陣縮放的 PD（位置誤差與速度回授），加上偏置／被動力補償：依模型的慣量、重力與阻尼把目標加速度轉成馬達力矩，並配合 Euler 步進的隱式阻尼作等效慣量修正，不補償接觸力。內部目標以最多 30°/s 移動，再經力矩限幅；這個數字是目標斜坡速率，不是碰撞或切換瞬間的實際關節速度保證。暫停時可先改目標，恢復後才移動。

所有模式一律使用 `ctrlrange = [-1, 1]`。J1–J6 的 gear 分別為 `[12, 20, 12, 3, 3, 2]`，對應本模型的力矩上限（N·m）；這是教學額定值，不是硬體校準。密度 1000 kg/m³、關節 damping 0.1 沿用核准原型。原本叉耳模型的座標、負載及 preset 分組不適用本模型。

`Home` 與 `Backspace` 都以 XML 的 `home` keyframe 為準；XML 的 qpos 用弧度儲存，畫面和角度設定 API 使用度。`ctrl_zero_targets()` 仍代表字面上的六個全零目標，與回 home 不同。真實碰撞保留，有限取樣不等於整個關節範圍都可達。

## API 速查

```c
/* 狀態監控：具名查詢使用完整名稱 */
sim_joint_angle_deg("J1_base_yaw");       // 關節角度（°）
sim_joint_vel_deg("J5_wrist_pitch");      // 角速度（°/s）
sim_body_pos("tool_roll", pos);          // body 原點，並非 tool_center site
sim_actuator_force(0);                  // J1 actuator 純量輸出，尚未乘 gear

/* 關節姿態：單位為度，順序 J1、J2、J3、J4、J5、J6 */
double pose[6] = {10, 65, -100, 0, 35, 0};
ctrl_set_pose_deg(pose);                 // 設定六個目標並進入姿態模式
ctrl_set_target_deg(1, 70);              // J2 肩部目標改為 +70°
ctrl_hold_current();                    // 捕捉目前角度並保持
ctrl_home_targets();                    // 平順回到核准的 home
ctrl_zero_targets();                    // 全零目標；與 home 不同

/* 定力矩控制 */
ctrl_set_phase(1);                      // J1/J4/J6 示範，退出姿態模式
double action[6] = {0.25, 0, 0, 0.10, 0, 0.10};
ctrl_set_motors(action);                 // 直接寫入；下一次 ctrl_apply() 會覆蓋
```

詳細 API 說明見 [`docs/index.html`](docs/index.html)。大量訓練前仍應依報告建議把全域 singleton 改成 instance-based C library。
