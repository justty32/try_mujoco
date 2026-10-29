# robot

用 C 語言控制 MuJoCo 的機器手臂物理模擬專案。目前主模型是六關節叉耳式串聯手臂：固定底座加三組 yaw / pitch 關節。沿用歷史檔名 `seven_cylinders.xml`，外形已改為固定座、薄轉盤、叉耳、轉軸與細連桿。

先看 [外形修改前後比較](docs/reports/geometry-2026-10-29/preview.html)：同視角的全貌、關節近景與組合姿態，全部為真實 MuJoCo 模型渲染。

開啟程式後預設以關節姿態模式保持零角度。可逐一調整六個關節的目標角，機械臂會移向目標並持續保持；原本的定力矩示範仍可用 `1`／`2`／`3` 切換。

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

測試不建立視窗。模型測試會用 MuJoCo 實際載入 `seven_cylinders.xml`，檢查串聯拓撲、關節軸、零姿態連接位置與短步進的有限數值；姿態控制測試檢查目標收斂、持續保持、限位／限幅，以及切回舊模式的控制值與範圍：

```bat
ctest --test-dir build --output-on-failure
```

本輪建置與三項 headless 測試通過，涵蓋模型／幾何、零姿態抵抗重力、組合目標收斂與保持、捕捉目前姿態、回零／重置、限位／限幅與舊 preset 命令保留。圖像已經真實模型軟體渲染核對，互動 GUI 的鍵盤操作尚未實測。詳見 [驗證紀錄](docs/reports/geometry-2026-10-29/README.md#建置與動態回歸)。

## 專案結構

```
robot/
├── assets/
│   └── seven_cylinders.xml   # 六關節叉耳式模型，沿用歷史檔名
├── src/
│   ├── main.c                # 主迴圈（< 50 行）
│   ├── sim.h / sim.c         # MuJoCo 模型生命週期、狀態查詢
│   ├── controller.h / .c     # 關節目標／姿態保持、定力矩 phase
│   └── viewer.h / viewer.c   # GLFW 視窗、渲染、HUD、input callback
├── tests/
│   ├── seven_cylinders_test.c # Headless MJCF 拓撲／座標測試
│   ├── controller_pose_test.c # 關節姿態追蹤／保持與控制邊界測試
│   └── arm_geometry_test.c    # 裝配間隙、碰撞與幾何邊界測試
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

1. 雙擊 `run.bat`（或從專案根目錄啟動 `build\robot.exe`），確認 HUD 顯示 `POSE HOLD`。
2. 按 `F2` 選 `pitch1`，按三次 `→`，目標由 0° 變成 +15°，第一支臂會抬起並保持。
3. 按 `F3` 選 `yaw2`，按兩次 `→`，把第二組旋轉盤轉向 +10°。HUD 的 `target` 是目標，`actual` 是目前角度，`>` 表示選中的關節。
4. 用 `Shift`＋方向鍵做 1° 細調；`Home` 讓六個目標回到 0°，`P` 則捕捉目前角度並保持。

這是**關節角控制**：直接指定每個關節的角度，尚未提供末端位置／姿態的反向運動學。目標受到模型關節範圍限制（yaw ±90°、pitch ±60°）；有地面或自身碰撞時，不保證能到達組合姿勢。

### 鍵盤

| 按鍵 | 動作 |
|------|------|
| `F1`–`F6` 或 `↑`／`↓` | 選擇 yaw1、pitch1、yaw2、pitch2、yaw3、pitch3 |
| `←`／`→` | 目標角 −5°／+5°；可按住重複調整，並切回姿態模式 |
| `Shift`＋`←`／`→` | 目標角 −1°／+1° |
| `P` | 捕捉目前六個角度，切到姿態保持 |
| `Home` | 六個目標回零，平順移向零姿態 |
| `1`／`2`／`3` | 切回原定力矩示範：yaw 正向／pitch 正向／全部反向 |
| `Space` | 暫停 / 繼續 |
| `Backspace` | 重置模擬、回零目標並進入姿態模式；保留暫停狀態 |
| `Esc` | 離開 |

### 滑鼠

| 操作 | 動作 |
|------|------|
| 左鍵拖曳 | 旋轉視角 |
| 右鍵拖曳 | 平移視角 |
| 滾輪 | 縮放 |

### 姿態保持的範圍

姿態模式使用質量矩陣縮放的 PD（位置誤差與速度回授），加上偏置／被動力補償：依模型的慣量、重力與阻尼把目標加速度轉成馬達力矩，並配合 Euler 步進的隱式阻尼作等效慣量修正，不補償接觸力。內部目標以最多 30°/s 移動，再經力矩限幅；這個數字是目標斜坡速率，不是碰撞或切換瞬間的實際關節速度保證。暫停時可先改目標，恢復後才移動。

目前細連桿模型零姿態下的 `pitch1` 約需 1.13 N·m 抵抗重力，原馬達 10 N·m 的上限已足夠。所有模式一律使用 `ctrlrange = [-1, 1]`，各組 yaw／pitch 的 gear 分別為 10／9／8；先前針對粗圓柱模型的姿態模式 ±40 N·m 特例已移除。舊 preset 的控制值不變，但幾何、質量與慣量變了，實際運動不會與舊模型完全相同。

模型的實體同時參與顯示與碰撞，沒有關閉碰撞來掩蓋裝配問題。六個關節的中心、軸、範圍與末端座標保留；密度明設為 1000 kg/m³，由 MuJoCo 計算質量與慣量。這是教學原型，不是已校準硬體。有限姿態取樣不能保證所有組合都無碰撞；例如 `pitch1 = -60°`、其餘為零時仍會碰到基座與地面。詳細邊界與原始數據見 [幾何驗證紀錄](docs/reports/geometry-2026-10-29/README.md)。

## API 速查

```c
/* 狀態監控 */
sim_joint_angle_deg("yaw1");           // 關節角度（°）
sim_joint_vel_deg("pitch3");           // 角速度（°/s）
sim_body_pos("arm3", pos);             // 世界座標 {x, y, z}
sim_actuator_force(0);                  // yaw1_motor 輸出力

/* 關節姿態：單位為度，順序 yaw1/pitch1/yaw2/pitch2/yaw3/pitch3 */
double pose[6] = {0, 15, 10, 0, 0, 0};
ctrl_set_pose_deg(pose);              // 設定六個目標並進入姿態模式
ctrl_set_target_deg(1, 20);           // pitch1 目標改為 +20°
ctrl_hold_current();                 // 捕捉目前角度並保持
ctrl_zero_targets();                 // 平順移向六個零目標

/* 原定力矩控制 */
ctrl_set_phase(1);                    // 切換 preset（1/2/3），退出姿態模式
double action[6] = {0.5, -0.3, 0, 0, 0, 0};
ctrl_set_motors(action);              // 直接寫入；下一次 ctrl_apply() 會覆蓋
```

詳細 API 說明見 [`docs/index.html`](docs/index.html)。大量訓練前仍應依報告建議把全域 singleton 改成 instance-based C library。
