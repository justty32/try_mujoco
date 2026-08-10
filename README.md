# robot

用 C 語言控制 MuJoCo 的機器手臂物理模擬專案。目前主模型是七個圓柱組成的 6-DoF 串聯手臂：固定底座加三組 yaw / pitch 關節。

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

**必須從 project root 執行**，讓相對路徑 `assets/` 可以解析：

```bat
build\robot.exe
```

## 測試

模型測試不建立視窗，會用 MuJoCo 實際載入 `seven_cylinders.xml`，檢查串聯拓撲、關節軸、零姿態連接位置與短步進的有限數值：

```bat
ctest --test-dir build --output-on-failure
```

## 專案結構

```
robot/
├── assets/
│   └── seven_cylinders.xml   # 7 個圓柱、6 個 yaw/pitch 關節
├── src/
│   ├── main.c                # 主迴圈（< 50 行）
│   ├── sim.h / sim.c         # MuJoCo 模型生命週期、狀態查詢
│   ├── controller.h / .c     # Motor 控制、phase 切換
│   └── viewer.h / viewer.c   # GLFW 視窗、渲染、HUD、input callback
├── tests/
│   └── seven_cylinders_test.c # Headless MJCF 拓撲／座標測試
├── docs/
│   ├── index.html             # API 文件（瀏覽器直接開啟）
│   └── simulation_environment_report.md
├── wf/                       # 非侵入式 agent 工作流與專案導航
├── CMakeLists.txt
└── build.bat
```

## 操作說明

### 鍵盤

| 按鍵 | 動作 |
|------|------|
| `1` | 三個 yaw motor 正向 |
| `2` | 三個 pitch motor 正向 |
| `3` | 六個 motor 反向 |
| `Space` | 暫停 / 繼續 |
| `Backspace` | 重置模擬 |
| `Esc` | 離開 |

### 滑鼠

| 操作 | 動作 |
|------|------|
| 左鍵拖曳 | 旋轉視角 |
| 右鍵拖曳 | 平移視角 |
| 滾輪 | 縮放 |

## API 速查

```c
/* 狀態監控 */
sim_joint_angle_deg("yaw1");           // 關節角度（°）
sim_joint_vel_deg("pitch3");           // 角速度（°/s）
sim_body_pos("arm3", pos);             // 世界座標 {x, y, z}
sim_actuator_force(0);                  // yaw1_motor 輸出力

/* 馬達控制 */
ctrl_set_phase(1);                     // 切換 preset（1/2/3）
double action[6] = {0.5, -0.3, 0, 0, 0, 0};
ctrl_set_motors(action);               // 直接寫入六個 motor
```

詳細 API 說明見 [`docs/index.html`](docs/index.html)。大量訓練前仍應依報告建議把全域 singleton 改成 instance-based C library。
