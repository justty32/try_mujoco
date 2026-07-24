# try_mujoco

用 C 語言控制 MuJoCo 模擬的練習專案，目標模型為三段圓柱手臂（yaw / pitch 兩個關節）。

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

## 專案結構

```
try_mujoco/
├── assets/
│   └── three_cylinders.xml   # 目標模型：底座 + yaw + pitch 三段圓柱
├── src/
│   ├── main.c                # 主迴圈（< 50 行）
│   ├── sim.h / sim.c         # MuJoCo 模型生命週期、狀態查詢
│   ├── controller.h / .c     # Motor 控制、phase 切換
│   └── viewer.h / viewer.c   # GLFW 視窗、渲染、HUD、input callback
├── docs/
│   └── index.html            # API 文件（瀏覽器直接開啟）
├── CMakeLists.txt
└── build.bat
```

## 操作說明

### 鍵盤

| 按鍵 | 動作 |
|------|------|
| `1` | Yaw motor +1.0 |
| `2` | Pitch motor +0.5 |
| `3` | 兩者反向 |
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
sim_joint_angle_deg("yaw_joint");      // 關節角度（°）
sim_joint_vel_deg("pitch_joint");      // 角速度（°/s）
sim_body_pos("pitch_cylinder", pos);   // 世界座標 {x, y, z}
sim_actuator_force(0);                 // yaw_motor 輸出力

/* 馬達控制 */
ctrl_set_phase(1);                     // 切換 preset（1/2/3）
ctrl_set_motors(0.5, -0.3);           // 直接設定 yaw / pitch
```

詳細說明見 [`docs/index.html`](docs/index.html)。
