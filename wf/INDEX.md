# INDEX — robot 專案地圖

← [AGENTS.md](../AGENTS.md)｜[WORKFLOWS.md](WORKFLOWS.md)

robot 是以 C11 封裝 MuJoCo、用於機器手臂物理模擬與演算法實驗的專案。

## Repo 佈局

| 路徑 | 內容 |
|------|------|
| `src/` | 模擬生命週期、控制器、互動 viewer 與程式入口 |
| `assets/` | MuJoCo MJCF 模型與場景 |
| `docs/` | API 文件、模擬環境評估報告與實作計畫 |
| `build/` | CMake 產物；不作為原始碼真相來源 |
| `CMakeLists.txt` | MuJoCo、GLFW、OpenGL 的建置設定 |
| `build.bat` | Windows/MinGW 建置入口 |
| `run.bat` | 固定正確工作目錄後啟動互動 viewer |
| `tests/` | 六軸圓柱模型、核准外形／幾何間隙與關節姿態控制 headless 驗證 |
| `wf/` | 非侵入式工作流、導航與 open 狀態 |

## 工作流與共通知識

- 工作流選擇見 [WORKFLOWS.md](WORKFLOWS.md)。
- 跨工作流的慣例與踩坑見 [workflows/common/README.md](workflows/common/README.md)。
- 結構整理原則見 [DEV-GUIDE.md](DEV-GUIDE.md)，只在需要整理時讀。

## 活狀態

| 檔案 | 用途 |
|------|------|
| [SESSION-LOG.md](SESSION-LOG.md) | agent 手上的未完成工作 |
| [WAIT_USER.md](WAIT_USER.md) | 需要使用者親自驗證或處理的工作 |
