# 程式碼慣例與導航維護

← [common/README.md](README.md)｜[INDEX.md](../../INDEX.md)

## 程式碼慣例

- 使用 C11；公開介面放 `.h`，實作狀態留在 `.c`，避免讓 viewer 成為模擬核心的必要依賴。
- MuJoCo 名稱、actuator 順序或模型拓撲變更時，同步搜尋並更新 `src/`、`assets/`、README 與 API 文件。
- 不以 magic index 假設模型永久不變；新增外部演算法 API 時，使用具名 ID、尺寸查詢與明確錯誤碼。
- 變更 MJCF 後必須由 MuJoCo 實際載入；串聯幾何另檢查零姿態 joint/site 世界座標。
- 新增或刪除主要原始碼、模型、建置目標時，同步更新 [INDEX.md](../../INDEX.md) 與根 README 的結構說明。

## 目前 code map

| 區域 | 主要檔案 |
|------|----------|
| 程式入口與互動迴圈 | `src/main.c` |
| MuJoCo 模型/data 生命週期與狀態查詢 | `src/sim.h`, `src/sim.c` |
| 動作輸入與示範控制 | `src/controller.h`, `src/controller.c` |
| GLFW 視窗、相機、HUD | `src/viewer.h`, `src/viewer.c` |
| MJCF 場景 | `assets/*.xml` |
| 建置 | `CMakeLists.txt`, `build.bat` |

