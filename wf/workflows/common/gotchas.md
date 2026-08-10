# 共通踩坑

← [common/README.md](README.md)｜[INDEX.md](../../INDEX.md)

- **MuJoCo cylinder 的長度是半長**：`geom type="cylinder" size="r h"` 中的 `h` 是沿軸半長；做串聯臂時，幾何中心與下一個 body 位置都要依完整長度計算。
- **執行目錄影響模型路徑**：目前模型以 `assets/...` 相對路徑載入，`build/robot.exe` 要從 repo root 執行。
- **既有 build 不是目前原始碼的證明**：先重新建置，再用新產物驗證；不要依賴 `build/` 的時間戳或舊執行檔。

