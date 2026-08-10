# testing — 建置與驗證

← [WORKFLOWS.md](../WORKFLOWS.md)｜[INDEX.md](../INDEX.md)

## 指令

- 重新建置：`cmd /c build.bat`
- 自動模型測試：`ctest --test-dir build --output-on-failure`
- 執行互動 viewer：從 repo root 執行 `build\robot.exe`
- 檢查 dirty 範圍：`git status --short` 與相關 `git diff -- <paths>`

## 驗證原則

- `seven_cylinders_model` 會以 MuJoCo loader 載入模型，檢查 7 段／6 關節／6 actuator、交替 yaw/pitch 軸、零姿態連接座標、end-effector site，以及短步進後的有限數值。
- 其他 MJCF 修改也要以 MuJoCo loader 實際載入；必要時增加正式 headless smoke check，檢查模型尺寸、名稱、joint/site 座標與有限數值。
- viewer 是人工除錯工具。人工視覺驗證無法取代 headless 的數值測試；若本環境無法判斷外觀，將項目記入 [WAIT_USER.md](../WAIT_USER.md)。
